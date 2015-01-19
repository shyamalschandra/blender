/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software  Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contributor(s): Bastien Montagne
 *
 * ***** END GPL LICENSE BLOCK *****
 *
 */

/** \file blender/modifiers/intern/MOD_setsplitnor.c
 *  \ingroup modifiers
 */

#include <string.h>

#include "MEM_guardedalloc.h"

#include "DNA_object_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_mesh_types.h"

#include "BLI_math.h"
#include "BLI_utildefines.h"
#include "BLI_linklist.h"
#include "BLI_string.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_mesh.h"
#include "BKE_modifier.h"
#include "BKE_object.h"
#include "BKE_deform.h"

#include "depsgraph_private.h"

#include "RE_shader_ext.h"

#include "MOD_util.h"


static bool ensure_target_dm(Object *target_ob, DerivedMesh **r_target_dm)
{
	*r_target_dm = target_ob->derivedFinal;
	if (!*r_target_dm) {
#if 0
		if (ELEM(target_ob->type, OB_CURVE, OB_SURF, OB_FONT)) {
			*r_target_dm = CDDM_from_curve(target_ob);
			return true;
		}
		else if (target_ob->type == OB_MESH) {
#else
		if (target_ob->type == OB_MESH) {
#endif
			Mesh *me = (Mesh *)target_ob->data;
			if (me->edit_btmesh) {
				*r_target_dm = CDDM_from_editbmesh(me->edit_btmesh, false, false);
				return true;
			}
			else {
				*r_target_dm = CDDM_from_mesh(me);
				return true;
			}
		}
	}
	return false;
}

static float get_weight(MDeformVert *dvert, const int defgrp_index, const bool use_invert_vgroup, const int vidx)
{
	if (!dvert || defgrp_index == -1) {
		return 1.0f;
	}
	else {
		const float weight = defvert_find_weight(&dvert[vidx], defgrp_index);
		return use_invert_vgroup ? 1.0f - weight : weight;
	}
}

static void copySplitNormalModifier_do_facenormal(
        CopySplitNormalModifierData *smd, Object *ob, DerivedMesh *dm,
        short (*clnors)[2], float (*polynors)[3],
        MDeformVert *dvert, const int defgrp_index, const bool use_invert_vgroup,
        MVert *mvert, const int num_verts, MEdge *medge, const int num_edges,
        MLoop *mloop, const int num_loops, MPoly *mpoly, const int num_polys)
{
	Object *target_ob = smd->target;
	DerivedMesh *target_dm;
	const bool free_target_dm = ensure_target_dm(target_ob, &target_dm);
	BVHTreeFromMesh treeData = {0};

	const bool use_current_clnors = (smd->flags & MOD_COPYSPLITNORMAL_USE_CURCLNORS) != 0;

	float (*cos)[3] = MEM_mallocN(sizeof(*cos) * num_verts, __func__);
	float *facs = MEM_mallocN(sizeof(*facs) * num_verts, __func__);

	/* Create a bvh-tree of the given target's faces. */
	bvhtree_from_mesh_faces(&treeData, target_dm, 0.0, 2, 6);
	if (treeData.tree != NULL) {
		const int target_num_polys = target_dm->getNumPolys(target_dm);
		BVHTreeNearest nearest = {0};

		float (*target_polynors)[3];
		bool free_target_polynors = false;

		int i;

		SpaceTransform loc2trgt;
		BLI_SPACE_TRANSFORM_SETUP(&loc2trgt, ob, target_ob);

		target_polynors = target_dm->getPolyDataArray(target_dm, CD_NORMAL);
		if (!target_polynors) {
			const int target_num_verts = target_dm->getNumVerts(target_dm);
			const int target_num_loops = target_dm->getNumLoops(target_dm);
			MVert *target_mvert = target_dm->getVertArray(target_dm);
			MLoop *target_mloop = target_dm->getLoopArray(target_dm);
			MPoly *target_mpoly = target_dm->getPolyArray(target_dm);

			target_polynors = MEM_mallocN(sizeof(*target_polynors) * target_num_polys, __func__);
			BKE_mesh_calc_normals_poly(target_mvert, target_num_verts, target_mloop, target_mpoly,
			                           target_num_loops, target_num_polys, target_polynors, false);
			free_target_polynors = true;
		}

		nearest.index = -1;
		dm->getVertCos(dm, cos);

		/* Find the nearest face. */
#ifndef __APPLE__
#pragma omp parallel for default(none) private(i) firstprivate(nearest) \
                         shared(treeData, cos, facs, target_polynors, loc2trgt, dvert) \
                         schedule(static)
#endif
		for (i = 0; i < num_verts; i++) {
			float tmp_co[3];

			facs[i] = get_weight(dvert, defgrp_index, use_invert_vgroup, i);

			/* Convert the vertex to tree coordinates. */
			copy_v3_v3(tmp_co, cos[i]);
			BLI_space_transform_apply(&loc2trgt, tmp_co);

			/* Use local proximity heuristics (to reduce the nearest search).
			 *
			 * If we already had an hit before, we assume this vertex is going to have a close hit to
			 * that other vertex, so we can initiate the "nearest.dist" with the expected value to that
			 * last hit.
			 * This will lead in prunning of the search tree.
			 */
			nearest.dist_sq = (nearest.index != -1) ? len_squared_v3v3(tmp_co, nearest.co) : FLT_MAX;
			/* Compute and store result. */
			BLI_bvhtree_find_nearest(treeData.tree, tmp_co, &nearest, treeData.nearest_callback, &treeData);

			/* XXX This is broken in polygon case - nearest.index is index of nearest tessellated face, not nearest poly!
			 *     Not worth fixing this here, better to wait for transferdata code, which handles all that mess itself.
			 */

			if (facs[i] && nearest.index != -1) {
				copy_v3_v3(cos[i], target_polynors[nearest.index]);
				/* Bring normal back in own space! */
				BLI_space_transform_invert_normal(&loc2trgt, cos[i]);
			}
			else {
				zero_v3(cos[i]);
			}
		}

		free_bvhtree_from_mesh(&treeData);
		if (free_target_polynors) {
			MEM_freeN(target_polynors);
		}

		BKE_mesh_normals_loop_custom_from_vertices_set(mvert, cos, facs, num_verts, medge, num_edges, mloop, num_loops,
		                                               mpoly, (const float(*)[3])polynors, num_polys,
		                                               clnors, use_current_clnors);
	}

	MEM_freeN(cos);
	MEM_freeN(facs);
	if (target_dm && free_target_dm) {
		target_dm->release(target_dm);
	}
}

static void copySplitNormalModifier_do_loopnormal(
        CopySplitNormalModifierData *smd, Object *ob, DerivedMesh *dm,
        short (*clnors)[2], float (*polynors)[3],
        MDeformVert *dvert, const int defgrp_index, const bool use_invert_vgroup,
        MVert *mvert, const int num_verts, MEdge *medge, const int num_edges,
        MLoop *mloop, const int num_loops, MPoly *mpoly, const int num_polys)
{
#if 0
	Object *target_ob = smd->target;
	DerivedMesh *target_dm;
	const bool free_target_dm = ensure_target_dm(target_ob, &target_dm);
	BVHTreeFromMesh treeData = {0};

	const bool use_current_clnors = (smd->flags & MOD_COPYSPLITNORMAL_USE_CURCLNORS) != 0;

	float (*cos)[3] = MEM_mallocN(sizeof(*cos) * num_verts, __func__);
	float (*nos)[3] = MEM_callocN(sizeof(*nos) * num_loops, __func__);
	float *facs = MEM_mallocN(sizeof(*facs) * num_loops, __func__);
	float *vfacs = MEM_mallocN(sizeof(*facs) * num_verts, __func__);

	/* Create a bvh-tree of the given target's vertices. */
	bvhtree_from_mesh_verts(&treeData, target_dm, 0.0, 2, 6);
	if (treeData.tree != NULL) {
		BVHTreeNearest nearest = {0};
		int *loop_to_poly = MEM_mallocN(sizeof(int) * (size_t)num_loops, __func__);

		const int target_num_verts = target_dm->getNumVerts(target_dm);
		const int target_num_edges = target_dm->getNumEdges(target_dm);
		const int target_num_polys = target_dm->getNumPolys(target_dm);
		const int target_num_loops = target_dm->getNumLoops(target_dm);
		MVert *target_mvert = target_dm->getVertArray(target_dm);
		MEdge *target_medge = target_dm->getEdgeArray(target_dm);
		MLoop *target_mloop = target_dm->getLoopArray(target_dm);
		MPoly *target_mpoly = target_dm->getPolyArray(target_dm);
		const float target_split_angle = ((Mesh *)(target_ob->data))->smoothresh;

		float (*target_lnors)[3] = MEM_callocN(sizeof(*target_lnors) * (size_t)target_num_loops, __func__);
		float (*target_polynors)[3];
		bool free_target_polynors = false;
		short (*target_clnors)[2] = dm->getLoopDataArray(target_dm, CD_CUSTOMLOOPNORMAL);
		int *target_loop_to_poly = MEM_mallocN(sizeof(int) * (size_t)target_num_loops, __func__);
		LinkNode *target_verts2loops_pool = MEM_callocN(sizeof(*target_verts2loops_pool) * (size_t)target_num_loops,
		                                                __func__);
		LinkNode **target_verts2loops = MEM_callocN(sizeof(*target_verts2loops) * (size_t)target_num_verts, __func__);

		int i;

		SpaceTransform loc2trgt;
		BLI_SPACE_TRANSFORM_SETUP(&loc2trgt, ob, target_ob);

		/* ***** Populate ob's needed data. ***** */

		for (i = 0; i < num_polys; i++) {
			MPoly *mp = &mpoly[i];
			int j = mp->loopstart;
			const int max = j + mp->totloop;

			for (; j < max; j++) {
				loop_to_poly[j] = i;
			}
		}

		/* ***** Populate target's needed data. ***** */

		target_polynors = target_dm->getPolyDataArray(target_dm, CD_NORMAL);
		if (!target_polynors) {
			target_polynors = MEM_mallocN(sizeof(*target_polynors) * target_num_polys, __func__);
			BKE_mesh_calc_normals_poly(target_mvert, target_num_verts, target_mloop, target_mpoly,
			                           target_num_loops, target_num_polys, target_polynors, false);
			free_target_polynors = true;
		}

		BKE_mesh_normals_loop_split(target_mvert, target_num_verts, target_medge, target_num_edges,
		                            target_mloop, target_lnors, target_num_loops,
		                            target_mpoly, (const float (*)[3])target_polynors, target_num_polys,
		                            target_split_angle, NULL, target_clnors, target_loop_to_poly);

		/* Build our target 'vertices to loops' mapping. */
		for (i = 0; i < target_num_loops; i++) {
			LinkNode *lnk = &target_verts2loops_pool[i];
			const int vidx = target_mloop[i].v;

			BLI_linklist_prepend_nlink(&target_verts2loops[vidx], SET_INT_IN_POINTER(i), lnk);
		}

		nearest.index = -1;
		dm->getVertCos(dm, cos);

		/* Find (match) the nearest vertices. */
#ifndef __APPLE__
#pragma omp parallel for default(none) private(i) firstprivate(nearest) \
                         shared(treeData, cos, vfacs, target_polynors, target_verts2loops, loc2trgt, dvert) \
                         schedule(static)
#endif
		for (i = 0; i < num_verts; i++) {
			float tmp_co[3];

			vfacs[i] = get_weight(dvert, defgrp_index, use_invert_vgroup, i);

			/* Convert the vertex to tree coordinates. */
			copy_v3_v3(tmp_co, cos[i]);
			BLI_space_transform_apply(&loc2trgt, tmp_co);

			/* Use local proximity heuristics (to reduce the nearest search).
			 *
			 * If we already had an hit before, we assume this vertex is going to have a close hit to
			 * that other vertex, so we can initiate the "nearest.dist" with the expected value to that
			 * last hit.
			 * This will lead in prunning of the search tree.
			 */
			nearest.dist_sq = (nearest.index != -1) ? len_squared_v3v3(tmp_co, nearest.co) : FLT_MAX;
			/* Compute and store result. */
			BLI_bvhtree_find_nearest(treeData.tree, tmp_co, &nearest, treeData.nearest_callback, &treeData);

			/* In case found closest vert has no loops associated... */
			cos[i][0] = (target_verts2loops[nearest.index] == NULL) ? -1.0f : (float)nearest.index;
		}

		/* And now, match all loops together, based on their respective faces' normals. */
		for (i = 0; i < num_loops; i++) {
			const MLoop *ml = &mloop[i];
			float pnor[3];
			const int target_vidx = (int)cos[ml->v][0];

			LinkNode *target_ml_lnk;
			float target_ml_best_dot = -1.1f;
			int target_lidx = -1;
			const float fac = facs[i] = vfacs[ml->v];

			/* Move our poly normal in target space! */
			copy_v3_v3(pnor, polynors[loop_to_poly[i]]);
			BLI_space_transform_apply_normal(&loc2trgt, pnor);

			if (target_vidx < 0) {
				/* nos is calloc'ed, no need to zero_v3 here. */
				continue;
			}

			for (target_ml_lnk = target_verts2loops[target_vidx]; target_ml_lnk; target_ml_lnk = target_ml_lnk->next) {
				const int t_lidx = GET_INT_FROM_POINTER(target_ml_lnk->link);
				const float t_dot = dot_v3v3(pnor, target_polynors[target_loop_to_poly[t_lidx]]);

				if (t_dot > target_ml_best_dot) {
					target_ml_best_dot = t_dot;
					target_lidx = t_lidx;
				}
			}

			/* XXX This will happen if the closest vert has no face (i.e. no loop).
			 *     Maybe we should use a custom bvh tree func to exclude such verts from our search?
			 */
			if (target_lidx < 0) {
				continue;
			}

			if (fac) {
				copy_v3_v3(nos[i], target_lnors[target_lidx]);
				/* Bring normal back in own space! */
				BLI_space_transform_invert_normal(&loc2trgt, nos[i]);
			}
			/* No else, since we calloc nos, no need to set it to zero vec... */
		}

		BKE_mesh_normals_loop_custom_set(mvert, num_verts, medge, num_edges, mloop, nos, facs, num_loops,
		                                 mpoly, (const float(*)[3])polynors, num_polys, clnors, use_current_clnors);

		free_bvhtree_from_mesh(&treeData);
		MEM_freeN(loop_to_poly);

		MEM_freeN(target_lnors);
		if (free_target_polynors) {
			MEM_freeN(target_polynors);
		}
		MEM_freeN(target_loop_to_poly);
		MEM_freeN(target_verts2loops_pool);
		MEM_freeN(target_verts2loops);
	}

	MEM_freeN(cos);
	MEM_freeN(nos);
	MEM_freeN(facs);
	MEM_freeN(vfacs);
	if (target_dm && free_target_dm) {
		target_dm->release(target_dm);
	}
#endif
}

static bool is_valid_target(CopySplitNormalModifierData *smd)
{
	if (ELEM(smd->mode, MOD_COPYSPLITNORMAL_MODE_GEOM_FACENOR, MOD_COPYSPLITNORMAL_MODE_GEOM_LOOPNOR) &&
	    smd->target && smd->target->type == OB_MESH)
	{
		return true;
	}
	return false;
}

static void copySplitNormalModifier_do(CopySplitNormalModifierData *smd, Object *ob, DerivedMesh *dm)
{
	const int num_verts = dm->getNumVerts(dm);
	const int num_edges = dm->getNumEdges(dm);
	const int num_loops = dm->getNumLoops(dm);
	const int num_polys = dm->getNumPolys(dm);
	MVert *mvert = dm->getVertArray(dm);
	MEdge *medge = dm->getEdgeArray(dm);
	MLoop *mloop = dm->getLoopArray(dm);
	MPoly *mpoly = dm->getPolyArray(dm);

	const bool use_invert_vgroup = ((smd->flags & MOD_COPYSPLITNORMAL_INVERT_VGROUP) != 0);

	int defgrp_index;
	MDeformVert *dvert;

	short (*clnors)[2];

	float (*polynors)[3];
	bool free_polynors = false;

	if (!is_valid_target(smd)) {
		return;
	}

	clnors = CustomData_duplicate_referenced_layer(&dm->loopData, CD_CUSTOMLOOPNORMAL, num_loops);
	if (!clnors) {
		DM_add_loop_layer(dm, CD_CUSTOMLOOPNORMAL, CD_CALLOC, NULL);
		clnors = dm->getLoopDataArray(dm, CD_CUSTOMLOOPNORMAL);
	}

	polynors = dm->getPolyDataArray(dm, CD_NORMAL);
	if (!polynors) {
		polynors = MEM_mallocN(sizeof(*polynors) * num_polys, __func__);
		BKE_mesh_calc_normals_poly(mvert, num_verts, mloop, mpoly, num_loops, num_polys, polynors, false);
		free_polynors = true;
	}

	modifier_get_vgroup(ob, dm, smd->defgrp_name, &dvert, &defgrp_index);

	if (smd->mode == MOD_COPYSPLITNORMAL_MODE_GEOM_FACENOR) {
		copySplitNormalModifier_do_facenormal(smd, ob, dm, clnors, polynors, dvert, defgrp_index, use_invert_vgroup,
		                                     mvert, num_verts, medge, num_edges, mloop, num_loops, mpoly, num_polys);
	}
	else if (smd->mode == MOD_COPYSPLITNORMAL_MODE_GEOM_LOOPNOR) {
		copySplitNormalModifier_do_loopnormal(smd, ob, dm, clnors, polynors, dvert, defgrp_index, use_invert_vgroup,
		                                     mvert, num_verts, medge, num_edges, mloop, num_loops, mpoly, num_polys);
	}

	if (free_polynors) {
		MEM_freeN(polynors);
	}
}

static void initData(ModifierData *md)
{
	CopySplitNormalModifierData *smd = (CopySplitNormalModifierData *)md;

	smd->mode = MOD_COPYSPLITNORMAL_MODE_GEOM_FACENOR;
	smd->flags = MOD_COPYSPLITNORMAL_USE_CURCLNORS;
}

static void copyData(ModifierData *md, ModifierData *target)
{
	modifier_copyData_generic(md, target);
}

static CustomDataMask requiredDataMask(Object *UNUSED(ob), ModifierData *md)
{
	CopySplitNormalModifierData *smd = (CopySplitNormalModifierData *)md;
	CustomDataMask dataMask = CD_CUSTOMLOOPNORMAL;

	/* Ask for vertexgroups if we need them. */
	if (smd->defgrp_name[0]) {
		dataMask |= (CD_MASK_MDEFORMVERT);
	}

	return dataMask;
}

static bool dependsOnNormals(ModifierData *UNUSED(md)) {
	return true;
}

static void foreachObjectLink(ModifierData *md, Object *ob, ObjectWalkFunc walk, void *userData)
{
	CopySplitNormalModifierData *smd = (CopySplitNormalModifierData *) md;

	walk(userData, ob, &smd->target);
}

static void foreachIDLink(ModifierData *md, Object *ob, IDWalkFunc walk, void *userData)
{
	CopySplitNormalModifierData *smd = (CopySplitNormalModifierData *) md;

	walk(userData, ob, (ID **)&smd->target);
}

static bool isDisabled(ModifierData *md, int UNUSED(useRenderParams))
{
	CopySplitNormalModifierData *smd = (CopySplitNormalModifierData *)md;

	return !is_valid_target(smd);
}

static void updateDepgraph(ModifierData *md, DagForest *forest, struct Scene *UNUSED(scene),
                           Object *UNUSED(ob), DagNode *obNode)
{
	CopySplitNormalModifierData *smd = (CopySplitNormalModifierData *) md;

	if (smd->target) {
		DagNode *Node = dag_get_node(forest, smd->target);

		dag_add_relation(forest, Node, obNode, DAG_RL_DATA_DATA | DAG_RL_OB_DATA, "CopySplitNormal Modifier");
	}
}

static DerivedMesh *applyModifier(ModifierData *md, Object *ob, DerivedMesh *dm, ModifierApplyFlag UNUSED(flag))
{
	copySplitNormalModifier_do((CopySplitNormalModifierData *)md, ob, dm);
	return dm;
}

ModifierTypeInfo modifierType_CopySplitNormal = {
	/* name */              "Set Split Normals",
	/* structName */        "CopySplitNormalModifierData",
	/* structSize */        sizeof(CopySplitNormalModifierData),
	/* type */              eModifierTypeType_Constructive,
	/* flags */             eModifierTypeFlag_AcceptsMesh |
	                        eModifierTypeFlag_AcceptsCVs |
	                        eModifierTypeFlag_SupportsMapping |
	                        eModifierTypeFlag_SupportsEditmode |
	                        eModifierTypeFlag_EnableInEditmode,
	/* copyData */          copyData,
	/* deformVerts */       NULL,
	/* deformMatrices */    NULL,
	/* deformVertsEM */     NULL,
	/* deformMatricesEM */  NULL,
	/* applyModifier */     applyModifier,
	/* applyModifierEM */   NULL,
	/* initData */          initData,
	/* requiredDataMask */  requiredDataMask,
	/* freeData */          NULL,
	/* isDisabled */        isDisabled,
	/* updateDepgraph */    updateDepgraph,
	/* dependsOnTime */     NULL,
	/* dependsOnNormals */  dependsOnNormals,
	/* foreachObjectLink */ foreachObjectLink,
	/* foreachIDLink */     foreachIDLink,
	/* foreachTexLink */    NULL,
};
