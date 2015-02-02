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

/** \file blender/modifiers/intern/MOD_setsplitnormal.c
 *  \ingroup modifiers
 */

#include <string.h>

#include "MEM_guardedalloc.h"

#include "DNA_object_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_mesh_types.h"

#include "BLI_math.h"
#include "BLI_utildefines.h"
#include "BLI_bitmap.h"
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


static void get_min_max_co(float (*cos)[3], const int num_verts, float r_min_co[3], float r_max_co[3])
{
	/* XXX Check we can't get this from object?! Don't think so (bbox does not account for DM/mod stack). */
	int i = num_verts;
	while (i--) {
		minmax_v3v3_v3(r_min_co, r_max_co, cos[i]);
	}
}

static void generate_vert_coordinates(DerivedMesh *dm, Object *ob, Object *ob_center, const bool use_bbox_center,
                                      const int num_verts, float (*r_cos)[3], float r_size[3])
{
	float min_co[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
	float max_co[3] = {FLT_MIN, FLT_MIN, FLT_MIN};
	float diff[3];
	bool do_diff = false;
	int i, j;

	dm->getVertCos(dm, r_cos);

	/* Compute min/max's, aka bbox (use target's ob one if available). */
	if (ob_center) {
		BKE_object_dimensions_get(ob_center, r_size);
		if (is_zero_v3(r_size)) {
			/* Use ob_center's size as fallback (when it's e.g. an empty...). */
			copy_v3_v3(r_size, ob_center->size);
		}
	}
	else {
		get_min_max_co(r_cos, num_verts, min_co, max_co);
		/* Set size. */
		sub_v3_v3v3(r_size, max_co, min_co);
	}

	/* Error checks - we do not want one or more of our sizes to be null! */
	if (is_zero_v3(r_size)) {
		r_size[0] = r_size[1] = r_size[2] = 1.0f;
	}
	else if (min_fff(r_size[0], r_size[1], r_size[2]) < FLT_EPSILON) {
		j = 3;
		while (j--) {
			if (r_size[j] < FLT_EPSILON) {
				r_size[j] = FLT_EPSILON;
			}
		}
	}

	if (ob_center) {
		/* Translate our coordinates so that center of ob_center is at (0, 0, 0). */
		float mat[4][4];

		/* Get ob_center coordinates in ob local coordinates. */
		invert_m4_m4(mat, ob_center->obmat);
		mul_m4_m4m4(mat, mat, ob->obmat);
		copy_v3_v3(diff, mat[3]);

		do_diff = true;
	}
	else if (use_bbox_center) {
		/* Translate our coordinates so that center of bounding box is at (0, 0, 0). */

		/* Compute bbox center in local coordinates. */
		add_v3_v3v3(diff, min_co, max_co);
		mul_v3_fl(diff, -0.5f);

		do_diff = true;
	}
	/* Else, no need to change coordinates! */

	if (do_diff) {
		i = num_verts;
		while (i--) {
			add_v3_v3(r_cos[i], diff);
		}
	}
}

/* Note this modifies nos_new in-place. */
static void mix_normals(
        const float mix_factor, MDeformVert *dvert, const int defgrp_index, const bool use_invert_vgroup,
        const short mix_mode,
        const int num_verts, MLoop *mloop, float (*nos_old)[3], float (*nos_new)[3], const int num_loops)
{
	/* Mix with org normals... */
	float *facs = NULL, *wfac;
	float (*no_new)[3], (*no_old)[3];
	int i;

	if (dvert) {
		facs = MEM_mallocN(sizeof(*facs) * (size_t)num_loops, __func__);
		BKE_defvert_extract_vgroup_to_loopweights(
		            dvert, defgrp_index, num_verts, mloop, num_loops, facs, use_invert_vgroup);
	}

	for (i = num_loops, no_new = nos_new, no_old = nos_old, wfac = facs; i--; no_new++, no_old++, wfac++) {
		const float fac = facs ? *wfac * mix_factor : mix_factor;

		switch (mix_mode) {
			case MOD_SETSPLITNORMAL_MIX_ADD:
				add_v3_v3(*no_new, *no_old);
				normalize_v3(*no_new);
				break;
			case MOD_SETSPLITNORMAL_MIX_SUB:
				sub_v3_v3(*no_new, *no_old);
				normalize_v3(*no_new);
				break;
			case MOD_SETSPLITNORMAL_MIX_MUL:
				mul_v3_v3(*no_new, *no_old);
				normalize_v3(*no_new);
				break;
			case MOD_SETSPLITNORMAL_MIX_COPY:
				break;
		}
		interp_v3_v3v3_slerp_safe(*no_new, *no_old, *no_new, fac);
	}

	MEM_SAFE_FREE(facs);
}

static void setSplitNormalModifier_do_ellipsoid(
        SetSplitNormalModifierData *smd, Object *ob, DerivedMesh *dm,
        short (*clnors)[2], float (*loopnors)[3], float (*polynors)[3],
        const short mix_mode, const float mix_factor,
        MDeformVert *dvert, const int defgrp_index, const bool use_invert_vgroup,
        MVert *mvert, const int num_verts, MEdge *medge, const int num_edges,
        MLoop *mloop, const int num_loops, MPoly *mpoly, const int num_polys)
{
	const bool use_bbox_center = ((smd->flags & MOD_SETSPLITNORMAL_CENTER_BBOX) != 0) && (smd->target == NULL);
	int i;

	float (*cos)[3] = MEM_mallocN(sizeof(*cos) * num_verts, __func__);
	float (*nos)[3] = MEM_mallocN(sizeof(*nos) * num_loops, __func__);
	float size[3];

	BLI_bitmap *done_verts = BLI_BITMAP_NEW((size_t)num_verts, __func__);

	generate_vert_coordinates(dm, ob, smd->target, use_bbox_center, num_verts, cos, size);

	/* size gives us our spheroid coefficients (A, B, C).
	 * Then, we want to find out for each vert its (a, b, c) triple (proportional to (A, B, C) one).
	 *
	 * Ellipsoid basic equation: (x^2/a^2) + (y^2/b^2) + (z^2/c^2) = 1.
	 * Since we want to find (a, b, c) matching this equation and proportional to (A, B, C), we can do:
	 *     m = B / A
	 *     n = C / A
	 * hence:
	 *     (x^2/a^2) + (y^2/b^2) + (z^2/c^2) = 1
	 *  -> b^2*c^2*x^2 + a^2*c^2*y^2 + a^2*b^2*z^2 = a^2*b^2*c^2
	 *     b = ma
	 *     c = na
	 *  -> m^2*a^2*n^2*a^2*x^2 + a^2*n^2*a^2*y^2 + a^2*m^2*a^2*z^2 = a^2*m^2*a^2*n^2*a^2
	 *  -> m^2*n^2*a^4*x^2 + n^2*a^4*y^2 + m^2*a^4*z^2 = m^2*n^2*a^6
	 *  -> a^2 = (m^2*n^2*x^2 + n^2y^2 + m^2z^2) / (m^2*n^2) = x^2 + (y^2 / m^2) + (z^2 / n^2)
	 *  -> b^2 = (m^2*n^2*x^2 + n^2y^2 + m^2z^2) / (n^2)     = (m^2 * x^2) + y^2 + (m^2 * z^2 / n^2)
	 *  -> c^2 = (m^2*n^2*x^2 + n^2y^2 + m^2z^2) / (m^2)     = (n^2 * x^2) + (n^2 * y^2 / m^2) + z^2
	 *
	 * All we have to do now is compute normal of the spheroid at that point:
	 *     n = (x / a^2, y / b^2, z / c^2)
	 * And we are done!
	 */
	{
		const float A = size[0], B = size[1], C = size[2];
		const float m2 = (B * B) / (A * A);
		const float n2 = (C * C) / (A * A);

		MLoop *ml;
		float (*no)[3];

		/* We reuse cos to now store the ellipsoid-normal of the verts! */
		for (i = num_loops, ml = mloop, no = nos; i-- ; ml++, no++) {
			const int vidx = ml->v;
			float *co = cos[vidx];

			if (!BLI_BITMAP_TEST(done_verts, vidx)) {
				const float x2 = co[0] * co[0];
				const float y2 = co[1] * co[1];
				const float z2 = co[2] * co[2];
				const float a2 = x2 + (y2 / m2) + (z2 / n2);
				const float b2 = (m2 * x2) + y2 + (m2 * z2 / n2);
				const float c2 = (n2 * x2) + (n2 * y2 / m2) + z2;

				co[0] /= a2;
				co[1] /= b2;
				co[2] /= c2;
				normalize_v3(co);

				BLI_BITMAP_ENABLE(done_verts, vidx);
			}
			copy_v3_v3(*no, co);
		}
	}

	if (loopnors) {
		mix_normals(mix_factor, dvert, defgrp_index, use_invert_vgroup,
		            mix_mode, num_verts, mloop, loopnors, nos, num_loops);
	}

	BKE_mesh_normals_loop_custom_set(mvert, num_verts, medge, num_edges, mloop, nos, num_loops,
	                                 mpoly, (const float(*)[3])polynors, num_polys, clnors);

	MEM_freeN(cos);
	MEM_freeN(nos);
	MEM_freeN(done_verts);
}

static void setSplitNormalModifier_do_trackto(
        SetSplitNormalModifierData *smd, Object *ob, DerivedMesh *dm,
        short (*clnors)[2], float (*loopnors)[3], float (*polynors)[3],
        const short mix_mode, const float mix_factor,
        MDeformVert *dvert, const int defgrp_index, const bool use_invert_vgroup,
        MVert *mvert, const int num_verts, MEdge *medge, const int num_edges,
        MLoop *mloop, const int num_loops, MPoly *mpoly, const int num_polys)
{
	const bool use_parallel_normals = (smd->flags & MOD_SETSPLITNORMAL_USE_PARALLEL_TRACKTO) != 0;
	const bool use_bbox_center = (smd->flags & MOD_SETSPLITNORMAL_CENTER_BBOX) != 0;

	float (*cos)[3] = MEM_mallocN(sizeof(*cos) * num_verts, __func__);
	float (*nos)[3] = MEM_mallocN(sizeof(*nos) * num_loops, __func__);

	float target_co[3];
	int i;

	dm->getVertCos(dm, cos);

	/* Get target's center coordinates in ob local coordinates. */
	{
		float mat[4][4];

		invert_m4_m4(mat, ob->obmat);
		mul_m4_m4m4(mat, mat, smd->target->obmat);
		copy_v3_v3(target_co, mat[3]);
	}

	if (use_parallel_normals) {
		float no[3];

		if (use_bbox_center) {
			float min_co[3], max_co[3];

			/* We use bbox center as ref, instead of object's center (i.e. (0, 0, 0) in local space). */
			get_min_max_co(cos, num_verts, min_co, max_co);
			madd_v3_v3v3fl(no, min_co, max_co, 0.5f);
			sub_v3_v3v3(no, target_co, no);
			normalize_v3(no);
		}
		else {
			normalize_v3_v3(no, target_co);
		}

		for (i = num_loops; i--; ) {
			copy_v3_v3(nos[i], no);
		}
	}
	else {
		BLI_bitmap *done_verts = BLI_BITMAP_NEW((size_t)num_verts, __func__);
		MLoop *ml;
		float (*no)[3];

		/* We reuse cos to now store the 'to target' normal of the verts! */
		for (i = num_loops, no = nos, ml = mloop; i--; no++, ml++) {
			const int vidx = ml->v;
			float *co = cos[vidx];

			if (!BLI_BITMAP_TEST(done_verts, vidx)) {
				sub_v3_v3v3(co, target_co, co);
				normalize_v3(co);

				BLI_BITMAP_ENABLE(done_verts, vidx);
			}

			copy_v3_v3(*no, co);
		}

		MEM_freeN(done_verts);
	}

	if (loopnors) {
		mix_normals(mix_factor, dvert, defgrp_index, use_invert_vgroup,
		            mix_mode, num_verts, mloop, loopnors, nos, num_loops);
	}

	BKE_mesh_normals_loop_custom_set(mvert, num_verts, medge, num_edges, mloop, nos, num_loops,
	                                 mpoly, (const float(*)[3])polynors, num_polys, clnors);

	MEM_freeN(cos);
	MEM_freeN(nos);
}

static bool is_valid_target(SetSplitNormalModifierData *smd)
{
	if (smd->mode == MOD_SETSPLITNORMAL_MODE_ELLIPSOID) {
		return true;
	}
	else if ((smd->mode == MOD_SETSPLITNORMAL_MODE_TRACKTO) && smd->target) {
		return true;
	}
	modifier_setError((ModifierData *)smd, "Invalid target settings");
	return false;
}

static void setSplitNormalModifier_do(SetSplitNormalModifierData *smd, Object *ob, DerivedMesh *dm)
{
	Mesh *me = ob->data;

	const int num_verts = dm->getNumVerts(dm);
	const int num_edges = dm->getNumEdges(dm);
	const int num_loops = dm->getNumLoops(dm);
	const int num_polys = dm->getNumPolys(dm);
	MVert *mvert = dm->getVertArray(dm);
	MEdge *medge = dm->getEdgeArray(dm);
	MLoop *mloop = dm->getLoopArray(dm);
	MPoly *mpoly = dm->getPolyArray(dm);

	const bool use_invert_vgroup = ((smd->flags & MOD_SETSPLITNORMAL_INVERT_VGROUP) != 0);
	const bool use_current_clnors = (smd->flags & MOD_SETSPLITNORMAL_USE_CURCLNORS) != 0;

	int defgrp_index;
	MDeformVert *dvert;

	float (*loopnors)[3] = NULL;
	short (*clnors)[2];

	float (*polynors)[3];
	bool free_polynors = false;

	/* Do not run that modifier at all if autosmooth is disabled! */
	if (!is_valid_target(smd) || !num_loops) {
		return;
	}

	if (!(me->flag & ME_AUTOSMOOTH)) {
		modifier_setError((ModifierData *)smd, "Enable 'Auto Smooth' option in mesh settings");
		return;
	}


	if (use_current_clnors) {
		dm->calcLoopNormals(dm, (me->flag & ME_AUTOSMOOTH) != 0, me->smoothresh);
		loopnors = dm->getLoopDataArray(dm, CD_NORMAL);
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

	if (smd->mode == MOD_SETSPLITNORMAL_MODE_ELLIPSOID) {
		setSplitNormalModifier_do_ellipsoid(
		            smd, ob, dm, clnors, loopnors, polynors,
		            smd->mix_mode, smd->mix_factor, dvert, defgrp_index, use_invert_vgroup,
		            mvert, num_verts, medge, num_edges, mloop, num_loops, mpoly, num_polys);
	}
	else if (smd->mode == MOD_SETSPLITNORMAL_MODE_TRACKTO) {
		setSplitNormalModifier_do_trackto(
		            smd, ob, dm, clnors, loopnors, polynors,
		            smd->mix_mode, smd->mix_factor, dvert, defgrp_index, use_invert_vgroup,
		            mvert, num_verts, medge, num_edges, mloop, num_loops, mpoly, num_polys);
	}

	if (free_polynors) {
		MEM_freeN(polynors);
	}
}

static void initData(ModifierData *md)
{
	SetSplitNormalModifierData *smd = (SetSplitNormalModifierData *)md;

	smd->mode = MOD_SETSPLITNORMAL_MODE_ELLIPSOID;
	smd->flags = MOD_SETSPLITNORMAL_USE_CURCLNORS;

	smd->mix_mode = MOD_SETSPLITNORMAL_MIX_COPY;
	smd->mix_factor = 1.0f;
}

static void copyData(ModifierData *md, ModifierData *target)
{
	modifier_copyData_generic(md, target);
}

static CustomDataMask requiredDataMask(Object *UNUSED(ob), ModifierData *md)
{
	SetSplitNormalModifierData *smd = (SetSplitNormalModifierData *)md;
	CustomDataMask dataMask = CD_CUSTOMLOOPNORMAL;

	/* Ask for vertexgroups if we need them. */
	if (smd->defgrp_name[0]) {
		dataMask |= (CD_MASK_MDEFORMVERT);
	}

	return dataMask;
}

static bool dependsOnNormals(ModifierData *UNUSED(md))
{
	return true;
}

static void foreachObjectLink(ModifierData *md, Object *ob, ObjectWalkFunc walk, void *userData)
{
	SetSplitNormalModifierData *smd = (SetSplitNormalModifierData *) md;

	walk(userData, ob, &smd->target);
}

static void foreachIDLink(ModifierData *md, Object *ob, IDWalkFunc walk, void *userData)
{
	SetSplitNormalModifierData *smd = (SetSplitNormalModifierData *) md;

	walk(userData, ob, (ID **)&smd->target);
}

static bool isDisabled(ModifierData *md, int UNUSED(useRenderParams))
{
	SetSplitNormalModifierData *smd = (SetSplitNormalModifierData *)md;

	return !is_valid_target(smd);
}

static void updateDepgraph(ModifierData *md, DagForest *forest, struct Scene *UNUSED(scene),
                           Object *UNUSED(ob), DagNode *obNode)
{
	SetSplitNormalModifierData *smd = (SetSplitNormalModifierData *) md;

	if (smd->target) {
		DagNode *Node = dag_get_node(forest, smd->target);

		dag_add_relation(forest, Node, obNode, DAG_RL_DATA_DATA | DAG_RL_OB_DATA, "SetSplitNormal Modifier");
	}
}

static DerivedMesh *applyModifier(ModifierData *md, Object *ob, DerivedMesh *dm, ModifierApplyFlag UNUSED(flag))
{
	setSplitNormalModifier_do((SetSplitNormalModifierData *)md, ob, dm);
	return dm;
}

ModifierTypeInfo modifierType_SetSplitNormal = {
	/* name */              "Set Split Normals",
	/* structName */        "SetSplitNormalModifierData",
	/* structSize */        sizeof(SetSplitNormalModifierData),
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
