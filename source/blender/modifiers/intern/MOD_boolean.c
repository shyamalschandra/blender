
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
 * The Original Code is Copyright (C) 2005 by the Blender Foundation.
 * All rights reserved.
 *
 * Contributor(s): Daniel Dunbar
 *                 Ton Roosendaal,
 *                 Ben Batt,
 *                 Brecht Van Lommel,
 *                 Campbell Barton
 *
 * ***** END GPL LICENSE BLOCK *****
 *
 */

/** \file blender/modifiers/intern/MOD_boolean.c
 *  \ingroup modifiers
 */

#ifdef WITH_MOD_BOOLEAN
#  define USE_BMESH
#endif

#include <stdio.h>

#include "DNA_object_types.h"

#include "BLI_utildefines.h"

#include "BLF_translation.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_modifier.h"

#include "depsgraph_private.h"

#include "MOD_boolean_util.h"
#include "MOD_util.h"

#include "PIL_time.h"
#include "PIL_time_utildefines.h"

#ifdef USE_BMESH
#include "BLI_math_geom.h"
#include "MEM_guardedalloc.h"

#include "bmesh.h"
#include "bmesh_tools.h"
#include "tools/bmesh_intersect.h"
#endif

static void copyData(ModifierData *md, ModifierData *target)
{
#if 0
	BooleanModifierData *bmd = (BooleanModifierData *) md;
	BooleanModifierData *tbmd = (BooleanModifierData *) target;
#endif
	modifier_copyData_generic(md, target);
}

static bool isDisabled(ModifierData *md, int UNUSED(useRenderParams))
{
	BooleanModifierData *bmd = (BooleanModifierData *) md;

	return !bmd->object;
}

static void foreachObjectLink(
        ModifierData *md, Object *ob,
        void (*walk)(void *userData, Object *ob, Object **obpoin),
        void *userData)
{
	BooleanModifierData *bmd = (BooleanModifierData *) md;

	walk(userData, ob, &bmd->object);
}

static void updateDepgraph(ModifierData *md, DagForest *forest,
                           struct Scene *UNUSED(scene),
                           Object *UNUSED(ob),
                           DagNode *obNode)
{
	BooleanModifierData *bmd = (BooleanModifierData *) md;

	if (bmd->object) {
		DagNode *curNode = dag_get_node(forest, bmd->object);

		dag_add_relation(forest, curNode, obNode,
		                 DAG_RL_DATA_DATA | DAG_RL_OB_DATA, "Boolean Modifier");
	}
}

#ifdef WITH_MOD_BOOLEAN
static DerivedMesh *get_quick_derivedMesh(DerivedMesh *derivedData, DerivedMesh *dm, int operation)
{
	DerivedMesh *result = NULL;

	if (derivedData->getNumPolys(derivedData) == 0 || dm->getNumPolys(dm) == 0) {
		switch (operation) {
			case eBooleanModifierOp_Intersect:
				result = CDDM_new(0, 0, 0, 0, 0);
				break;

			case eBooleanModifierOp_Union:
				if (derivedData->getNumPolys(derivedData)) result = derivedData;
				else result = CDDM_copy(dm);

				break;

			case eBooleanModifierOp_Difference:
				result = derivedData;
				break;
		}
	}

	return result;
}

#ifdef USE_BMESH

struct BMIsectUserData {
	int face_tot_first_mesh;
};

/**
 * Compare selected/unselected.
 */
static int bm_face_isect_pair(BMFace *f, void *user_data)
{
	struct BMIsectUserData *data = user_data;

	return (BM_elem_index_get(f) < data->face_tot_first_mesh);
}

static DerivedMesh *applyModifier(ModifierData *md, Object *ob,
                                  DerivedMesh *derivedData,
                                  ModifierApplyFlag flag)
{
	BooleanModifierData *bmd = (BooleanModifierData *) md;
	DerivedMesh *dm;

	if (!bmd->object)
		return derivedData;

	dm = get_dm_for_modifier(bmd->object, flag);

	if (dm) {
		DerivedMesh *result;

		/* when one of objects is empty (has got no faces) we could speed up
		 * calculation a bit returning one of objects' derived meshes (or empty one)
		 * Returning mesh is depended on modifiers operation (sergey) */
		result = get_quick_derivedMesh(derivedData, dm, bmd->operation);

		if (result == NULL) {
			BMesh *bm;
			const BMAllocTemplate allocsize = BMALLOC_TEMPLATE_FROM_DM(derivedData, dm);
			(void)ob;

			TIMEIT_START(NewBooleanDerivedMesh);
			bm = BM_mesh_create(&allocsize);

			DM_to_bmesh_ex(derivedData, bm, true);
			DM_to_bmesh_ex(dm, bm, true);


			if (1) {
				/* create tessface & intersect */
				const int looptris_tot = poly_to_tri_count(bm->totface, bm->totloop);
				int tottri;
				BMLoop *(*looptris)[3];

				struct BMIsectUserData user_data = {0};

				looptris = MEM_mallocN(sizeof(*looptris) * looptris_tot, __func__);

				BM_bmesh_calc_tessellation(bm, looptris, &tottri);

				user_data.face_tot_first_mesh = dm->getNumPolys(dm);

				BM_mesh_intersect(
				        bm,
				        looptris, tottri,
				        bm_face_isect_pair, &user_data,
				        false, true, FLT_EPSILON);

				MEM_freeN(looptris);
			}

			{
				/* group vars */
				int *groups_array;
				int (*group_index)[2];
				int group_tot;
				int i;
				bool is_inside;
				BMFace **ftable;

				BM_mesh_elem_table_ensure(bm, BM_FACE);
				ftable = bm->ftable;

				groups_array = MEM_mallocN(sizeof(*groups_array) * bm->totface, __func__);
				group_tot = BM_mesh_calc_face_groups(
				        bm, groups_array, &group_index,
				        NULL, NULL,
				        0, BM_EDGE);

				/* first check if island is inside */

				/* TODO, find if islands are inside/outside,
				 * for now remove alternate islands, as simple testcase */
				is_inside = false;


				for (i = 0; i < group_tot; i++) {
					int fg     = group_index[i][0];
					int fg_end = group_index[i][1] + fg;

					if (is_inside) {
						for (; fg != fg_end; fg++) {
							BM_face_kill(bm, ftable[groups_array[fg]]);
						}
					}
					is_inside = !is_inside;
				}

				MEM_freeN(groups_array);
				MEM_freeN(group_index);
			}

			result = CDDM_from_bmesh(bm, true);

			BM_mesh_free(bm);

			result->dirty |= DM_DIRTY_NORMALS;

			TIMEIT_END(NewBooleanDerivedMesh);

			return result;
		}

		/* if new mesh returned, return it; otherwise there was
		 * an error, so delete the modifier object */
		if (result)
			return result;
		else
			modifier_setError(md, "Cannot execute boolean operation");
	}

	return derivedData;
}
#else // USE_BMESH
static DerivedMesh *applyModifier(ModifierData *md, Object *ob,
                                  DerivedMesh *derivedData,
                                  ModifierApplyFlag flag)
{
	BooleanModifierData *bmd = (BooleanModifierData *) md;
	DerivedMesh *dm;

	if (!bmd->object)
		return derivedData;

	dm = get_dm_for_modifier(bmd->object, flag);

	if (dm) {
		DerivedMesh *result;

		/* when one of objects is empty (has got no faces) we could speed up
		 * calculation a bit returning one of objects' derived meshes (or empty one)
		 * Returning mesh is depended on modifiers operation (sergey) */
		result = get_quick_derivedMesh(derivedData, dm, bmd->operation);

		if (result == NULL) {
			// TIMEIT_START(NewBooleanDerivedMesh)

			result = NewBooleanDerivedMesh(dm, bmd->object, derivedData, ob,
			                               1 + bmd->operation);

			// TIMEIT_END(NewBooleanDerivedMesh)
		}

		/* if new mesh returned, return it; otherwise there was
		 * an error, so delete the modifier object */
		if (result)
			return result;
		else
			modifier_setError(md, "Cannot execute boolean operation");
	}
	
	return derivedData;
}
#endif // USE_BMESH
#else // WITH_MOD_BOOLEAN
static DerivedMesh *applyModifier(ModifierData *UNUSED(md), Object *UNUSED(ob),
                                  DerivedMesh *derivedData,
                                  ModifierApplyFlag UNUSED(flag))
{
	return derivedData;
}
#endif // WITH_MOD_BOOLEAN

static CustomDataMask requiredDataMask(Object *UNUSED(ob), ModifierData *UNUSED(md))
{
	CustomDataMask dataMask = CD_MASK_MTFACE | CD_MASK_MEDGE;

	dataMask |= CD_MASK_MDEFORMVERT;
	
	return dataMask;
}


ModifierTypeInfo modifierType_Boolean = {
	/* name */              "Boolean",
	/* structName */        "BooleanModifierData",
	/* structSize */        sizeof(BooleanModifierData),
	/* type */              eModifierTypeType_Nonconstructive,
	/* flags */             eModifierTypeFlag_AcceptsMesh |
	                        eModifierTypeFlag_UsesPointCache,

	/* copyData */          copyData,
	/* deformVerts */       NULL,
	/* deformMatrices */    NULL,
	/* deformVertsEM */     NULL,
	/* deformMatricesEM */  NULL,
	/* applyModifier */     applyModifier,
	/* applyModifierEM */   NULL,
	/* initData */          NULL,
	/* requiredDataMask */  requiredDataMask,
	/* freeData */          NULL,
	/* isDisabled */        isDisabled,
	/* updateDepgraph */    updateDepgraph,
	/* dependsOnTime */     NULL,
	/* dependsOnNormals */  NULL,
	/* foreachObjectLink */ foreachObjectLink,
	/* foreachIDLink */     NULL,
	/* foreachTexLink */    NULL,
};
