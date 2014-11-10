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
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2014 by Blender Foundation.
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): Bastien Montagne.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/blenkernel/intern/data_transfer.c
 *  \ingroup bke
 */

#include "MEM_guardedalloc.h"

#include "DNA_customdata_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BLI_array.h"
#include "BLI_math.h"
#include "BLI_blenlib.h"
#include "BLI_utildefines.h"

#include "BKE_context.h"
#include "BKE_customdata.h"
#include "BKE_data_transfer.h"
#include "BKE_deform.h"
#include "BKE_DerivedMesh.h"
#include "BKE_mesh_mapping.h"
#include "BKE_object.h"
#include "BKE_object_deform.h"

#include "data_transfer_intern.h"


CustomDataMask BKE_data_transfer_dttypes_to_cdmask(const int dtdata_types)
{
	CustomDataMask cddata_mask = 0;
	int i;

	for (i = 0; i < 32; i++) {
		const int dtdata_type = 1 << i;
		int cddata_type;

		if (!(dtdata_types & dtdata_type)) {
			continue;
		}

		cddata_type = BKE_data_transfer_dttype_to_cdtype(dtdata_type);
		if (!(cddata_type & CD_FAKE)) {
			cddata_mask |= 1LL << cddata_type;
		}
		else if (cddata_type == CD_FAKE_MDEFORMVERT) {
			cddata_mask |= CD_MASK_MDEFORMVERT;  /* Exception for vgroups :/ */
		}
		else if (cddata_type == CD_FAKE_UV) {
			cddata_mask |= CD_MASK_MTEXPOLY | CD_MASK_MLOOPUV;
		}
	}

	return cddata_mask;
}

/* Check what can do each layer type (if it is actually handled by transferdata, if it supports advanced mixing... */
bool BKE_data_transfer_get_dttypes_capacity(
        const int dtdata_types, bool *r_advanced_mixing, bool *r_threshold)
{
	int i;
	bool ret = false;

	*r_advanced_mixing = false;
	*r_threshold = false;

	for (i = 0; (i < 32) && !(ret && *r_advanced_mixing && *r_threshold); i++) {
		const int dtdata_type = 1 << i;

		if (!(dtdata_types & dtdata_type)) {
			continue;
		}

		switch (dtdata_type) {
		/* Vertex data */
			case DT_DATA_MDEFORMVERT:
				*r_advanced_mixing = true;
				*r_threshold = true;
				ret = true;
				break;
			case DT_DATA_SKIN:
				*r_threshold = true;
				ret = true;
				break;
			case DT_DATA_BWEIGHT_VERT:
				ret = true;
				break;
		/* Edge data */
			case DT_DATA_SHARP_EDGE:
				*r_threshold = true;
				ret = true;
				break;
			case DT_DATA_SEAM:
				*r_threshold = true;
				ret = true;
				break;
			case DT_DATA_CREASE:
				ret = true;
				break;
			case DT_DATA_BWEIGHT_EDGE:
				ret = true;
				break;
			case DT_DATA_FREESTYLE_EDGE:
				ret = true;
				break;
		/* Loop/Poly data */
			case DT_DATA_UV:
				ret = true;
				break;
			case DT_DATA_VCOL:
				*r_advanced_mixing = true;
				*r_threshold = true;
				ret = true;
				break;
			case DT_DATA_SHARP_FACE:
				ret = true;
				break;
			case DT_DATA_FREESTYLE_FACE:
				ret = true;
				break;
		}
	}

	return ret;
}

int BKE_data_transfer_dttype_to_cdtype(const int dtdata_type)
{
	switch (dtdata_type) {
		case DT_DATA_MDEFORMVERT:
			return CD_FAKE_MDEFORMVERT;
		case DT_DATA_SHAPEKEY:
			return CD_FAKE_SHAPEKEY;
		case DT_DATA_SKIN:
			return CD_MVERT_SKIN;
		case DT_DATA_BWEIGHT_VERT:
			return CD_FAKE_BWEIGHT;

		case DT_DATA_SHARP_EDGE:
			return CD_FAKE_SHARP;
		case DT_DATA_SEAM:
			return CD_FAKE_SEAM;
		case DT_DATA_CREASE:
			return CD_FAKE_CREASE;
		case DT_DATA_BWEIGHT_EDGE:
			return CD_FAKE_BWEIGHT;
		case DT_DATA_FREESTYLE_EDGE:
			return CD_FREESTYLE_EDGE;

		case DT_DATA_UV:
			return CD_FAKE_UV;
		case DT_DATA_SHARP_FACE:
			return CD_FAKE_SHARP;
		case DT_DATA_FREESTYLE_FACE:
			return CD_FREESTYLE_FACE;

		case DT_DATA_VCOL:
			return CD_MLOOPCOL;

		default:
			BLI_assert(0);
	}
	return 0;  /* Should never be reached! */
}

int BKE_data_transfer_dttype_to_fromto_idx(const int dtdata_type)
{
	switch (dtdata_type) {
		case DT_DATA_MDEFORMVERT:
			return DT_MULTILAYER_IDX_MDEFORMVERT;
		case DT_DATA_SHAPEKEY:
			return DT_MULTILAYER_IDX_SHAPEKEY;
		case DT_DATA_UV:
			return DT_MULTILAYER_IDX_UV;
		case DT_DATA_VCOL:
			return DT_MULTILAYER_IDX_VCOL;
		default:
			return DT_MULTILAYER_IDX_INVALID;
	}
}

/* ********** */

static loop_island_compute data_transfer_get_loop_islands_generator(const int cddata_type)
{
	switch (cddata_type) {
		case CD_FAKE_UV:
			return BKE_loop_poly_island_compute_uv;
			break;
		default:
			break;
	}
	return NULL;
}

float data_transfer_interp_float_do(
        const int mix_mode, const float val_dst, const float val_src, const float mix_factor)
{
	float val_ret;

	if (((mix_mode == CDT_MIX_REPLACE_ABOVE_THRESHOLD && (val_dst < mix_factor)) ||
	     (mix_mode == CDT_MIX_REPLACE_BELOW_THRESHOLD && (val_dst > mix_factor))))
	{
		return val_dst;  /* Do not affect destination. */
	}

	switch (mix_mode) {
		case CDT_MIX_REPLACE_ABOVE_THRESHOLD:
		case CDT_MIX_REPLACE_BELOW_THRESHOLD:
			return val_src;
		case CDT_MIX_MIX:
			val_ret = (val_dst + val_src) * 0.5f;
			break;
		case CDT_MIX_ADD:
			val_ret = val_dst + val_src;
			break;
		case CDT_MIX_SUB:
			val_ret = val_dst - val_src;
			break;
		case CDT_MIX_MUL:
			val_ret = val_dst * val_src;
			break;
		case CDT_MIX_TRANSFER:
		default:
			val_ret = val_src;
			break;
	}
	return interpf(val_ret, val_dst, mix_factor);
}

static void data_transfer_interp_char(const DataTransferLayerMapping *laymap, void *dest,
                                      void **sources, const float *weights, const int count, const float mix_factor)
{
	char **data_src = (char **)sources;
	char *data_dst = (char *)dest;

	const int mix_mode = laymap->mix_mode;
	float val_src = 0.0f;
	const float val_dst = (float)(*data_dst) / 255.0f;

	int i;

	for (i = count; i--;) {
		val_src += ((float)(*data_src[i]) / 255.0f) * weights[i];
	}

	val_src = data_transfer_interp_float_do(mix_mode, val_dst, val_src, mix_factor);

	CLAMP(val_src, 0.0f, 1.0f);

	*data_dst = (char)(val_src * 255.0f);
}

/* Helpers to match sources and destinations data layers (also handles 'conversions' in CD_FAKE cases). */

void data_transfer_layersmapping_add_item(
        ListBase *r_map, const int cddata_type, const int mix_mode, const float mix_factor, const float *mix_weights,
        void *data_src, void *data_dst, const int data_n_src, const int data_n_dst,
        const size_t elem_size, const size_t data_size, const size_t data_offset, const uint64_t data_flag,
        cd_datatransfer_interp interp)
{
	DataTransferLayerMapping *item = MEM_mallocN(sizeof(*item), __func__);

	BLI_assert(data_dst != NULL);

	item->data_type = cddata_type;
	item->mix_mode = mix_mode;
	item->mix_factor = mix_factor;
	item->mix_weights = mix_weights;

	item->data_src = data_src;
	item->data_dst = data_dst;
	item->data_n_src = data_n_src;
	item->data_n_dst = data_n_dst;
	item->elem_size = elem_size;

	item->data_size = data_size;
	item->data_offset = data_offset;
	item->data_flag = data_flag;

	item->interp = interp;

	BLI_addtail(r_map, item);
}

static void data_transfer_layersmapping_add_item_cd(
        ListBase *r_map, const int cddata_type, const int mix_mode, const float mix_factor, const float *mix_weights,
        void *data_src, void *data_dst)
{
	data_transfer_layersmapping_add_item(r_map, cddata_type, mix_mode, mix_factor, mix_weights, data_src, data_dst,
	                                     0, 0, 0, 0, 0, 0, NULL);
}

static bool data_transfer_layersmapping_cdlayers_multisrc_to_dst(
        ListBase *r_map, const int cddata_type, const int mix_mode, const float mix_factor, const float *mix_weights,
        const int num_create, CustomData *cd_src, CustomData *cd_dst, const bool dup_dst,
        const int tolayers, bool *use_layers_src, const int num_layers_src)
{
	void *data_src, *data_dst = NULL;
	int idx_src = num_layers_src;
	int idx_dst;

	switch (tolayers) {
		case DT_TOLAYERS_INDEX:
			{
				idx_dst = CustomData_number_of_layers(data_dst, cddata_type);

				/* Find last source actually used! */
				while (idx_src-- && !use_layers_src[idx_src]);
				idx_src++;

				if (idx_dst < idx_src) {
					if (!num_create) {
						return false;
					}
					/* Create as much data layers as necessary! */
					for (; idx_dst < idx_src; idx_dst++) {
						CustomData_add_layer(cd_dst, cddata_type, CD_CALLOC, NULL, num_create);
					}
				}
				while (idx_src--) {
					if (!use_layers_src[idx_src]) {
						continue;
					}
					data_src = CustomData_get_layer_n(cd_src, cddata_type, idx_src);
					/* If dest is a derivedmesh, we do not want to overwrite cdlayers of org mesh! */
					data_dst = dup_dst ? CustomData_duplicate_referenced_layer_n(cd_dst, cddata_type, idx_src, num_create) :
					                     CustomData_get_layer_n(cd_dst, cddata_type, idx_src);
					data_transfer_layersmapping_add_item_cd(r_map, cddata_type, mix_mode, mix_factor, mix_weights, data_src, data_dst);
				}
			}
			break;
		case DT_TOLAYERS_NAME:
			while (idx_src--) {
				const char *name;

				if (!use_layers_src[idx_src]) {
					continue;
				}

				name = CustomData_get_layer_name(cd_src, cddata_type, idx_src);
				data_src = CustomData_get_layer_n(cd_src, cddata_type, idx_src);

				if ((idx_dst = CustomData_get_named_layer(cd_dst, cddata_type, name)) == -1) {
					if (!num_create) {
						BLI_freelistN(r_map);
						return false;
					}
					CustomData_add_layer_named(cd_dst, cddata_type, CD_CALLOC, NULL, num_create, name);
					idx_dst = CustomData_get_named_layer(cd_dst, cddata_type, name);
				}
				/* If dest is a derivedmesh, we do not want to overwrite cdlayers of org mesh! */
				data_dst = dup_dst ? CustomData_duplicate_referenced_layer_n(cd_dst, cddata_type, idx_dst, num_create) :
				                     CustomData_get_layer_n(cd_dst, cddata_type, idx_dst);
				data_transfer_layersmapping_add_item_cd(r_map, cddata_type, mix_mode, mix_factor, mix_weights, data_src, data_dst);
			}
			break;
		default:
			return false;
	}

	return true;
}

static bool data_transfer_layersmapping_cdlayers(
        ListBase *r_map, const int cddata_type, const int mix_mode, const float mix_factor, const float *mix_weights,
        const int num_create, CustomData *cd_src, CustomData *cd_dst, const bool dup_dst,
        const int fromlayers, const int tolayers)
{
	int idx_src, idx_dst;
	void *data_src, *data_dst = NULL;

	if (CustomData_layertype_is_singleton(cddata_type)) {
		if (!(data_src = CustomData_get_layer(cd_src, cddata_type))) {
			return false;
		}
		/* If dest is a derivedmesh, we do not want to overwrite cdlayers of org mesh! */
		data_dst = dup_dst ? CustomData_duplicate_referenced_layer(cd_dst, cddata_type, num_create) :
		                     CustomData_get_layer(cd_dst, cddata_type);
		if (!data_dst) {
			if (!num_create) {
				return false;
			}
			data_dst = CustomData_add_layer(cd_dst, cddata_type, CD_CALLOC, NULL, num_create);
		}

		data_transfer_layersmapping_add_item_cd(r_map, cddata_type, mix_mode, mix_factor, mix_weights, data_src, data_dst);
	}
	else if (fromlayers == DT_FROMLAYERS_ACTIVE || fromlayers >= 0) {
		if (fromlayers >= 0) {  /* Real-layer index */
			idx_src = fromlayers;
		}
		else {
			if ((idx_src = CustomData_get_active_layer(cd_src, cddata_type)) == -1) {
				return false;
			}
		}
		data_src = CustomData_get_layer_n(cd_src, cddata_type, idx_src);
		if (!data_src) {
			return false;
		}

		if (tolayers >= 0) {  /* Real-layer index */
			idx_dst = tolayers;
			data_dst = CustomData_get_layer_n(cd_dst, cddata_type, idx_dst);
		}
		else if (tolayers == DT_TOLAYERS_ACTIVE) {
			if ((idx_dst = CustomData_get_active_layer(cd_dst, cddata_type)) == -1) {
				if (!num_create) {
					return false;
				}
				data_dst = CustomData_add_layer(cd_dst, cddata_type, CD_CALLOC, NULL, num_create);
			}
			else {
				/* If dest is a derivedmesh, we do not want to overwrite cdlayers of org mesh! */
				if (dup_dst) {
					data_dst = CustomData_duplicate_referenced_layer_n(cd_dst, cddata_type, idx_dst, num_create);
				}
				else {
					data_dst = CustomData_get_layer_n(cd_dst, cddata_type, idx_dst);
				}
			}
		}
		else if (tolayers == DT_TOLAYERS_INDEX) {
			int num = CustomData_number_of_layers(cd_dst, cddata_type);
			idx_dst = idx_src;
			if (num <= idx_dst) {
				if (!num_create) {
					return false;
				}
				/* Create as much data layers as necessary! */
				for (; num <= idx_dst; num++) {
					CustomData_add_layer(cd_dst, cddata_type, CD_CALLOC, NULL, num_create);
				}
			}
			/* If dest is a derivedmesh, we do not want to overwrite cdlayers of org mesh! */
			if (dup_dst) {
				data_dst = CustomData_duplicate_referenced_layer_n(cd_dst, cddata_type, idx_dst, num_create);
			}
			else {
				data_dst = CustomData_get_layer_n(cd_dst, cddata_type, idx_dst);
			}
		}
		else if (tolayers == DT_TOLAYERS_NAME) {
			const char *name = CustomData_get_layer_name(cd_src, cddata_type, idx_src);
			if ((idx_dst = CustomData_get_named_layer(cd_dst, cddata_type, name)) == -1) {
				if (!num_create) {
					return false;
				}
				CustomData_add_layer_named(cd_dst, cddata_type, CD_CALLOC, NULL, num_create, name);
				idx_dst = CustomData_get_named_layer(cd_dst, cddata_type, name);
			}
			/* If dest is a derivedmesh, we do not want to overwrite cdlayers of org mesh! */
			if (dup_dst) {
				data_dst = CustomData_duplicate_referenced_layer_n(cd_dst, cddata_type, idx_dst, num_create);
			}
			else {
				data_dst = CustomData_get_layer_n(cd_dst, cddata_type, idx_dst);
			}
		}
		else {
			return false;
		}

		if (!data_dst) {
			return false;
		}

		data_transfer_layersmapping_add_item_cd(r_map, cddata_type, mix_mode, mix_factor, mix_weights,
		                                        data_src, data_dst);
	}
	else if (fromlayers == DT_FROMLAYERS_ALL) {
		int num_src = CustomData_number_of_layers(cd_src, cddata_type);
		bool *use_layers_src = MEM_mallocN(sizeof(*use_layers_src) * (size_t)num_src, __func__);
		bool ret;

		memset(use_layers_src, true, sizeof(*use_layers_src) * num_src);

		ret = data_transfer_layersmapping_cdlayers_multisrc_to_dst(r_map, cddata_type, mix_mode, mix_factor, mix_weights,
		                                                           num_create, cd_src, cd_dst, dup_dst,
		                                                           tolayers, use_layers_src, num_src);

		MEM_freeN(use_layers_src);
		return ret;
	}
	else {
		return false;
	}

	return true;
}

static bool data_transfer_layersmapping_generate(
        ListBase *r_map, Object *ob_src, Object *ob_dst, DerivedMesh *dm_src, DerivedMesh *dm_dst, Mesh *me_dst,
        const int elem_type, int cddata_type, int mix_mode, float mix_factor, const float *mix_weights,
        const int num_create, const int fromlayers, const int tolayers)
{
	CustomData *cd_src, *cd_dst;

	if (elem_type == ME_VERT) {
		if (!(cddata_type & CD_FAKE)) {
			cd_src = dm_src->getVertDataLayout(dm_src);
			cd_dst = dm_dst ? dm_dst->getVertDataLayout(dm_dst) : &me_dst->vdata;

			if (!CustomData_has_layer(cd_src, cddata_type)) {
				return false;
			}

			if (!data_transfer_layersmapping_cdlayers(r_map, cddata_type, mix_mode, mix_factor, mix_weights,
			                                          num_create, cd_src, cd_dst, dm_dst != NULL,
			                                          fromlayers, tolayers))
			{
				/* We handle specific source selection cases here. */
				return false;
			}
			return true;
		}
		else if (cddata_type == CD_FAKE_BWEIGHT) {
			const size_t elem_size = sizeof(*((MVert *)NULL));
			const size_t data_size = sizeof(((MVert *)NULL)->bweight);
			const size_t data_offset = offsetof(MVert, bweight);
			const uint64_t data_flag = 0;

			if (!(dm_src->cd_flag & ME_CDFLAG_VERT_BWEIGHT)) {
				return false;
			}
			if (dm_dst) {
				dm_dst->cd_flag |= ME_CDFLAG_VERT_BWEIGHT;
			}
			else {
				me_dst->cd_flag |= ME_CDFLAG_VERT_BWEIGHT;
			}
			data_transfer_layersmapping_add_item(r_map, cddata_type, mix_mode, mix_factor, mix_weights,
			                                     dm_src->getVertArray(dm_src),
			                                     dm_dst ? dm_dst->getVertArray(dm_dst) : me_dst->mvert,
			                                     dm_src->getNumVerts(dm_src),
			                                     dm_dst ? dm_dst->getNumVerts(dm_dst) : me_dst->totvert,
			                                     elem_size, data_size, data_offset, data_flag,
			                                     data_transfer_interp_char);
			return true;
		}
		else if (cddata_type == CD_FAKE_MDEFORMVERT) {
			cd_src = dm_src->getVertDataLayout(dm_src);
			cd_dst = dm_dst ? dm_dst->getVertDataLayout(dm_dst) : &me_dst->vdata;

			return data_transfer_layersmapping_vgroups(r_map, mix_mode, mix_factor, mix_weights, num_create,
			                                           ob_src, ob_dst, cd_src, cd_dst, dm_dst != NULL,
			                                           fromlayers, tolayers);
		}
		else if (cddata_type == CD_FAKE_SHAPEKEY) {
			/* TODO: leaving shapekeys asside for now, quite specific case, since we can't access them from MVert :/ */
			return false;
		}
	}
	else if (elem_type == ME_EDGE) {
		if (!(cddata_type & CD_FAKE)) {  /* Unused for edges, currently... */
			cd_src = dm_src->getEdgeDataLayout(dm_src);
			cd_dst = dm_dst ? dm_dst->getEdgeDataLayout(dm_dst) : &me_dst->edata;

			if (!CustomData_has_layer(cd_src, cddata_type)) {
				return false;
			}

			if (!data_transfer_layersmapping_cdlayers(r_map, cddata_type, mix_mode, mix_factor, mix_weights,
			                                          num_create, cd_src, cd_dst, dm_dst != NULL,
			                                          fromlayers, tolayers))
			{
				/* We handle specific source selection cases here. */
				return false;
			}
			return true;
		}
		else if (cddata_type == CD_FAKE_CREASE) {
			const size_t elem_size = sizeof(*((MEdge *)NULL));
			const size_t data_size = sizeof(((MEdge *)NULL)->crease);
			const size_t data_offset = offsetof(MEdge, crease);
			const uint64_t data_flag = 0;

			if (!(dm_src->cd_flag & ME_CDFLAG_EDGE_CREASE)) {
				return false;
			}
			if (dm_dst) {
				dm_dst->cd_flag |= ME_CDFLAG_EDGE_CREASE;
			}
			else {
				me_dst->cd_flag |= ME_CDFLAG_EDGE_CREASE;
			}
			data_transfer_layersmapping_add_item(r_map, cddata_type, mix_mode, mix_factor, mix_weights,
			                                     dm_src->getEdgeArray(dm_src),
			                                     dm_dst ? dm_dst->getEdgeArray(dm_dst) : me_dst->medge,
			                                     dm_src->getNumEdges(dm_src),
			                                     dm_dst ? dm_dst->getNumEdges(dm_dst) : me_dst->totedge,
			                                     elem_size, data_size, data_offset, data_flag,
			                                     data_transfer_interp_char);
			return true;
		}
		else if (cddata_type == CD_FAKE_BWEIGHT) {
			const size_t elem_size = sizeof(*((MEdge *)NULL));
			const size_t data_size = sizeof(((MEdge *)NULL)->bweight);
			const size_t data_offset = offsetof(MEdge, bweight);
			const uint64_t data_flag = 0;

			if (!(dm_src->cd_flag & ME_CDFLAG_EDGE_BWEIGHT)) {
				return false;
			}
			if (dm_dst) {
				dm_dst->cd_flag |= ME_CDFLAG_EDGE_BWEIGHT;
			}
			else {
				me_dst->cd_flag |= ME_CDFLAG_EDGE_BWEIGHT;
			}
			data_transfer_layersmapping_add_item(r_map, cddata_type, mix_mode, mix_factor, mix_weights,
			                                     dm_src->getEdgeArray(dm_src),
			                                     dm_dst ? dm_dst->getEdgeArray(dm_dst) : me_dst->medge,
			                                     dm_src->getNumEdges(dm_src),
			                                     dm_dst ? dm_dst->getNumEdges(dm_dst) : me_dst->totedge,
			                                     elem_size, data_size, data_offset, data_flag,
			                                     data_transfer_interp_char);
			return true;
		}
		else if (ELEM(cddata_type, CD_FAKE_SHARP, CD_FAKE_SEAM)) {
			const size_t elem_size = sizeof(*((MEdge *)NULL));
			const size_t data_size = sizeof(((MEdge *)NULL)->flag);
			const size_t data_offset = offsetof(MEdge, flag);
			const uint64_t data_flag = (cddata_type == CD_FAKE_SHARP) ? ME_SHARP : ME_SEAM;
			data_transfer_layersmapping_add_item(r_map, cddata_type, mix_mode, mix_factor, mix_weights,
			                                     dm_src->getEdgeArray(dm_src),
			                                     dm_dst ? dm_dst->getEdgeArray(dm_dst) : me_dst->medge,
			                                     dm_src->getNumEdges(dm_src),
			                                     dm_dst ? dm_dst->getNumEdges(dm_dst) : me_dst->totedge,
			                                     elem_size, data_size, data_offset, data_flag, NULL);
			return true;
		}
		else {
			return false;
		}
	}
	else if (elem_type == ME_LOOP) {
		if (cddata_type == CD_FAKE_UV) {
			cddata_type = CD_MLOOPUV;
		}

		if (!(cddata_type & CD_FAKE)) {
			cd_src = dm_src->getLoopDataLayout(dm_src);
			cd_dst = dm_dst ? dm_dst->getLoopDataLayout(dm_dst) : &me_dst->ldata;

			if (!CustomData_has_layer(cd_src, cddata_type)) {
				return false;
			}

			if (!data_transfer_layersmapping_cdlayers(r_map, cddata_type, mix_mode, mix_factor, mix_weights,
			                                          num_create, cd_src, cd_dst, dm_dst != NULL,
			                                          fromlayers, tolayers))
			{
				/* We handle specific source selection cases here. */
				return false;
			}
			return true;
		}
		else {
			return false;
		}
	}
	else if (elem_type == ME_POLY) {
		if (cddata_type == CD_FAKE_UV) {
			cddata_type = CD_MTEXPOLY;
		}

		if (!(cddata_type & CD_FAKE)) {
			cd_src = dm_src->getPolyDataLayout(dm_src);
			cd_dst = dm_dst ? dm_dst->getPolyDataLayout(dm_dst) : &me_dst->pdata;

			if (!CustomData_has_layer(cd_src, cddata_type)) {
				return false;
			}

			if (!data_transfer_layersmapping_cdlayers(r_map, cddata_type, mix_mode, mix_factor, mix_weights,
			                                          num_create, cd_src, cd_dst, dm_dst != NULL,
			                                          fromlayers, tolayers))
			{
				/* We handle specific source selection cases here. */
				return false;
			}
			return true;
		}
		else if (cddata_type == CD_FAKE_SHARP) {
			const size_t elem_size = sizeof(*((MPoly *)NULL));
			const size_t data_size = sizeof(((MPoly *)NULL)->flag);
			const size_t data_offset = offsetof(MPoly, flag);
			const uint64_t data_flag = ME_SMOOTH;
			data_transfer_layersmapping_add_item(r_map, cddata_type, mix_mode, mix_factor, mix_weights,
			                                     dm_src->getPolyArray(dm_src),
			                                     dm_dst ? dm_dst->getPolyArray(dm_dst) : me_dst->mpoly,
			                                     dm_src->getNumPolys(dm_src),
			                                     dm_dst ? dm_dst->getNumPolys(dm_dst) : me_dst->totpoly,
			                                     elem_size, data_size, data_offset, data_flag, NULL);
			return true;
		}
		else {
			return false;
		}
	}

	return false;
}

bool BKE_data_transfer_dm(
        Scene *scene, Object *ob_src, Object *ob_dst, DerivedMesh *dm_dst, const int data_types, const bool use_create,
        const int map_vert_mode, const int map_edge_mode, const int map_loop_mode, const int map_poly_mode,
        SpaceTransform *space_transform, const float max_distance, const float ray_radius,
        const int fromlayers_select[DT_MULTILAYER_IDX_MAX], const int tolayers_select[DT_MULTILAYER_IDX_MAX],
        const int mix_mode, const float mix_factor, const char *vgroup_name, const bool invert_vgroup)
{
#define VDATA 0
#define EDATA 1
#define LDATA 2
#define PDATA 3
#define DATAMAX 4

	DerivedMesh *dm_src;
	Mesh *me_dst;
	int i;

	MDeformVert *mdef = NULL;
	int vg_idx = -1;
	float *weights[DATAMAX] = {NULL};

	Mesh2MeshMapping geom_map[DATAMAX] = {{0}};
	bool geom_map_init[DATAMAX] = {0};
	ListBase lay_map = {0};
	bool changed = false;

	CustomDataMask dm_src_mask = CD_MASK_BAREMESH;

	BLI_assert((ob_src != ob_dst) && (ob_src->type == OB_MESH) && (ob_dst->type == OB_MESH));

	me_dst = ob_dst->data;

	if (vgroup_name) {
		if (dm_dst) {
			mdef = dm_dst->getVertDataArray(dm_dst, CD_MDEFORMVERT);
		}
		else {
			mdef = CustomData_get_layer(&me_dst->vdata, CD_MDEFORMVERT);
		}
		if (mdef) {
			vg_idx = defgroup_name_index(ob_dst, vgroup_name);
		}
	}

	/* Get source DM.*/
	dm_src_mask |= BKE_data_transfer_dttypes_to_cdmask(data_types);
	dm_src = mesh_get_derived_final(scene, ob_src, dm_src_mask);

	/* Check all possible data types.
	 * Note item mappings and dest mix weights are cached. */
	for (i = 0; i < 32; i++) {
		const int dtdata_type = 1 << i;
		int cddata_type;
		int fromlayers, tolayers, fromto_idx;

		if (!(data_types & dtdata_type)) {
			continue;
		}

		cddata_type = BKE_data_transfer_dttype_to_cdtype(dtdata_type);

		fromto_idx = BKE_data_transfer_dttype_to_fromto_idx(dtdata_type);
		if (fromto_idx != DT_MULTILAYER_IDX_INVALID) {
			fromlayers = fromlayers_select[fromto_idx];
			tolayers = tolayers_select[fromto_idx];
		}
		else {
			fromlayers = tolayers = 0;
		}

		if (DT_DATATYPE_IS_VERT(dtdata_type)) {
			MVert *verts_dst = dm_dst ? dm_dst->getVertArray(dm_dst) : me_dst->mvert;
			const int num_verts_dst = dm_dst ? dm_dst->getNumVerts(dm_dst) : me_dst->totvert;
			const int num_create = use_create ? num_verts_dst : 0;

			if (!geom_map_init[VDATA]) {
				BKE_dm2mesh_mapping_verts_compute(map_vert_mode, space_transform, max_distance, ray_radius,
				                                  verts_dst, num_verts_dst, dm_src, &geom_map[VDATA]);
				geom_map_init[VDATA] = true;
			}

			if (mdef && vg_idx != -1 && !weights[VDATA]) {
				weights[VDATA] = MEM_mallocN(sizeof(*(weights[VDATA])) * (size_t)num_verts_dst, __func__);
				BKE_defvert_extract_vgroup_to_vertweights(mdef, vg_idx, num_verts_dst, weights[VDATA], invert_vgroup);
			}

			if (data_transfer_layersmapping_generate(&lay_map, ob_src, ob_dst, dm_src, dm_dst, me_dst, ME_VERT,
			                                         cddata_type, mix_mode, mix_factor, weights[VDATA],
			                                         num_create, fromlayers, tolayers))
			{
				DataTransferLayerMapping *lay_mapit;

				changed = (lay_map.first != NULL);

				for (lay_mapit = lay_map.first; lay_mapit; lay_mapit = lay_mapit->next) {
					CustomData_data_transfer(&geom_map[VDATA], lay_mapit);
				}

				BLI_freelistN(&lay_map);
			}
		}
		if (DT_DATATYPE_IS_EDGE(dtdata_type)) {
			MVert *verts_dst = dm_dst ? dm_dst->getVertArray(dm_dst) : me_dst->mvert;
			const int num_verts_dst = dm_dst ? dm_dst->getNumVerts(dm_dst) : me_dst->totvert;
			MEdge *edges_dst = dm_dst ? dm_dst->getEdgeArray(dm_dst) : me_dst->medge;
			const int num_edges_dst = dm_dst ? dm_dst->getNumEdges(dm_dst) : me_dst->totedge;
			const int num_create = use_create ? num_edges_dst : 0;

			if (!geom_map_init[EDATA]) {
				BKE_dm2mesh_mapping_edges_compute(map_edge_mode, space_transform, max_distance, ray_radius,
				                                  verts_dst, num_verts_dst, edges_dst, num_edges_dst,
				                                  dm_src, &geom_map[EDATA]);
				geom_map_init[EDATA] = true;
			}

			if (mdef && vg_idx != -1 && !weights[EDATA]) {
				weights[EDATA] = MEM_mallocN(sizeof(*weights[EDATA]) * (size_t)num_edges_dst, __func__);
				BKE_defvert_extract_vgroup_to_edgeweights(mdef, vg_idx, num_verts_dst, edges_dst, num_edges_dst,
				                                          weights[EDATA], invert_vgroup);
			}

			if (data_transfer_layersmapping_generate(&lay_map, ob_src, ob_dst, dm_src, dm_dst, me_dst, ME_EDGE,
			                                         cddata_type, mix_mode, mix_factor, weights[EDATA],
			                                         num_create, fromlayers, tolayers))
			{
				DataTransferLayerMapping *lay_mapit;

				changed = (lay_map.first != NULL);

				for (lay_mapit = lay_map.first; lay_mapit; lay_mapit = lay_mapit->next) {
					CustomData_data_transfer(&geom_map[EDATA], lay_mapit);
				}

				BLI_freelistN(&lay_map);
			}
		}
		if (DT_DATATYPE_IS_LOOP(dtdata_type)) {
			MVert *verts_dst = dm_dst ? dm_dst->getVertArray(dm_dst) : me_dst->mvert;
			const int num_verts_dst = dm_dst ? dm_dst->getNumVerts(dm_dst) : me_dst->totvert;
			MEdge *edges_dst = dm_dst ? dm_dst->getEdgeArray(dm_dst) : me_dst->medge;
			const int num_edges_dst = dm_dst ? dm_dst->getNumEdges(dm_dst) : me_dst->totedge;
			MPoly *polys_dst = dm_dst ? dm_dst->getPolyArray(dm_dst) : me_dst->mpoly;
			const int num_polys_dst = dm_dst ? dm_dst->getNumPolys(dm_dst) : me_dst->totpoly;
			MLoop *loops_dst = dm_dst ? dm_dst->getLoopArray(dm_dst) : me_dst->mloop;
			const int num_loops_dst = dm_dst ? dm_dst->getNumLoops(dm_dst) : me_dst->totloop;
			CustomData *pdata_dst = dm_dst ? dm_dst->getPolyDataLayout(dm_dst) : &me_dst->pdata;
			CustomData *ldata_dst = dm_dst ? dm_dst->getLoopDataLayout(dm_dst) : &me_dst->ldata;
			const int num_create = use_create ? me_dst->totloop : 0;

			loop_island_compute island_callback = data_transfer_get_loop_islands_generator(cddata_type);

			if (!geom_map_init[LDATA]) {
				BKE_dm2mesh_mapping_loops_compute(map_loop_mode, space_transform, max_distance, ray_radius,
				                                  verts_dst, num_verts_dst, edges_dst, num_edges_dst,
				                                  loops_dst, num_loops_dst, polys_dst, num_polys_dst,
				                                  ldata_dst, pdata_dst,
				                                  me_dst->smoothresh, dm_src, island_callback, &geom_map[LDATA]);
				geom_map_init[LDATA] = true;
			}

			if (mdef && vg_idx != -1 && !weights[LDATA]) {
				weights[LDATA] = MEM_mallocN(sizeof(*weights[LDATA]) * (size_t)num_loops_dst, __func__);
				BKE_defvert_extract_vgroup_to_loopweights(mdef, vg_idx, num_verts_dst, loops_dst, num_loops_dst,
				                                          weights[LDATA], invert_vgroup);
			}

			if (data_transfer_layersmapping_generate(&lay_map, ob_src, ob_dst, dm_src, dm_dst, me_dst, ME_LOOP,
			                                         cddata_type, mix_mode, mix_factor, weights[LDATA],
			                                         num_create, fromlayers, tolayers))
			{
				DataTransferLayerMapping *lay_mapit;

				changed = (lay_map.first != NULL);

				for (lay_mapit = lay_map.first; lay_mapit; lay_mapit = lay_mapit->next) {
					CustomData_data_transfer(&geom_map[LDATA], lay_mapit);
				}

				BLI_freelistN(&lay_map);
			}
		}
		if (DT_DATATYPE_IS_POLY(dtdata_type)) {
			MVert *verts_dst = dm_dst ? dm_dst->getVertArray(dm_dst) : me_dst->mvert;
			const int num_verts_dst = dm_dst ? dm_dst->getNumVerts(dm_dst) : me_dst->totvert;
			MPoly *polys_dst = dm_dst ? dm_dst->getPolyArray(dm_dst) : me_dst->mpoly;
			const int num_polys_dst = dm_dst ? dm_dst->getNumPolys(dm_dst) : me_dst->totpoly;
			MLoop *loops_dst = dm_dst ? dm_dst->getLoopArray(dm_dst) : me_dst->mloop;
			const int num_loops_dst = dm_dst ? dm_dst->getNumLoops(dm_dst) : me_dst->totloop;
			CustomData *pdata_dst = dm_dst ? dm_dst->getPolyDataLayout(dm_dst) : &me_dst->pdata;
			const int num_create = use_create ? num_polys_dst : 0;

			if (!geom_map_init[PDATA]) {
				BKE_dm2mesh_mapping_polys_compute(map_poly_mode, space_transform, max_distance, ray_radius,
				                                  verts_dst, num_verts_dst, loops_dst, num_loops_dst,
				                                  polys_dst, num_polys_dst, pdata_dst, dm_src, &geom_map[PDATA]);
				geom_map_init[PDATA] = true;
			}

			if (mdef && vg_idx != -1 && !weights[PDATA]) {
				weights[PDATA] = MEM_mallocN(sizeof(*weights[PDATA]) * (size_t)num_polys_dst, __func__);
				BKE_defvert_extract_vgroup_to_polyweights(mdef, vg_idx, num_verts_dst, loops_dst, num_loops_dst,
				                                          polys_dst, num_polys_dst, weights[PDATA], invert_vgroup);
			}

			if (data_transfer_layersmapping_generate(&lay_map, ob_src, ob_dst, dm_src, dm_dst, me_dst, ME_POLY,
			                                         cddata_type, mix_mode, mix_factor, weights[PDATA],
			                                         num_create, fromlayers, tolayers))
			{
				DataTransferLayerMapping *lay_mapit;

				changed = (lay_map.first != NULL);

				for (lay_mapit = lay_map.first; lay_mapit; lay_mapit = lay_mapit->next) {
					CustomData_data_transfer(&geom_map[PDATA], lay_mapit);
				}

				BLI_freelistN(&lay_map);
			}
		}
	}

	for (i = 0; i < DATAMAX; i++) {
		BKE_mesh2mesh_mapping_free(&geom_map[i]);
		MEM_SAFE_FREE(weights[i]);
	}

	return changed;

#undef VDATA
#undef EDATA
#undef LDATA
#undef PDATA
#undef DATAMAX
}

bool BKE_data_transfer_mesh(
        Scene *scene, Object *ob_src, Object *ob_dst, const int data_types, const bool use_create,
        const int map_vert_mode, const int map_edge_mode, const int map_loop_mode, const int map_poly_mode,
        SpaceTransform *space_transform, const float max_distance, const float ray_radius,
        const int fromlayers_select[DT_MULTILAYER_IDX_MAX], const int tolayers_select[DT_MULTILAYER_IDX_MAX],
        const int mix_mode, const float mix_factor, const char *vgroup_name, const bool invert_vgroup)
{
	return BKE_data_transfer_dm(scene, ob_src, ob_dst, NULL, data_types, use_create,
	                            map_vert_mode, map_edge_mode, map_loop_mode, map_poly_mode, space_transform,
	                            max_distance, ray_radius, fromlayers_select, tolayers_select,
	                            mix_mode, mix_factor, vgroup_name, invert_vgroup);
}
