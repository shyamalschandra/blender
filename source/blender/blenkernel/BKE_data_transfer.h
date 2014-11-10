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
 * The Original Code is Copyright (C) 2014 Blender Foundation.
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): Bastien Montagne
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/blenkernel/BKE_data_transfer.h
 *  \ingroup bke
 */

#ifndef __BKE_DATA_TRANSFER_H__
#define __BKE_DATA_TRANSFER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "BKE_customdata.h"

struct Object;
struct Scene;
struct SpaceTransform;

/* Warning, those def are stored in files (TransferData modifier), *DO NOT* modify those values. */
enum {
	DT_DATA_MDEFORMVERT                 = 1 << 0,
	DT_DATA_SHAPEKEY                    = 1 << 1,
	DT_DATA_SKIN                        = 1 << 2,
	DT_DATA_BWEIGHT_VERT                = 1 << 3,

	DT_DATA_SHARP_EDGE                  = 1 << 8,
	DT_DATA_SEAM                        = 1 << 9,
	DT_DATA_CREASE                      = 1 << 10,
	DT_DATA_BWEIGHT_EDGE                = 1 << 11,
	DT_DATA_FREESTYLE_EDGE              = 1 << 12,

	DT_DATA_UV                          = 1 << 16,
	DT_DATA_SHARP_FACE                  = 1 << 17,
	DT_DATA_FREESTYLE_FACE              = 1 << 18,

	DT_DATA_VCOL                        = 1 << 24,
};

CustomDataMask BKE_data_transfer_dttypes_to_cdmask(const int dtdata_types);
bool BKE_data_transfer_get_dttypes_capacity(
        const int dtdata_types, bool *r_advanced_mixing, bool *r_threshold);

int BKE_data_transfer_dttype_to_cdtype(const int dtdata_type);
int BKE_data_transfer_dttype_to_fromto_idx(const int dtdata_type);

#define DT_DATATYPE_IS_VERT(_dt) ELEM(_dt, DT_DATA_MDEFORMVERT, DT_DATA_SHAPEKEY, DT_DATA_SKIN,  \
                                           DT_DATA_BWEIGHT_VERT)
#define DT_DATATYPE_IS_EDGE(_dt) ELEM(_dt, DT_DATA_CREASE, DT_DATA_SHARP_EDGE, DT_DATA_SEAM,  \
                                           DT_DATA_BWEIGHT_EDGE, DT_DATA_FREESTYLE_EDGE)
#define DT_DATATYPE_IS_POLY(_dt) ELEM(_dt, DT_DATA_UV, DT_DATA_SHARP_FACE, DT_DATA_FREESTYLE_FACE)
#define DT_DATATYPE_IS_LOOP(_dt) ELEM(_dt, DT_DATA_UV, DT_DATA_VCOL)

#define DT_DATATYPE_IS_MULTILAYERS(_dt) ELEM(_dt, DT_DATA_MDEFORMVERT, DT_DATA_SHAPEKEY, DT_DATA_UV, DT_DATA_VCOL)


enum {
	DT_MULTILAYER_IDX_INVALID           = -1,
	DT_MULTILAYER_IDX_MDEFORMVERT       = 0,
	DT_MULTILAYER_IDX_SHAPEKEY          = 1,
	DT_MULTILAYER_IDX_UV                = 2,
	DT_MULTILAYER_IDX_VCOL              = 3,
	DT_MULTILAYER_IDX_MAX               = 4,
};

/* Below we keep positive values for real layers idx (generated dynamically). */
/* How to select data layers, for types supporting multi-layers.
 * Here too, some options are highly dependent on type of transferred data! */
enum {
	DT_FROMLAYERS_ACTIVE                 = -1,
	DT_FROMLAYERS_ALL                    = -2,
	/* Datatype-specific. */
	DT_FROMLAYERS_VGROUP                 = 1 << 8,
	DT_FROMLAYERS_VGROUP_BONE_SELECTED   = -(DT_FROMLAYERS_VGROUP | 1),
	DT_FROMLAYERS_VGROUP_BONE_DEFORM     = -(DT_FROMLAYERS_VGROUP | 2),
	/* Other types-related modes... */
};

/* How to map a source layer to a destination layer, for types supporting multi-layers.
 * Note: if no matching layer can be found, it will be created. */
enum {
	DT_TOLAYERS_ACTIVE                   = -1,  /* Only for DT_LAYERS_FROMSEL_ACTIVE. */
	DT_TOLAYERS_NAME                     = -2,
	DT_TOLAYERS_INDEX                    = -3,
#if 0  /* TODO */
	DT_TOLAYERS_CREATE                   = -4,  /* Never replace existing data in dst, always create new layers. */
#endif
};

bool BKE_data_transfer_mesh(
        struct Scene *scene, struct Object *ob_src, struct Object *ob_dst, const int data_types, const bool use_create,
        const int map_vert_mode, const int map_edge_mode, const int map_loop_mode, const int map_poly_mode,
        struct SpaceTransform *space_transform, const float max_distance, const float precision,
        const int fromlayers_select[DT_MULTILAYER_IDX_MAX], const int tolayers_select[DT_MULTILAYER_IDX_MAX],
        const int mix_mode, const float mix_factor, const char *vgroup_name, const bool invert_vgroup);
bool BKE_data_transfer_dm(
        struct Scene *scene, struct Object *ob_src, struct Object *ob_dst, struct DerivedMesh *dm_dst,
        const int data_types, const bool use_create,
        const int map_vert_mode, const int map_edge_mode, const int map_loop_mode, const int map_poly_mode,
        struct SpaceTransform *space_transform, const float max_distance, const float ray_radius,
        const int fromlayers_select[DT_MULTILAYER_IDX_MAX], const int tolayers_select[DT_MULTILAYER_IDX_MAX],
        const int mix_mode, const float mix_factor, const char *vgroup_name, const bool invert_vgroup);

#ifdef __cplusplus
}
#endif

#endif  /* __BKE_DATA_TRANSFER_H__ */

