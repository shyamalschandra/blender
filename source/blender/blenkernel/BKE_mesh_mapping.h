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
 * The Original Code is Copyright (C) 2001-2002 by NaN Holding BV.
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): (mar-2001 nzc)
 *
 * ***** END GPL LICENSE BLOCK *****
 */
#ifndef __BKE_MESH_MAPPING_H__
#define __BKE_MESH_MAPPING_H__

/** \file BKE_mesh_mapping.h
 *  \ingroup bke
 */

struct CustomData;
struct DerivedMesh;
struct MVert;
struct MPoly;
struct MEdge;
struct MLoop;
struct MLoopUV;

/* map from uv vertex to face (for select linked, stitch, uv suburf) */

/* UvVertMap */
#define STD_UV_CONNECT_LIMIT  0.0001f

typedef struct UvVertMap {
	struct UvMapVert **vert;
	struct UvMapVert *buf;
} UvVertMap;

typedef struct UvMapVert {
	struct UvMapVert *next;
	unsigned int f;
	unsigned char tfindex, separate, flag;
} UvMapVert;

/* UvElement stores per uv information so that we can quickly access information for a uv.
 * it is actually an improved UvMapVert, including an island and a direct pointer to the face
 * to avoid initializing face arrays */
typedef struct UvElement {
	/* Next UvElement corresponding to same vertex */
	struct UvElement *next;
	/* Face the element belongs to */
	struct BMLoop *l;
	/* index in loop. */
	unsigned short tfindex;
	/* Whether this element is the first of coincident elements */
	unsigned char separate;
	/* general use flag */
	unsigned char flag;
	/* If generating element map with island sorting, this stores the island index */
	unsigned short island;
} UvElement;


/* UvElementMap is a container for UvElements of a mesh. It stores some UvElements belonging to the
 * same uv island in sequence and the number of uvs per island so it is possible to access all uvs
 * belonging to an island directly by iterating through the buffer.
 */
typedef struct UvElementMap {
	/* address UvElements by their vertex */
	struct UvElement **vert;
	/* UvElement Store */
	struct UvElement *buf;
	/* Total number of UVs in the layer. Useful to know */
	int totalUVs;
	/* Number of Islands in the mesh */
	int totalIslands;
	/* Stores the starting index in buf where each island begins */
	int *islandIndices;
} UvElementMap;

/* invalid island index is max short. If any one has the patience
 * to make that many islands, he can bite me :p */
#define INVALID_ISLAND 0xFFFF

/* Connectivity data */
typedef struct MeshElemMap {
	int *indices;
	int count;
} MeshElemMap;

/* mapping */
UvVertMap *BKE_mesh_uv_vert_map_create(
        struct MPoly *mpoly, struct MLoop *mloop, struct MLoopUV *mloopuv,
        unsigned int totpoly, unsigned int totvert, int selected, float *limit);
UvMapVert *BKE_mesh_uv_vert_map_get_vert(UvVertMap *vmap, unsigned int v);
void       BKE_mesh_uv_vert_map_free(UvVertMap *vmap);

void BKE_mesh_vert_poly_map_create(
        MeshElemMap **r_map, int **r_mem,
        const struct MPoly *mface, const struct MLoop *mloop,
        int totvert, int totface, int totloop);
void BKE_mesh_vert_loop_map_create(
        MeshElemMap **r_map, int **r_mem,
        const struct MPoly *mface, const struct MLoop *mloop,
        int totvert, int totface, int totloop);
void BKE_mesh_vert_edge_map_create(
        MeshElemMap **r_map, int **r_mem,
        const struct MEdge *medge, int totvert, int totedge);
void BKE_mesh_edge_poly_map_create(
        MeshElemMap **r_map, int **r_mem,
        const struct MEdge *medge, const int totedge,
        const struct MPoly *mpoly, const int totpoly,
        const struct MLoop *mloop, const int totloop);
void BKE_mesh_origindex_map_create(
        MeshElemMap **r_map, int **r_mem,
        const int totorig,
        const int *final_origindex, const int totfinal);

/* smoothgroups */
int *BKE_mesh_calc_smoothgroups(
        const struct MEdge *medge, const int totedge,
        const struct MPoly *mpoly, const int totpoly,
        const struct MLoop *mloop, const int totloop,
        int *r_totgroup, const bool use_bitflags);

/* Loop islands data helpers. */
enum {
	MISLAND_TYPE_VERT = 1,
	MISLAND_TYPE_EDGE = 2,
	MISLAND_TYPE_POLY = 3,
	MISLAND_TYPE_LOOP = 4,
};

typedef struct MeshIslands {
	short item_type;  /* MISLAND_TYPE_... */
	short island_type;  /* MISLAND_TYPE_... */

	int nbr_items;
	int *items_to_islands_idx;

	int nbr_islands;
	MeshElemMap **islands;  /* Array of pointers, one item per island. */

	void *mem;  /* Memory handler, internal use only. */
	size_t allocated_islands;
} MeshIslands;

void BKE_loop_islands_init(MeshIslands *islands, const short item_type, const int num_items, const short island_type);
void BKE_loop_islands_free(MeshIslands *islands);
void BKE_loop_islands_add_island(MeshIslands *islands, const int num_items, int *item_indices,
                                 const int num_island_items, int *island_item_indices);

typedef bool (*loop_island_compute)(struct DerivedMesh *dm, MeshIslands *r_islands);
/* Above vert/UV mapping stuff does not do what we need here, but does things we do not need here.
 * So better keep them separated for now, I think.
 */
bool BKE_loop_poly_island_compute_uv(struct DerivedMesh *dm, MeshIslands *r_islands);

/* Generic ways to map some geometry elements from a source mesh to a dest one. */

typedef struct Mesh2MeshMappingItem {
	int nbr_sources;
	int *indices_src;  /* NULL if no source found. */
	float *weights_src;  /* NULL if no source found, else, always normalized! */
	float hit_distance;  /* FLT_MAX if irrelevant or no source found. */
	int island;  /* For loops only. */
} Mesh2MeshMappingItem;

/* All mapping computing func return this. */
typedef struct Mesh2MeshMapping {
	int nbr_items;
	Mesh2MeshMappingItem *items;  /* Array, one item per dest element. */

	void *mem;  /* Memory handler, internal use only. */
} Mesh2MeshMapping;

/* Helpers! */
void BKE_mesh2mesh_mapping_init(Mesh2MeshMapping *map, const int num_items);
void BKE_mesh2mesh_mapping_free(Mesh2MeshMapping *map);

void BKE_mesh2mesh_mapping_item_define_invalid(Mesh2MeshMapping *map, const int idx);

/* TODO:
 * Add other 'from/to' mapping sources, like e.g. using an UVMap, etc.
 *     http://blenderartists.org/forum/showthread.php?346458-Move-Vertices-to-the-location-of-the-Reference-Mesh-based-on-the-UV-Position
 * We could also use similar topology mappings inside a same mesh
 * (cf. Campbell's 'select face islands from similar topology' wip work).
 * Also, users will have to check, whether we can get rid of some modes here, not sure all will be useful!
 */
enum {
	M2MMAP_USE_VERT                      = 1 << 4,
	M2MMAP_USE_EDGE                      = 1 << 5,
	M2MMAP_USE_LOOP                      = 1 << 6,
	M2MMAP_USE_POLY                      = 1 << 7,

	M2MMAP_USE_NEAREST                   = 1 << 8,
	M2MMAP_USE_NORPROJ                   = 1 << 9,
	M2MMAP_USE_INTERP                    = 1 << 10,
	M2MMAP_USE_NORMAL                    = 1 << 11,

	/* ***** Target's vertices ***** */
	M2MMAP_MODE_VERT                     = 1 << 24,
	/* Nearest source vert. */
	M2MMAP_MODE_VERT_NEAREST             = M2MMAP_MODE_VERT | M2MMAP_USE_VERT | M2MMAP_USE_NEAREST,

	/* Nearest vertex of nearest edge. */
	M2MMAP_MODE_VERT_EDGE_NEAREST        = M2MMAP_MODE_VERT | M2MMAP_USE_EDGE | M2MMAP_USE_NEAREST,
	/* This one uses two verts of selected edge (weighted interpolation). */
	/* Nearest point on nearest edge. */
	M2MMAP_MODE_VERT_EDGEINTERP_NEAREST  = M2MMAP_MODE_VERT | M2MMAP_USE_EDGE | M2MMAP_USE_NEAREST | M2MMAP_USE_INTERP,

	/* Nearest vertex of nearest poly. */
	M2MMAP_MODE_VERT_POLY_NEAREST        = M2MMAP_MODE_VERT | M2MMAP_USE_POLY | M2MMAP_USE_NEAREST,
	/* Those two use all verts of selected poly (weighted interpolation). */
	/* Nearest point on nearest poly. */
	M2MMAP_MODE_VERT_POLYINTERP_NEAREST  = M2MMAP_MODE_VERT | M2MMAP_USE_POLY | M2MMAP_USE_NEAREST | M2MMAP_USE_INTERP,
	/* Point on nearest face hit by ray from target vertex's normal. */
	M2MMAP_MODE_VERT_POLYINTERP_VNORPROJ = M2MMAP_MODE_VERT | M2MMAP_USE_POLY | M2MMAP_USE_NORPROJ | M2MMAP_USE_INTERP,

	/* ***** Target's edges ***** */
	M2MMAP_MODE_EDGE                     = 1 << 25,

	/* Source edge which both vertices are nearest of dest ones. */
	M2MMAP_MODE_EDGE_VERT_NEAREST        = M2MMAP_MODE_EDGE | M2MMAP_USE_VERT | M2MMAP_USE_NEAREST,

	/* Nearest source edge (using mid-point). */
	M2MMAP_MODE_EDGE_NEAREST             = M2MMAP_MODE_EDGE | M2MMAP_USE_EDGE | M2MMAP_USE_NEAREST,

	/* Nearest edge of nearest poly (using mid-point). */
	M2MMAP_MODE_EDGE_POLY_NEAREST        = M2MMAP_MODE_EDGE | M2MMAP_USE_POLY | M2MMAP_USE_NEAREST,

	/* Cast a set of rays from along dest edge, interpolating its vertices' normals, and use hit source edges. */
	M2MMAP_MODE_EDGE_EDGEINTERP_VNORPROJ = M2MMAP_MODE_EDGE | M2MMAP_USE_VERT | M2MMAP_USE_NORPROJ | M2MMAP_USE_INTERP,

	/* ***** Target's loops ***** */
	/* Note: when islands are given to loop mapping func, all loops from the same destination face will always be mapped
	 *       to loops of source faces within a same island, regardless of mapping mode. */
	M2MMAP_MODE_LOOP                     = 1 << 26,

	/* Best normal-matching loop from nearest vert. */
	M2MMAP_MODE_LOOP_NEAREST_LOOPNOR     = M2MMAP_MODE_LOOP | M2MMAP_USE_LOOP | M2MMAP_USE_VERT | M2MMAP_USE_NEAREST | M2MMAP_USE_NORMAL,
	/* Loop from best normal-matching poly from nearest vert. */
	M2MMAP_MODE_LOOP_NEAREST_POLYNOR     = M2MMAP_MODE_LOOP | M2MMAP_USE_POLY | M2MMAP_USE_VERT | M2MMAP_USE_NEAREST | M2MMAP_USE_NORMAL,

	/* Loop from nearest vertex of nearest poly. */
	M2MMAP_MODE_LOOP_POLY_NEAREST        = M2MMAP_MODE_LOOP | M2MMAP_USE_POLY | M2MMAP_USE_NEAREST,
	/* Those two use all verts of selected poly (weighted interpolation). */
	/* Nearest point on nearest poly. */
	M2MMAP_MODE_LOOP_POLYINTERP_NEAREST  = M2MMAP_MODE_LOOP | M2MMAP_USE_POLY | M2MMAP_USE_NEAREST | M2MMAP_USE_INTERP,
	/* Point on nearest face hit by ray from target loop's normal. */
	M2MMAP_MODE_LOOP_POLYINTERP_LNORPROJ = M2MMAP_MODE_LOOP | M2MMAP_USE_POLY | M2MMAP_USE_NORPROJ | M2MMAP_USE_INTERP,

	/* ***** Target's polygons ***** */
	M2MMAP_MODE_POLY                     = 1 << 27,

	/* Nearest source poly. */
	M2MMAP_MODE_POLY_NEAREST             = M2MMAP_MODE_POLY | M2MMAP_USE_POLY | M2MMAP_USE_NEAREST,
	/* Source poly from best normal-matching dest poly. */
	M2MMAP_MODE_POLY_NOR                 = M2MMAP_MODE_POLY | M2MMAP_USE_POLY | M2MMAP_USE_NORMAL,

	/* Project dest poly onto source mesh using its normal, and use interpolation of all intersecting source polys. */
	M2MMAP_MODE_POLY_POLYINTERP_PNORPROJ = M2MMAP_MODE_POLY | M2MMAP_USE_POLY | M2MMAP_USE_NORPROJ | M2MMAP_USE_INTERP,

	/* ***** Same topology, applies to all four elements types. ***** */
	M2MMAP_MODE_TOPOLOGY                 = M2MMAP_MODE_VERT | M2MMAP_MODE_EDGE | M2MMAP_MODE_LOOP | M2MMAP_MODE_POLY,
};

/* TODO add mesh2mesh versions (we'll need mesh versions of bvhtree funcs too, though!). */

void BKE_dm2mesh_mapping_verts_compute(
        const int mode, const struct SpaceTransform *space_transform, const float max_dist, const float ray_radius,
        const struct MVert *verts_dst, const int numverts_dst,
        struct DerivedMesh *dm_src, Mesh2MeshMapping *r_map);

void BKE_dm2mesh_mapping_edges_compute(
        const int mode, const struct SpaceTransform *space_transform, const float max_dist, const float ray_radius,
        const struct MVert *verts_dst, const int numverts_dst, const struct MEdge *edges_dst, const int numedges_dst,
        struct DerivedMesh *dm_src, Mesh2MeshMapping *r_map);

void BKE_dm2mesh_mapping_loops_compute(
        const int mode, const struct SpaceTransform *space_transform, const float max_dist, const float ray_radius,
        struct MVert *verts_dst, const int numverts_dst, struct MEdge *edges_dst, const int numedges_dst,
        struct MLoop *loops_dst, const int numloops_dst, struct MPoly *polys_dst, const int numpolys_dst,
        struct CustomData *ldata_dst, struct CustomData *pdata_dst, const float split_angle_dst,
        struct DerivedMesh *dm_src, loop_island_compute gen_islands_src, struct Mesh2MeshMapping *r_map);

void BKE_dm2mesh_mapping_polys_compute(
        const int mode, const struct SpaceTransform *space_transform, const float max_dist, const float ray_radius,
        struct MVert *verts_dst, const int numverts_dst, struct MLoop *loops_dst, const int numloops_dst,
        struct MPoly *polys_dst, const int numpolys_dst, struct CustomData *pdata_dst, struct DerivedMesh *dm_src,
        struct Mesh2MeshMapping *r_map);


/* No good (portable) way to have exported inlined functions... */
#define BKE_MESH_TESSFACE_VINDEX_ORDER(_mf, _v)  (                          \
    (CHECK_TYPE_INLINE(_mf, MFace *),                                       \
     CHECK_TYPE_INLINE(&(_v), unsigned int *)),                             \
    ((_mf->v1 == _v) ? 0 :                                                  \
     (_mf->v2 == _v) ? 1 :                                                  \
     (_mf->v3 == _v) ? 2 :                                                  \
     (_mf->v4 && _mf->v4 == _v) ? 3 : -1)                                   \
    )

#endif  /* __BKE_MESH_MAPPING_H__ */
