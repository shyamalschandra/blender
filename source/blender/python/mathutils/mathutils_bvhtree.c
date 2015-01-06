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
 * Contributor(s): Lukas Toenne
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/python/mathutils/mathutils_bvhtree.c
 *  \ingroup mathutils
 *
 * This file defines the 'mathutils.bvhtree' module, a general purpose module to access
 * blenders bvhtree for mesh surface nearest-element search and ray casting.
 */

#include <Python.h>

#include "MEM_guardedalloc.h"

#include "BLI_utildefines.h"
#include "BLI_kdopbvh.h"
#include "BLI_math.h"

#include "DNA_object_types.h"

#include "BKE_bvhutils.h"
#include "BKE_customdata.h"
#include "BKE_DerivedMesh.h"
#include "BKE_editmesh_bvh.h"

#include "bmesh.h"

#include "../generic/py_capi_utils.h"
#include "../generic/python_utildefines.h"
#include "../bmesh/bmesh_py_types.h"

#include "mathutils.h"
#include "mathutils_bvhtree.h"  /* own include */

#include "BLI_strict_flags.h"

typedef struct {
	PyObject_HEAD
} PyBVHTree;

typedef struct {
	PyBVHTree base;
	/* Object DerivedMesh data */
	Object *ob;
	BVHTreeFromMesh meshdata;
	bool use_poly_index;
} PyBVHTree_DerivedMesh;

typedef struct {
	PyBVHTree base;
	/* BMesh data */
	BMBVHTree *bmdata;
	BMLoop *(*bmlooptris)[3];
	int bmtotlooptris;
} PyBVHTree_BMesh;

/* -------------------------------------------------------------------- */
/* Utility helper functions */

static int dm_tessface_to_poly_index(DerivedMesh *dm, int tessface_index)
{
	if (tessface_index != ORIGINDEX_NONE && tessface_index < dm->getNumTessFaces(dm)) {
		/* double lookup */
		const int *index_mf_to_mpoly;
		if ((index_mf_to_mpoly = dm->getTessFaceDataArray(dm, CD_ORIGINDEX))) {
			const int *index_mp_to_orig = dm->getPolyDataArray(dm, CD_ORIGINDEX);
			return DM_origindex_mface_mpoly(index_mf_to_mpoly, index_mp_to_orig, tessface_index);
		}
	}

	return ORIGINDEX_NONE;
}

static PyObject *bvhtree_ray_hit_to_py(const float co[3], const float no[3], int index, float dist)
{
	PyObject *py_retval = PyTuple_New(4);
	PyTuple_SET_ITEMS(py_retval,
	        Vector_CreatePyObject(co, 3, NULL),
	        Vector_CreatePyObject(no, 3, NULL),
	        PyLong_FromLong(index),
	        PyFloat_FromDouble(dist));
	return py_retval;
}

static PyObject *bvhtree_nearest_to_py(const float co[3], const float no[3], int index, float dist_sq)
{
	PyObject *py_retval = PyTuple_New(4);
	PyTuple_SET_ITEMS(py_retval,
	        Vector_CreatePyObject(co, 3, NULL),
	        Vector_CreatePyObject(no, 3, NULL),
	        PyLong_FromLong(index),
	        PyFloat_FromDouble(dist_sq));
	return py_retval;
}

/* -------------------------------------------------------------------- */
/* BVHTree */

static int PyBVHTree__tp_init(PyBVHTree *UNUSED(self), PyObject *UNUSED(args), PyObject *UNUSED(kwargs))
{
	return 0;
}

static void PyBVHTree__tp_dealloc(PyBVHTree *self)
{
	Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyMethodDef PyBVHTree_methods[] = {
	{NULL, NULL, 0, NULL}
};

PyDoc_STRVAR(py_BVHTree_doc,
"BVH tree based on :class:`Mesh` data.\n"
);
PyTypeObject PyBVHTree_Type = {
	PyVarObject_HEAD_INIT(NULL, 0)
	"BVHTree",                                   /* tp_name */
	sizeof(PyBVHTree),                           /* tp_basicsize */
	0,                                           /* tp_itemsize */
	/* methods */
	(destructor)PyBVHTree__tp_dealloc,           /* tp_dealloc */
	NULL,                                        /* tp_print */
	NULL,                                        /* tp_getattr */
	NULL,                                        /* tp_setattr */
	NULL,                                        /* tp_compare */
	NULL,                                        /* tp_repr */
	NULL,                                        /* tp_as_number */
	NULL,                                        /* tp_as_sequence */
	NULL,                                        /* tp_as_mapping */
	NULL,                                        /* tp_hash */
	NULL,                                        /* tp_call */
	NULL,                                        /* tp_str */
	NULL,                                        /* tp_getattro */
	NULL,                                        /* tp_setattro */
	NULL,                                        /* tp_as_buffer */
	Py_TPFLAGS_DEFAULT,                          /* tp_flags */
	py_BVHTree_doc,                              /* Documentation string */
	NULL,                                        /* tp_traverse */
	NULL,                                        /* tp_clear */
	NULL,                                        /* tp_richcompare */
	0,                                           /* tp_weaklistoffset */
	NULL,                                        /* tp_iter */
	NULL,                                        /* tp_iternext */
	(struct PyMethodDef *)PyBVHTree_methods,     /* tp_methods */
	NULL,                                        /* tp_members */
	NULL,                                        /* tp_getset */
	NULL,                                        /* tp_base */
	NULL,                                        /* tp_dict */
	NULL,                                        /* tp_descr_get */
	NULL,                                        /* tp_descr_set */
	0,                                           /* tp_dictoffset */
	(initproc)PyBVHTree__tp_init,                /* tp_init */
	(allocfunc)PyType_GenericAlloc,              /* tp_alloc */
	(newfunc)PyType_GenericNew,                  /* tp_new */
	(freefunc)0,                                 /* tp_free */
	NULL,                                        /* tp_is_gc */
	NULL,                                        /* tp_bases */
	NULL,                                        /* tp_mro */
	NULL,                                        /* tp_cache */
	NULL,                                        /* tp_subclasses */
	NULL,                                        /* tp_weaklist */
	(destructor) NULL                            /* tp_del */
};

/* -------------------------------------------------------------------- */
/* BVHTreeDerivedMesh */

static int PyBVHTreeDerivedMesh__tp_init(PyBVHTree_DerivedMesh *self, PyObject *args, PyObject *kwargs)
{
	BVHTreeFromMesh *meshdata = &self->meshdata;
	const char *keywords[] = {"object", "type", NULL};
	
	PyObject *py_ob;
	Object *ob;
	const char *type = "POLYS";
	
	if (PyBVHTree_Type.tp_init((PyObject *)self, args, kwargs) < 0)
		return -1;
	
	if (!PyArg_ParseTupleAndKeywords(args, kwargs, (char *)"O|s:BVHTreeDerivedMesh", (char **)keywords,
	                                 &py_ob, &type))
	{
		return -1;
	}
	
	ob = PyC_RNA_AsPointer(py_ob, "Object");
	if (!ob) {
		return -1;
	}
	
	if (ob->derivedFinal == NULL) {
		PyErr_Format(PyExc_ValueError, "Object '%.200s' has no mesh data to be used for BVH tree", ob->id.name + 2);
		return -1;
	}
	
	self->ob = ob;
	
	if (STREQ(type, "FACES")) {
		bvhtree_from_mesh_faces(meshdata, ob->derivedFinal, 0.0f, 4, 6);
		self->use_poly_index = false;
	}
	else if (STREQ(type, "POLYS")) {
		bvhtree_from_mesh_faces(meshdata, ob->derivedFinal, 0.0f, 4, 6);
		self->use_poly_index = true;
	}
	else if (STREQ(type, "VERTS")) {
		bvhtree_from_mesh_verts(meshdata, ob->derivedFinal, 0.0f, 4, 6);
		self->use_poly_index = false;
	}
	else if (STREQ(type, "EDGES")) {
		bvhtree_from_mesh_edges(meshdata, ob->derivedFinal, 0.0f, 4, 6);
		self->use_poly_index = false;
	}
	else {
		PyErr_Format(PyExc_ValueError, "'type' must be 'FACES', 'POLYS', 'VERTS' or 'EDGES', not '%.200s'", type);
		return -1;
	}
	
	return 0;
}

static void PyBVHTreeDerivedMesh__tp_dealloc(PyBVHTree_DerivedMesh *self)
{
	BVHTreeFromMesh *meshdata = &self->meshdata;
	
	self->ob = NULL;
	free_bvhtree_from_mesh(meshdata);
	
	Py_TYPE(self)->tp_free((PyObject *)self);
}

PyDoc_STRVAR(py_BVHTreeDerivedMesh_ray_cast_doc,
".. method:: ray_cast(ray_start, ray_end)\n"
"\n"
"   Cast a ray onto the mesh.\n"
"\n"
"   :arg ray_start: Start location of the ray in object space.\n"
"   :type ray_start: :class:`Vector`\n"
"   :arg ray_end: End location of the ray in object space.\n"
"   :type ray_end: :class:`Vector`\n"
"   :return: Returns a tuple (:class:`Vector` location, :class:`Vector` normal, int index, float distance), index==-1 if no hit was found.\n"
"   :rtype: :class:`tuple`\n"
);
static PyObject *py_BVHTreeDerivedMesh_ray_cast(PyBVHTree_DerivedMesh *self, PyObject *args)
{
	const char *error_prefix = "ray_cast";
	
	BVHTreeFromMesh *meshdata = &self->meshdata;
	Object *ob = self->ob;
	
	PyObject *py_ray_start, *py_ray_end;
	float ray_start[3], ray_end[3];
	float ray_nor[3], ray_len;
	
	if (!PyArg_ParseTuple(args, (char *)"OO:ray_cast", &py_ray_start, &py_ray_end))
	{
		return NULL;
	}
	
	if (mathutils_array_parse(ray_start, 2, 3, py_ray_start, error_prefix) == -1 ||
	    mathutils_array_parse(ray_end, 2, 3, py_ray_end, error_prefix) == -1)
	{
		return NULL;
	}
	
	sub_v3_v3v3(ray_nor, ray_end, ray_start);
	ray_len = normalize_v3(ray_nor);
	
	/* may fail if the mesh has no faces, in that case the ray-cast misses */
	if (meshdata->tree && meshdata->raycast_callback && ob->derivedFinal) {
		BVHTreeRayHit hit;
		hit.dist = ray_len;
		hit.index = -1;
		
		if (BLI_bvhtree_ray_cast(meshdata->tree, ray_start, ray_nor, 0.0f, &hit,
		                         meshdata->raycast_callback, meshdata) != -1)
		{
			if (hit.dist <= ray_len) {
				int ret_index = self->use_poly_index ? dm_tessface_to_poly_index(ob->derivedFinal, hit.index) : hit.index;
				return bvhtree_ray_hit_to_py(hit.co, hit.no, ret_index, hit.dist);
			}
		}
	}
	
	return bvhtree_ray_hit_to_py(NULL, NULL, -1, 0.0f);
}

PyDoc_STRVAR(py_BVHTreeDerivedMesh_find_nearest_doc,
".. method:: find_nearest(point, max_dist=1.84467e+19)\n"
"\n"
"   Find the nearest element to a point.\n"
"\n"
"   :arg point: Find nearest element to this point.\n"
"   :type ray_start: :class:`Vector`\n"
"   :art max_dist: Maximum search distance\n"
"   :type max_dist: :float\n"
"   :return: Returns a tuple (:class:`Vector` location, :class:`Vector` normal, int index, float distance_squared), index==-1 if no hit was found.\n"
"   :rtype: :class:`tuple`\n"
);
static PyObject *py_BVHTreeDerivedMesh_find_nearest(PyBVHTree_DerivedMesh *self, PyObject *args)
{
	const char *error_prefix = "find_nearest";

	BVHTreeFromMesh *meshdata = &self->meshdata;
	Object *ob = self->ob;
	
	PyObject *py_point;
	float point[3];
	float max_dist = 1.844674352395373e+19f;
	BVHTreeNearest nearest;
	
	if (!PyArg_ParseTuple(args, (char *)"O|f:find_nearest", &py_point, &max_dist))
	{
		return NULL;
	}
	
	if (mathutils_array_parse(point, 2, 3, py_point, error_prefix) == -1) {
		return NULL;
	}
	
	nearest.index = -1;
	nearest.dist_sq = max_dist * max_dist;
	
	/* may fail if the mesh has no faces, in that case the ray-cast misses */
	if (meshdata->tree && meshdata->nearest_callback && ob->derivedFinal) {
		if (BLI_bvhtree_find_nearest(meshdata->tree, point, &nearest,
		                             meshdata->nearest_callback, meshdata) != -1)
		{
			int ret_index = self->use_poly_index ? dm_tessface_to_poly_index(ob->derivedFinal, nearest.index) : nearest.index;
			return bvhtree_nearest_to_py(nearest.co, nearest.no, ret_index, nearest.dist_sq);
		}
	}
	
	return bvhtree_ray_hit_to_py(NULL, NULL, -1, 0.0f);
}

static PyMethodDef PyBVHTreeDerivedMesh_methods[] = {
	{"ray_cast", (PyCFunction)py_BVHTreeDerivedMesh_ray_cast, METH_VARARGS, py_BVHTreeDerivedMesh_ray_cast_doc},
	{"find_nearest", (PyCFunction)py_BVHTreeDerivedMesh_find_nearest, METH_VARARGS, py_BVHTreeDerivedMesh_find_nearest_doc},
	{NULL, NULL, 0, NULL}
};

PyDoc_STRVAR(py_BVHTreeDerivedMesh_doc,
".. method:: BVHTreeDerivedMesh(object, type='POLYS')\n"
"\n"
"   BVH tree based on :class:`Object` mesh data.\n"
"\n"
"   :arg object: Mesh object.\n"
"   :type object: :class:`Object`\n"
"   :art type: Maximum search distance\n"
"   :type type: :string in ['POLYS', 'FACES', 'VERTS', 'EDGES']\n"
);
PyTypeObject PyBVHTreeDerivedMesh_Type = {
	PyVarObject_HEAD_INIT(NULL, 0)
	"BVHTreeDerivedMesh",                        /* tp_name */
	sizeof(PyBVHTree_DerivedMesh),                /* tp_basicsize */
	0,                                           /* tp_itemsize */
	/* methods */
	(destructor)PyBVHTreeDerivedMesh__tp_dealloc,/* tp_dealloc */
	NULL,                                        /* tp_print */
	NULL,                                        /* tp_getattr */
	NULL,                                        /* tp_setattr */
	NULL,                                        /* tp_compare */
	NULL,                                        /* tp_repr */
	NULL,                                        /* tp_as_number */
	NULL,                                        /* tp_as_sequence */
	NULL,                                        /* tp_as_mapping */
	NULL,                                        /* tp_hash */
	NULL,                                        /* tp_call */
	NULL,                                        /* tp_str */
	NULL,                                        /* tp_getattro */
	NULL,                                        /* tp_setattro */
	NULL,                                        /* tp_as_buffer */
	Py_TPFLAGS_DEFAULT,                          /* tp_flags */
	py_BVHTreeDerivedMesh_doc,                   /* Documentation string */
	NULL,                                        /* tp_traverse */
	NULL,                                        /* tp_clear */
	NULL,                                        /* tp_richcompare */
	0,                                           /* tp_weaklistoffset */
	NULL,                                        /* tp_iter */
	NULL,                                        /* tp_iternext */
	(struct PyMethodDef *)PyBVHTreeDerivedMesh_methods, /* tp_methods */
	NULL,                                        /* tp_members */
	NULL,                                        /* tp_getset */
	NULL,                                        /* tp_base */
	NULL,                                        /* tp_dict */
	NULL,                                        /* tp_descr_get */
	NULL,                                        /* tp_descr_set */
	0,                                           /* tp_dictoffset */
	(initproc)PyBVHTreeDerivedMesh__tp_init,     /* tp_init */
	(allocfunc)PyType_GenericAlloc,              /* tp_alloc */
	(newfunc)PyType_GenericNew,                  /* tp_new */
	(freefunc)0,                                 /* tp_free */
	NULL,                                        /* tp_is_gc */
	NULL,                                        /* tp_bases */
	NULL,                                        /* tp_mro */
	NULL,                                        /* tp_cache */
	NULL,                                        /* tp_subclasses */
	NULL,                                        /* tp_weaklist */
	(destructor) NULL                            /* tp_del */
};

/* -------------------------------------------------------------------- */
/* BVHTreeBMesh */


static int PyBVHTreeBMesh__tp_init(PyBVHTree_BMesh *self, PyObject *args, PyObject *kwargs)
{
	const char *keywords[] = {"bm", NULL};
	
	PyObject *py_bm;
	BMesh *bm;
	int tottri;
	int flag = 0; /* TODO add optional RESPECT_SELECT and RESPECT_HIDDEN flag options */
	
	if (PyBVHTree_Type.tp_init((PyObject *)self, args, kwargs) < 0)
		return -1;
	
	if (!PyArg_ParseTupleAndKeywords(args, kwargs, (char *)"O!:from_bmesh", (char **)keywords,
	                                 &BPy_BMesh_Type, &py_bm))
	{
		return -1;
	}
	bm = ((BPy_BMesh *)py_bm)->bm;
	
	self->bmtotlooptris = poly_to_tri_count(bm->totface, bm->totloop);
	self->bmlooptris = MEM_mallocN(sizeof(*self->bmlooptris) * (size_t)self->bmtotlooptris, __func__);
	BM_bmesh_calc_tessellation(bm, self->bmlooptris, &tottri);
	
	self->bmdata = BKE_bmbvh_new(bm, self->bmlooptris, tottri, flag, NULL, false);
	
	return 0;
}

static void PyBVHTreeBMesh__tp_dealloc(PyBVHTree_BMesh *self)
{
	if (self->bmlooptris) {
		MEM_freeN(self->bmlooptris);
		self->bmlooptris = NULL;
		self->bmtotlooptris = 0;
	}
	if (self->bmdata) {
		BKE_bmbvh_free(self->bmdata);
		self->bmdata = NULL;
	}
	
	Py_TYPE(self)->tp_free((PyObject *)self);
}

PyDoc_STRVAR(py_BVHTreeBMesh_ray_cast_doc,
".. method:: ray_cast(ray_start, ray_end)\n"
"\n"
"   Cast a ray onto the mesh.\n"
"\n"
"   :arg ray_start: Start location of the ray in object space.\n"
"   :type ray_start: :class:`Vector`\n"
"   :arg ray_end: End location of the ray in object space.\n"
"   :type ray_end: :class:`Vector`\n"
"   :return: Returns a tuple (:class:`Vector` location, :class:`Vector` normal, int index, float distance), index==-1 if no hit was found.\n"
"   :rtype: :class:`tuple`\n"
);
static PyObject *py_BVHTreeBMesh_ray_cast(PyBVHTree_BMesh *self, PyObject *args)
{
	const char *error_prefix = "ray_cast";
	
	BMBVHTree *bmdata = self->bmdata;
	
	PyObject *py_ray_start, *py_ray_end;
	float ray_start[3], ray_end[3];
	float ray_nor[3], ray_len;
	
	if (!PyArg_ParseTuple(args, (char *)"OO:ray_cast", &py_ray_start, &py_ray_end))
	{
		return NULL;
	}
	
	if (mathutils_array_parse(ray_start, 2, 3, py_ray_start, error_prefix) == -1 ||
	    mathutils_array_parse(ray_end, 2, 3, py_ray_end, error_prefix) == -1)
	{
		return NULL;
	}
	
	sub_v3_v3v3(ray_nor, ray_end, ray_start);
	ray_len = normalize_v3(ray_nor);
	
	/* may fail if the mesh has no faces, in that case the ray-cast misses */
	if (bmdata) {
		BMFace *hit_face;
		float hit_co[3], hit_dist;
		
		hit_dist = ray_len;
		
		hit_face = BKE_bmbvh_ray_cast(bmdata, ray_start, ray_nor, 0.0f, &hit_dist, hit_co, NULL);
		if (hit_face && hit_dist <= ray_len) {
			int ret_index = BM_elem_index_get(hit_face);
			return bvhtree_ray_hit_to_py(hit_co, hit_face->no, ret_index, hit_dist);
		}
	}
	
	return bvhtree_ray_hit_to_py(NULL, NULL, -1, 0.0f);
}

PyDoc_STRVAR(py_BVHTreeBMesh_find_nearest_doc,
".. method:: find_nearest(point, max_dist=1.84467e+19)\n"
"\n"
"   Find the nearest element to a point.\n"
"\n"
"   :arg point: Find nearest element to this point.\n"
"   :type ray_start: :class:`Vector`\n"
"   :art max_dist: Maximum search distance\n"
"   :type max_dist: :float\n"
"   :return: Returns a tuple (:class:`Vector` location, :class:`Vector` normal, int index, float distance_squared), index==-1 if no hit was found.\n"
"   :rtype: :class:`tuple`\n"
);
static PyObject *py_BVHTreeBMesh_find_nearest(PyBVHTree_BMesh *self, PyObject *args)
{
	const char *error_prefix = "find_nearest";
	
	BMBVHTree *bmdata = self->bmdata;
	
	PyObject *py_point;
	float point[3];
	float max_dist = 1.844674352395373e+19f;
	
	if (!PyArg_ParseTuple(args, (char *)"O|f:find_nearest", &py_point, &max_dist))
	{
		return NULL;
	}
	
	if (mathutils_array_parse(point, 2, 3, py_point, error_prefix) == -1)
		return NULL;
	
	/* may fail if the mesh has no faces, in that case the ray-cast misses */
	if (bmdata) {
		BMVert *nearest_vert;
		
		nearest_vert = BKE_bmbvh_find_vert_closest(bmdata, point, max_dist);
		if (nearest_vert) {
			return bvhtree_ray_hit_to_py(nearest_vert->co, nearest_vert->no, BM_elem_index_get(nearest_vert), len_squared_v3v3(point, nearest_vert->co));
		}
	}
	
	return bvhtree_ray_hit_to_py(NULL, NULL, -1, 0.0f);
}

static PyMethodDef PyBVHTreeBMesh_methods[] = {
	{"ray_cast", (PyCFunction)py_BVHTreeBMesh_ray_cast, METH_VARARGS, py_BVHTreeBMesh_ray_cast_doc},
	{"find_nearest", (PyCFunction)py_BVHTreeBMesh_find_nearest, METH_VARARGS, py_BVHTreeBMesh_find_nearest_doc},
	{NULL, NULL, 0, NULL}
};

PyDoc_STRVAR(py_BVHTreeBMesh_doc,
".. method:: BVHTreeBMesh(bm)\n"
"\n"
"   BVH tree based on :class:`BMesh` data.\n"
"\n"
"   :arg bm: BMesh data.\n"
"   :type bm: :class:`BMesh`\n"
);
PyTypeObject PyBVHTreeBMesh_Type = {
	PyVarObject_HEAD_INIT(NULL, 0)
	"BVHTreeBMesh",                              /* tp_name */
	sizeof(PyBVHTree_BMesh),                     /* tp_basicsize */
	0,                                           /* tp_itemsize */
	/* methods */
	(destructor)PyBVHTreeBMesh__tp_dealloc,      /* tp_dealloc */
	NULL,                                        /* tp_print */
	NULL,                                        /* tp_getattr */
	NULL,                                        /* tp_setattr */
	NULL,                                        /* tp_compare */
	NULL,                                        /* tp_repr */
	NULL,                                        /* tp_as_number */
	NULL,                                        /* tp_as_sequence */
	NULL,                                        /* tp_as_mapping */
	NULL,                                        /* tp_hash */
	NULL,                                        /* tp_call */
	NULL,                                        /* tp_str */
	NULL,                                        /* tp_getattro */
	NULL,                                        /* tp_setattro */
	NULL,                                        /* tp_as_buffer */
	Py_TPFLAGS_DEFAULT,                          /* tp_flags */
	py_BVHTreeBMesh_doc,                         /* Documentation string */
	NULL,                                        /* tp_traverse */
	NULL,                                        /* tp_clear */
	NULL,                                        /* tp_richcompare */
	0,                                           /* tp_weaklistoffset */
	NULL,                                        /* tp_iter */
	NULL,                                        /* tp_iternext */
	(struct PyMethodDef *)PyBVHTreeBMesh_methods,/* tp_methods */
	NULL,                                        /* tp_members */
	NULL,                                        /* tp_getset */
	NULL,                                        /* tp_base */
	NULL,                                        /* tp_dict */
	NULL,                                        /* tp_descr_get */
	NULL,                                        /* tp_descr_set */
	0,                                           /* tp_dictoffset */
	(initproc)PyBVHTreeBMesh__tp_init,           /* tp_init */
	(allocfunc)PyType_GenericAlloc,              /* tp_alloc */
	(newfunc)PyType_GenericNew,                  /* tp_new */
	(freefunc)0,                                 /* tp_free */
	NULL,                                        /* tp_is_gc */
	NULL,                                        /* tp_bases */
	NULL,                                        /* tp_mro */
	NULL,                                        /* tp_cache */
	NULL,                                        /* tp_subclasses */
	NULL,                                        /* tp_weaklist */
	(destructor) NULL                            /* tp_del */
};

/* -------------------------------------------------------------------- */
/* Module definition */

PyDoc_STRVAR(py_bvhtree_doc,
"BVH tree structures for proximity searches and ray casts on meshes."
);
static struct PyModuleDef bvhtree_moduledef = {
	PyModuleDef_HEAD_INIT,
	"mathutils.bvhtree",                         /* m_name */
	py_bvhtree_doc,                              /* m_doc */
	0,                                           /* m_size */
	NULL,                                        /* m_methods */
	NULL,                                        /* m_reload */
	NULL,                                        /* m_traverse */
	NULL,                                        /* m_clear */
	NULL                                         /* m_free */
};

PyMODINIT_FUNC PyInit_mathutils_bvhtree(void)
{
	PyObject *m = PyModule_Create(&bvhtree_moduledef);

	if (m == NULL) {
		return NULL;
	}

	/* Register classes */
	if (PyType_Ready(&PyBVHTree_Type) < 0) {
		return NULL;
	}
	
	PyBVHTreeDerivedMesh_Type.tp_base = &PyBVHTree_Type;
	if (PyType_Ready(&PyBVHTreeDerivedMesh_Type) < 0) {
		return NULL;
	}
	
	PyBVHTreeBMesh_Type.tp_base = &PyBVHTree_Type;
	if (PyType_Ready(&PyBVHTreeBMesh_Type) < 0) {
		return NULL;
	}

	PyModule_AddObject(m, "BVHTree", (PyObject *) &PyBVHTree_Type);
	PyModule_AddObject(m, "BVHTreeDerivedMesh", (PyObject *) &PyBVHTreeDerivedMesh_Type);
	PyModule_AddObject(m, "BVHTreeBMesh", (PyObject *) &PyBVHTreeBMesh_Type);

	return m;
}
