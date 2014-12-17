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

#include "../generic/py_capi_utils.h"

#include "mathutils.h"
#include "mathutils_bvhtree.h"  /* own include */

#include "BLI_strict_flags.h"

typedef struct {
	PyObject_HEAD
	BVHTreeFromMesh treedata;
	Object *ob;
} PyObjectBVHTree;

typedef struct {
	PyObject_HEAD
	BMBVHTree *treedata;
} PyBMeshBVHTree;

/* -------------------------------------------------------------------- */
/* Utility helper functions */

static bool parse_vector(PyObject *vec, float value[3])
{
	if (BaseMath_ReadCallback((VectorObject *)vec) == -1)
		return false;
	
	value[0] = ((VectorObject *)vec)->vec[0];
	value[1] = ((VectorObject *)vec)->vec[1];
	if (((VectorObject *)vec)->size > 2)
		value[2] = ((VectorObject *)vec)->vec[2];
	else
		value[2] = 0.0f;  /* if its a 2d vector then set the z to be zero */
	
	return true;
}

static int dm_tessface_to_poly_index(DerivedMesh *dm, int tessface_index)
{
	if (tessface_index != ORIGINDEX_NONE) {
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

	PyTuple_SET_ITEM(py_retval, 0, Vector_CreatePyObject((float *)co, 3, Py_NEW, NULL));
	PyTuple_SET_ITEM(py_retval, 1, Vector_CreatePyObject((float *)no, 3, Py_NEW, NULL));
	PyTuple_SET_ITEM(py_retval, 2, PyLong_FromLong(index));
	PyTuple_SET_ITEM(py_retval, 3, PyFloat_FromDouble(dist));

	return py_retval;
}

/* -------------------------------------------------------------------- */
/* ObjectBVHTree */

static int PyObjectBVHTree__tp_init(PyObjectBVHTree *self, PyObject *args, PyObject *kwargs)
{
	const char *keywords[] = {"object", NULL};
	
	PyObject *py_ob;
	Object *ob;
	
	if (!PyArg_ParseTupleAndKeywords(args, kwargs, (char *)"O:from_faces", (char **)keywords,
	                                 &py_ob))
	{
		return NULL;
	}
	
	ob = PyC_RNA_AsPointer(py_ob, "Object");
	if (!ob) {
		return -1;
	}
	
	memset(&self->treedata, 0, sizeof(BVHTreeFromMesh));
	self->ob = ob;
	
	return 0;
}

static void PyObjectBVHTree__tp_dealloc(PyObjectBVHTree *self)
{
	free_bvhtree_from_mesh(&self->treedata);
	Py_TYPE(self)->tp_free((PyObject *)self);
}

PyDoc_STRVAR(py_ObjectBVHTree_from_faces_doc,
".. method:: from_faces(object)\n"
"\n"
"   Construct the BVHTree from mesh faces.\n"
"\n"
"   :arg object: Point 3d position.\n"
"   :type object: :class:`Object`\n"
);
static PyObject *py_ObjectBVHTree_from_faces(PyObjectBVHTree *self)
{
	Object *ob = self->ob;
	BVHTreeFromMesh *treedata = &self->treedata;
	
	/* free existing data */
	free_bvhtree_from_mesh(treedata);
	
	if (ob->derivedFinal == NULL) {
		PyErr_Format(PyExc_ValueError, "Object '%.200s' has no mesh data to be used for BVH tree", ob->id.name + 2);
		return NULL;
	}
	
	/* no need to managing allocation or freeing of the BVH data. this is generated and freed as needed */
	bvhtree_from_mesh_faces(treedata, ob->derivedFinal, 0.0f, 4, 6);
	
	Py_RETURN_NONE;
}

PyDoc_STRVAR(py_ObjectBVHTree_free_doc,
".. method:: clear()\n"
"\n"
"   Remove all BVH data.\n"
);
static PyObject *py_ObjectBVHTree_free(PyObjectBVHTree *self)
{
	BVHTreeFromMesh *treedata = &self->treedata;
	
	/* free existing data */
	free_bvhtree_from_mesh(treedata);
	
	Py_RETURN_NONE;
}

PyDoc_STRVAR(py_ObjectBVHTree_ray_cast_doc,
".. method:: ray_cast(ray_start, ray_end)\n"
"\n"
"   Cast a ray onto the mesh.\n"
"\n"
"   :arg ray_start: Start location of the ray in object space.\n"
"   :type ray_start: :class:`Vector`\n"
"   :arg ray_end: End location of the ray in object space.\n"
"   :type ray_end: :class:`Vector`\n"
"   :return: Returns a tuple (:class:`Vector` location, :class:`Vector` normal, int index), index==-1 if no hit was found.\n"
"   :rtype: :class:`tuple`\n"
);
static PyObject *py_ObjectBVHTree_ray_cast(PyObjectBVHTree *self, PyObject *args, PyObject *kwargs)
{
	static const float ZERO[3] = {0.0f, 0.0f, 0.0f};
	
	BVHTreeFromMesh *treedata = &self->treedata;
	Object *ob = self->ob;
	const char *keywords[] = {"ray_start", "ray_end", NULL};
	
	PyObject *py_start, *py_end;
	float start[3], end[3];
	
	if (!PyArg_ParseTupleAndKeywords(args, kwargs, (char *)"O!O!:ray_cast", (char **)keywords,
	                                 &vector_Type, &py_start,
	                                 &vector_Type, &py_end))
	{
		return NULL;
	}
	
	if (!parse_vector(py_start, start))
		return NULL;
	if (!parse_vector(py_end, end))
		return NULL;
	
	/* may fail if the mesh has no faces, in that case the ray-cast misses */
	if (treedata->tree && ob->derivedFinal) {
		BVHTreeRayHit hit;
		float ray_nor[3], dist;
		sub_v3_v3v3(ray_nor, end, start);

		dist = hit.dist = normalize_v3(ray_nor);
		hit.index = -1;
		
		if (BLI_bvhtree_ray_cast(treedata->tree, start, ray_nor, 0.0f, &hit,
		                         treedata->raycast_callback, treedata) != -1)
		{
			if (hit.dist <= dist) {
				int poly_index = dm_tessface_to_poly_index(ob->derivedFinal, hit.index);
				
				return bvhtree_ray_hit_to_py(hit.co, hit.no, poly_index, hit.dist);
			}
		}
	}
	
	return bvhtree_ray_hit_to_py(ZERO, ZERO, -1, 0.0f);
}


static PyMethodDef PyObjectBVHTree_methods[] = {
	{"from_faces", (PyCFunction)py_ObjectBVHTree_from_faces, METH_VARARGS | METH_KEYWORDS, py_ObjectBVHTree_from_faces_doc},
	{"free", (PyCFunction)py_ObjectBVHTree_free, METH_NOARGS, py_ObjectBVHTree_free_doc},
	{"ray_cast", (PyCFunction)py_ObjectBVHTree_ray_cast, METH_VARARGS | METH_KEYWORDS, py_ObjectBVHTree_ray_cast_doc},
	{NULL, NULL, 0, NULL}
};

PyDoc_STRVAR(py_ObjectBVHTree_doc,
"BVH tree based on :class:`Mesh` data.\n"
);
PyTypeObject PyObjectBVHTree_Type = {
	PyVarObject_HEAD_INIT(NULL, 0)
	"ObjectBVHTree",                             /* tp_name */
	sizeof(PyObjectBVHTree),                     /* tp_basicsize */
	0,                                           /* tp_itemsize */
	/* methods */
	(destructor)PyObjectBVHTree__tp_dealloc,     /* tp_dealloc */
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
	py_ObjectBVHTree_doc,                          /* Documentation string */
	NULL,                                        /* tp_traverse */
	NULL,                                        /* tp_clear */
	NULL,                                        /* tp_richcompare */
	0,                                           /* tp_weaklistoffset */
	NULL,                                        /* tp_iter */
	NULL,                                        /* tp_iternext */
	(struct PyMethodDef *)PyObjectBVHTree_methods, /* tp_methods */
	NULL,                                        /* tp_members */
	NULL,                                        /* tp_getset */
	NULL,                                        /* tp_base */
	NULL,                                        /* tp_dict */
	NULL,                                        /* tp_descr_get */
	NULL,                                        /* tp_descr_set */
	0,                                           /* tp_dictoffset */
	(initproc)PyObjectBVHTree__tp_init,          /* tp_init */
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

	/* Register the 'ObjectBVHTree' class */
	if (PyType_Ready(&PyObjectBVHTree_Type)) {
		return NULL;
	}
	PyModule_AddObject(m, "ObjectBVHTree", (PyObject *) &PyObjectBVHTree_Type);
	
#if 0
	/* Register the 'BMeshBVHTree' class */
	if (PyType_Ready(&PyBMeshBVHTree_Type)) {
		return NULL;
	}
	PyModule_AddObject(m, "BMeshBVHTree", (PyObject *) &PyBMeshBVHTree_Type);
#endif

	return m;
}
