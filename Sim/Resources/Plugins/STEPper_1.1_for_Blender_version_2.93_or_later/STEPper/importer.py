# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#
# Copyright 2021 Tommi Hyppänen

import sys
from os.path import dirname

file_dirname = dirname(__file__)
if file_dirname not in sys.path:
    sys.path.append(file_dirname)

import importlib
import os
import random
from collections import defaultdict
from dataclasses import dataclass, field

import numpy as np

# import trimesh works in dev, but not in deploy
from . import trimesh

importlib.reload(trimesh)
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_NurbsConvert, BRepBuilderAPI_Transform
from OCC.Core.BRepLProp import BRepLProp_SLProps
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRepTools import breptools
from OCC.Core.GeomAPI import GeomAPI_ProjectPointOnSurf
from OCC.Core.GeomConvert import geomconvert_SurfaceToBSplineSurface
from OCC.Core.GeomLProp import GeomLProp_SLProps
from OCC.Core.gp import gp, gp_Dir, gp_Pln, gp_Pnt, gp_Pnt2d, gp_Trsf, gp_Vec, gp_XYZ

# from OCC.Core.Standard import Standard_Real
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.IMeshTools import IMeshTools_Parameters
from OCC.Core.Interface import Interface_Static_SetIVal
from OCC.Core.Poly import poly
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCC.Core.STEPCAFControl import STEPCAFControl_Reader
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TCollection import TCollection_ExtendedString
from OCC.Core.TColStd import TColStd_SequenceOfAsciiString
from OCC.Core.TDF import TDF_Label, TDF_LabelSequence
from OCC.Core.TDocStd import TDocStd_Document
from OCC.Core.TopAbs import (
    TopAbs_COMPOUND,
    TopAbs_EDGE,
    TopAbs_FACE,
    TopAbs_FORWARD,
    TopAbs_INTERNAL,
    TopAbs_REVERSED,
    TopAbs_SHELL,
    TopAbs_SOLID,
    TopAbs_VERTEX,
    TopAbs_WIRE,
    topabs_ShapeTypeToString,
)
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopLoc import TopLoc_Location

# from OCC.Core.TopExp import topexp_MapShapes
# from OCC.Core.TopTools import TopTools_MapOfShape, TopTools_IndexedMapOfShape
from OCC.Core.TopoDS import TopoDS_Shape, topods_Face
from OCC.Core.XCAFApp import XCAFApp_Application_GetApplication
from OCC.Core.XCAFDoc import XCAFDoc_DocumentTool_ColorTool, XCAFDoc_DocumentTool_ShapeTool
from OCC.Core.XSControl import XSControl_WorkSession

# from . import OCC
# print("[STEP import] OCC version:", OCC.VERSION)


def b_XYZ(v):
    x = v.XYZ()
    return (x.X(), x.Y(), x.Z())


def b_RGB(c):
    return (c.Red(), c.Green(), c.Blue())


def trsf_matrix(shp):
    trsf = shp.Location().Transformation()
    matrix = np.eye(4, dtype=np.float32)
    for row in range(1, 4):
        for col in range(1, 5):
            matrix[row - 1, col - 1] = trsf.Value(row, col)
    return matrix


def nurbs_parse(current_face):
    """
    Get NURBS points for a TopAbs_FACE
    Modified originally from PyGeM: https://github.com/mathLab/PyGeM MIT license

    return: nurb_points[u][v][x, y, z: float]
    """

    # performing some conversions to get the right format (BSplineSurface)
    nurbs_converter = BRepBuilderAPI_NurbsConvert(current_face)
    nurbs_converter.Perform(current_face)
    brep_face = BRep_Tool.Surface(topods_Face(nurbs_converter.Shape()))
    bspline_face = geomconvert_SurfaceToBSplineSurface(brep_face)

    # openCascade object
    occ_face = bspline_face.GetObject()

    # extract the Control Points of each face
    n_poles_u = occ_face.NbUPoles()
    n_poles_v = occ_face.NbVPoles()

    # cycle over the poles to get their coordinates
    points = []
    for pole_u_direction in range(n_poles_u):
        points.append([])
        for pole_v_direction in range(n_poles_v):
            coords = occ_face.Pole(pole_u_direction + 1, pole_v_direction + 1)
            points[-1].append([coords.X(), coords.Y(), coords.Z()])

    return points


# def get_shape_matrix(shp):
#     mtx = trsf_matrix(shp)
#     for row in range(mtx.shape[0]):
#         for col in range(mtx.shape[1]):
#             obj.matrix_world[row][col] = mtx[row][col]


# def make_face_hash(f):
#     items = []
#     # unfortunately, HashCode seems to separate colored and uncolored shapes
#     # items.append(f.HashCode(2**30))
#     items.append(f.Location().HashCode(2 ** 30))
#     return tuple(items)


def force_ascii(i_file):
    from pathlib import Path

    print("Attempting to format STEP file as ASCII 7-bit")
    p = Path(i_file)
    print(p.stat().st_size // 1024, "kB")
    import tempfile

    with tempfile.NamedTemporaryFile("w", encoding="ASCII") as fo:
        temp_name = fo.name
        print(temp_name)
        with p.open("rb") as f:
            while il := f.readline():
                fo.write(il.decode("ASCII"))
    print("done ASCII conversion.")
    return temp_name


# TODO: proper parametrization
def equalize_2d_points(pts):
    """Equalize aspect ratio of 2D point dimensions"""
    x_a, x_b = 1.0, 0.0
    y_a, y_b = 1.0, 0.0

    for i, uv in enumerate(pts):
        if uv[0] < x_a:
            x_a = uv[0]
        if uv[0] > x_b:
            x_b = uv[0]
        if uv[1] < y_a:
            y_a = uv[1]
        if uv[1] > y_b:
            y_b = uv[1]

    rx = abs(x_b - x_a)
    ry = abs(y_b - y_a)
    if rx != 0.0 and ry != 0.0:
        ratio = rx / ry
    else:
        ratio = 1.0

    ratio1 = 1 / ratio
    for i, uv in enumerate(pts):
        pts[i] = (pts[i][0] * ratio1, pts[i][1])

    return pts


@dataclass
class ShapeTreeNode:
    """
    A node for the OpenCASCADE CAD data ShapeTree
    """

    parent: int
    index: int
    tag: int
    name: str
    children: list[int] = field(default_factory=list)
    local_transform: np.ndarray = np.eye(4, dtype=np.float32)
    global_transform: np.ndarray = np.eye(4, dtype=np.float32)
    shape: TopoDS_Shape = None

    def get_values(self):
        """
        parent, index, tag, name
        """
        return (
            self.parent,
            self.index,
            self.tag,
            self.name,
            self.shape,
            self.local_transform,
            self.global_transform,
        )

    def set_shape(self, shape):
        if shape:
            if not isinstance(shape, TopoDS_Shape):
                raise ValueError("Input shape is not OpenCASCADE TopoDS_Shape")
            self.shape = shape
        else:
            self.shape = None


class ShapeTree:
    """
    Intermediary data structure to partially abstract OpenCASCADE away from the rest of the program
    """

    def __init__(self):
        self.nodes = []

        # Root node has special values
        self.nodes.append(ShapeTreeNode(-1, 0, -1, "root"))

    def get_root_id(self):
        return 0

    def get_max_id(self):
        return len(self.nodes) - 1

    def add(self, parent, label) -> ShapeTreeNode:
        loc = len(self.nodes)
        node = ShapeTreeNode(parent, loc, label.Tag(), label.GetLabelName())
        self.nodes[parent].children.append(loc)
        self.nodes.append(node)
        return self.nodes[-1]

    def get_shapes(self):
        # return {i.shape: i.index for i in self.nodes if i.shape}
        return [(i.shape, i.index) for i in self.nodes]

    def print_transforms(self):
        for i in self.nodes:
            print(i.local_transform)


class ReadSTEP:
    def __init__(self, filename):
        # from . import stepanalyzer
        # SA = stepanalyzer.StepAnalyzer(filename=filename)
        # print(SA.dump())

        self.read_file(filename)

    def query_color(self, shape, label, overwrite=False):
        # default color = pink
        color_tool = self.color_tool
        c = Quantity_Color(1.0, 0.0, 1.0, Quantity_TOC_RGB)
        colorSet = False
        # if (
        #     color_tool.GetInstanceColor(shape, 0, c)
        #     or color_tool.GetInstanceColor(shape, 1, c)
        #     or color_tool.GetInstanceColor(shape, 2, c)
        # ):
        #     if overwrite:
        #         color_tool.SetInstanceColor(shape, 0, c)
        #         color_tool.SetInstanceColor(shape, 1, c)
        #         color_tool.SetInstanceColor(shape, 2, c)
        #     colorSet = True

        if not colorSet:
            if (
                color_tool.GetColor(label, 0, c)
                or color_tool.GetColor(label, 1, c)
                or color_tool.GetColor(label, 2, c)
            ):
                if overwrite:
                    color_tool.SetInstanceColor(shape, 0, c)
                    color_tool.SetInstanceColor(shape, 1, c)
                    color_tool.SetInstanceColor(shape, 2, c)
                colorSet = True
        return c, colorSet, None

    def label_matrix(self, lab):
        trsf = self.shape_tool.GetLocation(lab).Transformation()
        matrix = np.eye(4, dtype=np.float32)
        for row in range(1, 4):
            for col in range(1, 5):
                matrix[row - 1, col - 1] = trsf.Value(row, col)
        # print(matrix)
        return matrix

    def explore_partial(self, shp, te_type):
        c_set = set([])
        ex = TopExp_Explorer(shp, te_type)
        # Todo: use label->tag
        while ex.More():
            c = ex.Current()
            if c not in c_set:
                c_set.add(c)
            ex.Next()
        return len(c_set)

    def explore_shape(self, shp):
        def _ct(ta_type):
            return self.explore_partial(shp, ta_type)

        return (
            _ct(TopAbs_COMPOUND),
            _ct(TopAbs_SOLID),
            _ct(TopAbs_SHELL),
            _ct(TopAbs_FACE),
            _ct(TopAbs_WIRE),
            _ct(TopAbs_EDGE),
            _ct(TopAbs_VERTEX),
        )

    def shape_info(self, shp):
        st = self.shape_tool
        lab = self.shape_label[shp]
        vals = (
            st.IsAssembly(lab),
            st.IsFree(lab),
            st.IsShape(lab),
            st.IsCompound(lab),
            st.IsComponent(lab),
            st.IsSimpleShape(lab),
            shp.Locked(),
        )

        lookup = ["A", "F", "S", "C", "T", "s", "L"]
        res = "".join([lookup[i] for i, v in enumerate(vals) if v])

        # res += f", C:{shp.NbChildren()}"

        res += ", C:{} So:{} Sh:{} F:{} Wi:{} E:{} V:{}".format(*self.explore_shape(shp))

        return " " + res + " "
        # return f"As:{} F:{} S:{} "
        # f"Cd:{} Ct:{} Si:{}"

    def transfer_with_units(self, filename):

        # Init new doc and reader
        doc = TDocStd_Document(TCollection_ExtendedString("STEP"))
        step_reader = STEPCAFControl_Reader()
        step_reader.SetColorMode(True)
        step_reader.SetNameMode(True)
        step_reader.SetMatMode(True)
        step_reader.SetLayerMode(True)

        # Read simple STEP file for correct units
        session = XSControl_WorkSession()
        step_simple_reader = STEPControl_Reader(session)

        print("DataExchange: Reading STEP")

        status = step_simple_reader.ReadFile(filename)
        if status != IFSelect_RetDone:
            raise AssertionError("Error: can't read file.")

        print("STEP read into memory")

        # read units
        ulen_names = TColStd_SequenceOfAsciiString()
        uang_names = TColStd_SequenceOfAsciiString()
        usld_names = TColStd_SequenceOfAsciiString()
        step_simple_reader.FileUnits(ulen_names, uang_names, usld_names)

        # for i in range(ulen_names.Length()):
        #     ulen = ulen_names.Value(i + 1)
        #     uang = uang_names.Value(i + 1)
        #     usld = usld_names.Value(i + 1)
        #     print(ulen.ToCString(), uang.ToCString(), usld.ToCString())

        # default is MM
        scale = 0.001

        if ulen_names.Length() > 0:
            scaleval = ulen_names.Value(1).ToCString().lower()

            # INCH, MM, FT, MI, M, KM, MIL, CM
            # UM, UIN ??

            scales = {
                "millimeter": 0.001,
                "millimetre": 0.001,
                "centimeter": 0.01,
                "centimetre": 0.01,
                "kilometer": 1000.0,
                "kilometre": 1000.0,
                "meter": 1.0,
                "metre": 1.0,
                "inch": 0.0254,
                "foot": 0.3048,
                "mile": 1609.34,
                "mil": 0.0254 * 0.001,
            }

            if scaleval in scales:
                scale = scales[scaleval]
            else:
                print("ERROR: Undefined scale:", scaleval)

            print("Scale from file (meters per unit):", scaleval, scale)

        else:
            print("Using default scale (millimeters)")

        self.scale = scale

        status = step_reader.ReadFile(self.filename)
        assert status == IFSelect_RetDone

        print("DataExchange: Transferring")
        # print("Roots:", step_reader.NbRootsForTransfer())
        transfer_result = step_reader.Transfer(doc)
        if not transfer_result:
            print("Dataexchange transfer FAILED.")
        else:
            print("DataExchange: Transfer done")

        return doc

    def transfer_simple(self, fname):
        # see stepanalyzer.py for license details

        # Create the application, empty document and shape_tool
        doc = TDocStd_Document(TCollection_ExtendedString("STEP"))
        app = XCAFApp_Application_GetApplication()
        app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc)

        # Read file and return populated doc
        step_reader = STEPCAFControl_Reader()
        step_reader.SetColorMode(True)
        step_reader.SetLayerMode(True)
        step_reader.SetNameMode(True)
        step_reader.SetMatMode(True)
        status = step_reader.ReadFile(fname)
        if status == IFSelect_RetDone:
            step_reader.Transfer(doc)
        self.scale = 0.001

        return doc

    def init_reader(self, filename):
        if not os.path.isfile(filename):
            raise FileNotFoundError("%s not found." % filename)

        # self.filename = force_ascii(filename)
        self.filename = filename

        doc = self.transfer_with_units(self.filename)

        self.shape_tool = XCAFDoc_DocumentTool_ShapeTool(doc.Main())
        self.shape_tool.SetAutoNaming(True)
        self.color_tool = XCAFDoc_DocumentTool_ColorTool(doc.Main())

        # material_tool = XCAFDoc_DocumentTool_MaterialTool(doc.Main())
        # layer_tool = XCAFDoc_DocumentTool_LayerTool(doc.Main())

        self.shape_label = {}
        self.face_colors = {}
        self.sub_shapes = {}
        self.tag_info = {}
        self.skipped_shapes = set([])
        self.import_problems = {"Triangulation": 0, "Undefined normals": 0, "Empty shape": 0}

    def read_file(self, filename):
        """Returns list of tuples (topods_shape, label, color)
        Use OCAF.
        """

        self.init_reader(filename)

        # output_shapes = {}
        # outliers = defaultdict(set)

        def _get_sub_shapes(lab, level, tree, leaf_id):
            # print(" " * (2 * level) + lab.GetLabelName())
            master_leaf = tree.nodes[leaf_id]
            # l_comps = TDF_LabelSequence()
            # self.shape_tool.GetComponents(lab, l_comps)
            if self.shape_tool.IsAssembly(lab):
                # Get transform for pure transform (empty)
                # Empty has eye transform, inherit global from parent

                # empty = tree.add(leaf.index, lab, empty=True)
                # output_shapes[shape] = empty

                # Read contained shapes
                l_c = TDF_LabelSequence()
                self.shape_tool.GetComponents(lab, l_c)
                for i in range(l_c.Length()):
                    label = l_c.Value(i + 1)
                    if self.shape_tool.IsReference(label):
                        label_reference = TDF_Label()
                        self.shape_tool.GetReferredShape(label, label_reference)

                        label_transform = self.label_matrix(label)
                        node = tree.add(master_leaf.index, label_reference)
                        new_leaf = tree.nodes[node.index]
                        new_leaf.local_transform = label_transform
                        new_leaf.global_transform = master_leaf.global_transform @ label_transform

                        _get_sub_shapes(label_reference, level + 1, tree, node.index)
                    else:
                        # TODO: process rest of the data
                        pass

            elif self.shape_tool.IsSimpleShape(lab):
                # TODO: self.shape_label stops being unique when shapes aren't transformed
                shape = self.shape_tool.GetShape(lab)
                self.shape_label[shape] = lab
                master_leaf.set_shape(shape)

                c, ok, name = self.query_color(shape, lab)
                self.face_colors[shape] = c if ok else None

                l_subss = TDF_LabelSequence()
                self.shape_tool.GetSubShapes(lab, l_subss)
                self.sub_shapes[shape] = []
                for i in range(l_subss.Length()):
                    lab_subs = l_subss.Value(i + 1)
                    shape_sub = self.shape_tool.GetShape(lab_subs)
                    self.shape_label[shape_sub] = lab_subs
                    self.sub_shapes[shape].append(shape_sub)

                    tc, ok, name = self.query_color(shape_sub, lab_subs)
                    self.face_colors[shape_sub] = tc if ok else None

                    # ftype_n = shape_sub.ShapeType()
                    # ftype = topabs_ShapeTypeToString(ftype_n)
                    # if ftype != "FACE":
                    #     outliers[lab.GetLabelName()].add(ftype + "=" + repr(ftype_n))
            else:
                print("DataExchange error: Item is neither assembly or a simple shape")

        def _get_shapes():
            # self.shape_tool.UpdateAssemblies()

            labels = TDF_LabelSequence()
            self.shape_tool.GetFreeShapes(labels)
            print("Roots:", labels.Length())

            # TODO: crashes with 7.5.0rc and 7.5.1
            # base_label = TDF_Label()
            # self.shape_tool.BaseLabel()
            # print(base_label.Tag())

            # labels = TDF_LabelSequence()
            # self.shape_tool.GetShapes(labels)
            # self.rootlabel = labels.Value(1)

            # l_c = TDF_LabelSequence()
            # self.shape_tool.GetComponents(base_label, l_c)
            # print("Base:", l_c.Length())

            tree = ShapeTree()
            for i in range(labels.Length()):
                print("DataExchange: Reading shape ({}/{})".format(i + 1, labels.Length()))

                root_item = labels.Value(i + 1)
                node = tree.add(tree.get_root_id(), root_item)
                _get_sub_shapes(root_item, 0, tree, node.index)

            return tree

        tree = _get_shapes()

        # print("Outliers:", outliers)

        # import json
        # print(json.dumps(tree, indent=2))
        # self.output_shapes = output_shapes
        self.tree = tree

    def triangulate_face(self, face, tform):
        bt = BRep_Tool()
        location = TopLoc_Location()
        facing = bt.Triangulation(face, location)
        if facing is None:
            # Mesh error, no triangulation found for part
            self.import_problems["Triangulation"] += 1
            return None, None

        normcalc = poly()
        normcalc.ComputeNormals(facing)

        # nsurf = bt.Surface(face)
        surface = BRepAdaptor_Surface(face)
        prop = BRepLProp_SLProps(surface, 2, gp.Resolution())
        # prop = BRepLProp_SLProps(surface, 2, 1e-4)
        face_uv = facing.UVNodes()

        # Calculate UV bounds
        Umin, Umax, Vmin, Vmax = 0.0, 0.0, 0.0, 0.0
        for t in range(1, face_uv.Length()):
            v = face_uv.Value(t)
            x, y = v.X(), v.Y()
            if t == 1:
                Umin, Umax, Vmin, Vmax = x, x, y, y
            if x < Umin:
                Umin = x
            if x > Umax:
                Umin = x
            if y < Vmin:
                Vmin = y
            if y > Vmax:
                Vmin = y
        Ucenter = (Umin + Umax) * 0.5
        Vcenter = (Vmin + Vmax) * 0.5

        tab = facing.Nodes()
        tri = facing.Triangles()

        verts = []
        norms = []
        tris = []
        uvs = []

        undef_normals = False
        new_points = 0
        messages = {}

        itform = tform.Inverted()

        # Build normals
        d_nbnodes = facing.NbNodes()
        assert d_nbnodes < 1000000000
        for t in range(1, d_nbnodes + 1):
            pt = tab.Value(t)
            loc = b_XYZ(pt)

            # nvert = bm.verts.new(loc)
            # nvert.index = t - 1

            assert len(loc) == 3
            assert type(loc[0]) == float
            assert type(loc) == tuple
            verts.append(loc)

            # Get triangulation normal

            # pt = gp_Pnt(loc[0], loc[1], loc[2])
            # pt_surf = GeomAPI_ProjectPointOnSurf(pt, nsurf)
            # fU, fV = pt_surf.Parameters(1)
            # prop = GeomLProp_SLProps(nsurf, fU, fV, 2, gp.Resolution())

            uv = face_uv.Value(t)
            u, v = uv.X(), uv.Y()
            uvs.append((u, v))

            # The edges of UV give invalid normals, hence this
            prop.SetParameters((u - Ucenter) * 0.999 + Ucenter, (v - Vcenter) * 0.999 + Vcenter)

            if prop.IsNormalDefined():
                normal = prop.Normal().Transformed(itform)
                # normal = prop.Normal()
                nn = np.array(b_XYZ(normal))
                if face.Orientation() == TopAbs_REVERSED:
                    nn = -nn
            else:
                nn = np.array((0.0, 0.0, 1.0))
                undef_normals = True

            # norms.append(tuple(float(nnn) for nnn in nn))
            norms.append(np.float32(nn))

        # Build triangulation
        d_nbtriangles = facing.NbTriangles()
        assert d_nbtriangles < 1000000000
        for t in range(1, d_nbtriangles + 1):
            T1, T2, T3 = tri.Value(t).Get()

            if face.Orientation() != TopAbs_FORWARD:
                T1, T2 = T2, T1

            # v_list = (verts[T1 - 1], verts[T2 - 1], verts[T3 - 1])
            # nf = bm.faces.new(v_list)
            # nf.smooth = True
            # nf.normal_update()
            tris.append((T1 - 1, T2 - 1, T3 - 1))

            # for v in (T1, T2, T3):
            #     if norms[v - 1] is None:
            #         added_norms.append(np.array(nf.normal))
            #     else:
            #         added_norms.append(norms[v - 1])

            # new_norms.append(norms[v - 1])

        messages["Triangles"] = facing.NbTriangles()
        messages["Points"] = new_points

        if undef_normals:
            self.import_problems["Undefined normals"] += 1

        tri_data = []
        for ti, t in enumerate(tris):
            tri_data.append(
                trimesh.TriData(
                    t, [norms[i] for i in t], [uvs[i] for i in t], None, None, None, None
                )
            )

        return trimesh.TriMesh(verts=verts, tris=tri_data), messages

    def build_trimesh(self, shape, lin_def=0.8, ang_def=0.5, hacks=set([])):
        out_mesh = trimesh.TriMesh()
        out_mesh.matrix = np.eye(4, dtype=np.float32)

        # TODO: REMOVE HACK
        # if (
        #     "skip_solids" in hacks
        #     and self.explore_partial(shape, TopAbs_EDGE) > 14000
        #     and self.explore_partial(shape, TopAbs_SOLID) == 0
        # ):
        #     self.skipped_shapes.add(self.shape_label[shape].GetLabelName())
        #     return out_mesh

        if "skip_solids" in hacks and self.explore_partial(shape, TopAbs_SOLID) == 0:
            self.skipped_shapes.add(self.shape_label[shape].GetLabelName())
            return out_mesh

        iter_shapes = [shape] + self.sub_shapes[shape]
        # if len(self.sub_shapes[shape]) > 0:
        #     iter_shapes = [shape] + self.sub_shapes[shape][::-1]

        brt = breptools()
        for shp in iter_shapes:
            # Clean all previous triangulations
            brt.Clean(shp)

        # NOTE: Subshapes can have different colors from each other

        face_data = {}
        for shp_i, shp in enumerate(iter_shapes):
            # is_subshape = shp_i > 0

            # NOTE: must define colors at this level
            col = self.face_colors[shp]

            # Subshape transforms can be different from the mainshape transform
            ex = TopExp_Explorer(shp, TopAbs_FACE)
            if not ex.More():
                self.import_problems["Empty shape"] += 1
                continue

            # TODO: subshape order not known (check for shell/solid/compound)

            # imsp = IMeshTools_Parameters()
            # imsp.Angle = ang_def
            # imsp.AngleInterior = ang_def
            # imsp.Deflection = lin_def
            # imsp.DeflectionInterior = lin_def

            # imsp.ForceFaceDeflection = False
            # imsp.InParallel = True

            # imsp.CleanModel = True
            # imsp.AllowQualityDecrease = True
            # imsp.AdjustMinSize = True
            # imsp.MinSize = lin_def * 2.0

            # imsp.ControlSurfaceDeflection = True
            # imsp.Relative = False
            # imsp.InternalVerticesMode = True

            # # (hack) FREEZE here
            # brepmesh = BRepMesh_IncrementalMesh(shape, imsp)

            brepmesh = BRepMesh_IncrementalMesh(shp, lin_def, False, ang_def, False)
            brepmesh.Perform()
            trf = shp.Location().Transformation()
            # Iterate through faces with TopExp_Explorer
            while ex.More():
                exc = ex.Current()
                face = topods_Face(exc)
                face_hash = face
                # face_hash = exc.TShape().HashCode(2**30)
                # face_hash = self.shape_label[shape].GetLabelName()
                # face_hash = make_face_hash(exc)

                mesh, message = self.triangulate_face(face, trf)

                if mesh:
                    # If shape or sub-shape has defined color, set it so
                    if col is not None:
                        mesh.colorize(b_RGB(col))
                        mesh.set_material_name(Quantity_Color.StringName(Quantity_Color.Name(col)))
                    face_data[face_hash] = mesh

                ex.Next()

        # TODO: this just overwrites every face with the last one that was read
        # TODO: instead, overwrite only if it has color
        # partial implementation in TriMesh.add_tri_overwrite
        for k, mesh in face_data.items():
            # key=topods_shape, value=TriMesh
            if len(mesh.verts) > 0:
                out_mesh.add_mesh_overwrite_identical(mesh)
                # out_mesh.add_mesh(mesh)

        # ex.Clear()
        # ex.Destroy()
        # del ex

        return out_mesh

    def build_nurbs(self, shape, lin_def=0.8, ang_def=0.5):
        iter_shapes = [shape]
        points = []
        for shp_i, shp in enumerate(iter_shapes):
            ex = TopExp_Explorer(shp, TopAbs_FACE)
            if not ex.More():
                self.import_problems["Empty shape"] += 1
                return []

            trf = shp.Location().Transformation()
            while ex.More():
                pt = nurbs_parse(topods_Face(ex.Current()))
                points.append(pt)
                ex.Next()

        print(points)
        return points
 