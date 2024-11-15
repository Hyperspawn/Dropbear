# This file was automatically generated by SWIG (http://www.swig.org).
# Version 4.0.1
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

"""
StdPrs module, see official documentation at
https://www.opencascade.com/doc/occt-7.4.0/refman/html/package_stdprs.html
"""

from sys import version_info as _swig_python_version_info
if _swig_python_version_info < (2, 7, 0):
    raise RuntimeError("Python 2.7 or later required")

# Import the low-level C/C++ module
if __package__ or "." in __name__:
    from . import _StdPrs
else:
    import _StdPrs

try:
    import builtins as __builtin__
except ImportError:
    import __builtin__

_swig_new_instance_method = _StdPrs.SWIG_PyInstanceMethod_New
_swig_new_static_method = _StdPrs.SWIG_PyStaticMethod_New

def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except __builtin__.Exception:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)


def _swig_setattr_nondynamic_instance_variable(set):
    def set_instance_attr(self, name, value):
        if name == "thisown":
            self.this.own(value)
        elif name == "this":
            set(self, name, value)
        elif hasattr(self, name) and isinstance(getattr(type(self), name), property):
            set(self, name, value)
        else:
            raise AttributeError("You cannot add instance attributes to %s" % self)
    return set_instance_attr


def _swig_setattr_nondynamic_class_variable(set):
    def set_class_attr(cls, name, value):
        if hasattr(cls, name) and not isinstance(getattr(cls, name), property):
            set(cls, name, value)
        else:
            raise AttributeError("You cannot add class attributes to %s" % cls)
    return set_class_attr


def _swig_add_metaclass(metaclass):
    """Class decorator for adding a metaclass to a SWIG wrapped class - a slimmed down version of six.add_metaclass"""
    def wrapper(cls):
        return metaclass(cls.__name__, cls.__bases__, cls.__dict__.copy())
    return wrapper


class _SwigNonDynamicMeta(type):
    """Meta class to enforce nondynamic attributes (no new attributes) for a class"""
    __setattr__ = _swig_setattr_nondynamic_class_variable(type.__setattr__)


class SwigPyIterator(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined - class is abstract")
    __repr__ = _swig_repr
    __swig_destroy__ = _StdPrs.delete_SwigPyIterator
    value = _swig_new_instance_method(_StdPrs.SwigPyIterator_value)
    incr = _swig_new_instance_method(_StdPrs.SwigPyIterator_incr)
    decr = _swig_new_instance_method(_StdPrs.SwigPyIterator_decr)
    distance = _swig_new_instance_method(_StdPrs.SwigPyIterator_distance)
    equal = _swig_new_instance_method(_StdPrs.SwigPyIterator_equal)
    copy = _swig_new_instance_method(_StdPrs.SwigPyIterator_copy)
    next = _swig_new_instance_method(_StdPrs.SwigPyIterator_next)
    __next__ = _swig_new_instance_method(_StdPrs.SwigPyIterator___next__)
    previous = _swig_new_instance_method(_StdPrs.SwigPyIterator_previous)
    advance = _swig_new_instance_method(_StdPrs.SwigPyIterator_advance)
    __eq__ = _swig_new_instance_method(_StdPrs.SwigPyIterator___eq__)
    __ne__ = _swig_new_instance_method(_StdPrs.SwigPyIterator___ne__)
    __iadd__ = _swig_new_instance_method(_StdPrs.SwigPyIterator___iadd__)
    __isub__ = _swig_new_instance_method(_StdPrs.SwigPyIterator___isub__)
    __add__ = _swig_new_instance_method(_StdPrs.SwigPyIterator___add__)
    __sub__ = _swig_new_instance_method(_StdPrs.SwigPyIterator___sub__)
    def __iter__(self):
        return self

# Register SwigPyIterator in _StdPrs:
_StdPrs.SwigPyIterator_swigregister(SwigPyIterator)


def _dumps_object(klass):
    """ Overwrite default string output for any wrapped object.
    By default, __repr__ method returns something like:
    <OCC.Core.TopoDS.TopoDS_Shape; proxy of <Swig Object of type 'TopoDS_Shape *' at 0x02BB0758> >
    This is too much verbose.
    We prefer :
    <class 'gp_Pnt'>
    or
    <class 'TopoDS_Shape'>
    """
    klass_name = str(klass.__class__).split(".")[3].split("'")[0]
    repr_string = "<class '" + klass_name + "'"
# for TopoDS_Shape, we also look for the base type
    if klass_name == "TopoDS_Shape":
        if klass.IsNull():
            repr_string += ": Null>"
            return repr_string
        st = klass.ShapeType()
        types = {OCC.Core.TopAbs.TopAbs_VERTEX: "Vertex",
                 OCC.Core.TopAbs.TopAbs_SOLID: "Solid",
                 OCC.Core.TopAbs.TopAbs_EDGE: "Edge",
                 OCC.Core.TopAbs.TopAbs_FACE: "Face",
                 OCC.Core.TopAbs.TopAbs_SHELL: "Shell",
                 OCC.Core.TopAbs.TopAbs_WIRE: "Wire",
                 OCC.Core.TopAbs.TopAbs_COMPOUND: "Compound",
                 OCC.Core.TopAbs.TopAbs_COMPSOLID: "Compsolid"}
        repr_string += "; Type:%s" % types[st]        
    elif hasattr(klass, "IsNull"):
        if klass.IsNull():
            repr_string += "; Null"
    repr_string += ">"
    return repr_string


from six import with_metaclass
import warnings
from OCC.Wrapper.wrapper_utils import Proxy, deprecated

import OCC.Core.Standard
import OCC.Core.NCollection
import OCC.Core.Geom
import OCC.Core.gp
import OCC.Core.GeomAbs
import OCC.Core.TColgp
import OCC.Core.TColStd
import OCC.Core.TCollection
import OCC.Core.Prs3d
import OCC.Core.Graphic3d
import OCC.Core.BVH
import OCC.Core.Quantity
import OCC.Core.Aspect
import OCC.Core.Bnd
import OCC.Core.Image
import OCC.Core.OSD
import OCC.Core.TopoDS
import OCC.Core.Message
import OCC.Core.TopAbs
import OCC.Core.TopLoc
import OCC.Core.HLRAlgo
import OCC.Core.Poly
import OCC.Core.TShort
import OCC.Core.TopTools
import OCC.Core.Adaptor3d
import OCC.Core.Adaptor2d
import OCC.Core.Geom2d
import OCC.Core.math
import OCC.Core.BRepAdaptor
import OCC.Core.GeomAdaptor
import OCC.Core.Geom2dAdaptor
import OCC.Core.BRep
StdPrs_Volume_Autodetection = _StdPrs.StdPrs_Volume_Autodetection
StdPrs_Volume_Closed = _StdPrs.StdPrs_Volume_Closed
StdPrs_Volume_Opened = _StdPrs.StdPrs_Volume_Opened
class StdPrs_Point(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    Add = _swig_new_static_method(_StdPrs.StdPrs_Point_Add)
    Match = _swig_new_static_method(_StdPrs.StdPrs_Point_Match)

    def __init__(self):
        _StdPrs.StdPrs_Point_swiginit(self, _StdPrs.new_StdPrs_Point())
    __swig_destroy__ = _StdPrs.delete_StdPrs_Point

# Register StdPrs_Point in _StdPrs:
_StdPrs.StdPrs_Point_swigregister(StdPrs_Point)
StdPrs_Point_Add = _StdPrs.StdPrs_Point_Add
StdPrs_Point_Match = _StdPrs.StdPrs_Point_Match

class StdPrs_Vertex(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    Add = _swig_new_static_method(_StdPrs.StdPrs_Vertex_Add)
    Match = _swig_new_static_method(_StdPrs.StdPrs_Vertex_Match)

    def __init__(self):
        _StdPrs.StdPrs_Vertex_swiginit(self, _StdPrs.new_StdPrs_Vertex())
    __swig_destroy__ = _StdPrs.delete_StdPrs_Vertex

# Register StdPrs_Vertex in _StdPrs:
_StdPrs.StdPrs_Vertex_swigregister(StdPrs_Vertex)
StdPrs_Vertex_Add = _StdPrs.StdPrs_Vertex_Add
StdPrs_Vertex_Match = _StdPrs.StdPrs_Vertex_Match

class StdPrs_BndBox(OCC.Core.Prs3d.Prs3d_Root):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined")
    __repr__ = _swig_repr
    Add = _swig_new_static_method(_StdPrs.StdPrs_BndBox_Add)
    FillSegments = _swig_new_static_method(_StdPrs.StdPrs_BndBox_FillSegments)
    fillSegments = _swig_new_static_method(_StdPrs.StdPrs_BndBox_fillSegments)

    __repr__ = _dumps_object

    __swig_destroy__ = _StdPrs.delete_StdPrs_BndBox

# Register StdPrs_BndBox in _StdPrs:
_StdPrs.StdPrs_BndBox_swigregister(StdPrs_BndBox)
StdPrs_BndBox_Add = _StdPrs.StdPrs_BndBox_Add
StdPrs_BndBox_FillSegments = _StdPrs.StdPrs_BndBox_FillSegments
StdPrs_BndBox_fillSegments = _StdPrs.StdPrs_BndBox_fillSegments

class StdPrs_Curve(OCC.Core.Prs3d.Prs3d_Root):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    Add = _swig_new_static_method(_StdPrs.StdPrs_Curve_Add)
    Match = _swig_new_static_method(_StdPrs.StdPrs_Curve_Match)

    __repr__ = _dumps_object


    def __init__(self):
        _StdPrs.StdPrs_Curve_swiginit(self, _StdPrs.new_StdPrs_Curve())
    __swig_destroy__ = _StdPrs.delete_StdPrs_Curve

# Register StdPrs_Curve in _StdPrs:
_StdPrs.StdPrs_Curve_swigregister(StdPrs_Curve)
StdPrs_Curve_Add = _StdPrs.StdPrs_Curve_Add
StdPrs_Curve_Match = _StdPrs.StdPrs_Curve_Match

class StdPrs_HLRPolyShape(OCC.Core.Prs3d.Prs3d_Root):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    Add = _swig_new_static_method(_StdPrs.StdPrs_HLRPolyShape_Add)

    __repr__ = _dumps_object


    def __init__(self):
        _StdPrs.StdPrs_HLRPolyShape_swiginit(self, _StdPrs.new_StdPrs_HLRPolyShape())
    __swig_destroy__ = _StdPrs.delete_StdPrs_HLRPolyShape

# Register StdPrs_HLRPolyShape in _StdPrs:
_StdPrs.StdPrs_HLRPolyShape_swigregister(StdPrs_HLRPolyShape)
StdPrs_HLRPolyShape_Add = _StdPrs.StdPrs_HLRPolyShape_Add

class StdPrs_HLRShape(OCC.Core.Prs3d.Prs3d_Root):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    Add = _swig_new_static_method(_StdPrs.StdPrs_HLRShape_Add)

    __repr__ = _dumps_object


    def __init__(self):
        _StdPrs.StdPrs_HLRShape_swiginit(self, _StdPrs.new_StdPrs_HLRShape())
    __swig_destroy__ = _StdPrs.delete_StdPrs_HLRShape

# Register StdPrs_HLRShape in _StdPrs:
_StdPrs.StdPrs_HLRShape_swigregister(StdPrs_HLRShape)
StdPrs_HLRShape_Add = _StdPrs.StdPrs_HLRShape_Add

class StdPrs_HLRToolShape(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    Hidden = _swig_new_instance_method(_StdPrs.StdPrs_HLRToolShape_Hidden)
    InitHidden = _swig_new_instance_method(_StdPrs.StdPrs_HLRToolShape_InitHidden)
    InitVisible = _swig_new_instance_method(_StdPrs.StdPrs_HLRToolShape_InitVisible)
    MoreHidden = _swig_new_instance_method(_StdPrs.StdPrs_HLRToolShape_MoreHidden)
    MoreVisible = _swig_new_instance_method(_StdPrs.StdPrs_HLRToolShape_MoreVisible)
    NbEdges = _swig_new_instance_method(_StdPrs.StdPrs_HLRToolShape_NbEdges)
    NextHidden = _swig_new_instance_method(_StdPrs.StdPrs_HLRToolShape_NextHidden)
    NextVisible = _swig_new_instance_method(_StdPrs.StdPrs_HLRToolShape_NextVisible)

    def __init__(self, *args):
        r"""
        :param TheShape:
        	:type TheShape: TopoDS_Shape
        	:param TheProjector:
        	:type TheProjector: HLRAlgo_Projector
        	:rtype: None
        """
        _StdPrs.StdPrs_HLRToolShape_swiginit(self, _StdPrs.new_StdPrs_HLRToolShape(*args))
    Visible = _swig_new_instance_method(_StdPrs.StdPrs_HLRToolShape_Visible)

    __repr__ = _dumps_object

    __swig_destroy__ = _StdPrs.delete_StdPrs_HLRToolShape

# Register StdPrs_HLRToolShape in _StdPrs:
_StdPrs.StdPrs_HLRToolShape_swigregister(StdPrs_HLRToolShape)

class StdPrs_Isolines(OCC.Core.Prs3d.Prs3d_Root):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined")
    __repr__ = _swig_repr
    Add = _swig_new_static_method(_StdPrs.StdPrs_Isolines_Add)
    AddOnSurface = _swig_new_static_method(_StdPrs.StdPrs_Isolines_AddOnSurface)
    AddOnTriangulation = _swig_new_static_method(_StdPrs.StdPrs_Isolines_AddOnTriangulation)
    UVIsoParameters = _swig_new_static_method(_StdPrs.StdPrs_Isolines_UVIsoParameters)

    __repr__ = _dumps_object

    __swig_destroy__ = _StdPrs.delete_StdPrs_Isolines

# Register StdPrs_Isolines in _StdPrs:
_StdPrs.StdPrs_Isolines_swigregister(StdPrs_Isolines)
StdPrs_Isolines_Add = _StdPrs.StdPrs_Isolines_Add
StdPrs_Isolines_AddOnSurface = _StdPrs.StdPrs_Isolines_AddOnSurface
StdPrs_Isolines_AddOnTriangulation = _StdPrs.StdPrs_Isolines_AddOnTriangulation
StdPrs_Isolines_UVIsoParameters = _StdPrs.StdPrs_Isolines_UVIsoParameters

class StdPrs_Plane(OCC.Core.Prs3d.Prs3d_Root):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    Add = _swig_new_static_method(_StdPrs.StdPrs_Plane_Add)
    Match = _swig_new_static_method(_StdPrs.StdPrs_Plane_Match)

    __repr__ = _dumps_object


    def __init__(self):
        _StdPrs.StdPrs_Plane_swiginit(self, _StdPrs.new_StdPrs_Plane())
    __swig_destroy__ = _StdPrs.delete_StdPrs_Plane

# Register StdPrs_Plane in _StdPrs:
_StdPrs.StdPrs_Plane_swigregister(StdPrs_Plane)
StdPrs_Plane_Add = _StdPrs.StdPrs_Plane_Add
StdPrs_Plane_Match = _StdPrs.StdPrs_Plane_Match

class StdPrs_PoleCurve(OCC.Core.Prs3d.Prs3d_Root):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    Add = _swig_new_static_method(_StdPrs.StdPrs_PoleCurve_Add)
    Match = _swig_new_static_method(_StdPrs.StdPrs_PoleCurve_Match)
    Pick = _swig_new_static_method(_StdPrs.StdPrs_PoleCurve_Pick)

    __repr__ = _dumps_object


    def __init__(self):
        _StdPrs.StdPrs_PoleCurve_swiginit(self, _StdPrs.new_StdPrs_PoleCurve())
    __swig_destroy__ = _StdPrs.delete_StdPrs_PoleCurve

# Register StdPrs_PoleCurve in _StdPrs:
_StdPrs.StdPrs_PoleCurve_swigregister(StdPrs_PoleCurve)
StdPrs_PoleCurve_Add = _StdPrs.StdPrs_PoleCurve_Add
StdPrs_PoleCurve_Match = _StdPrs.StdPrs_PoleCurve_Match
StdPrs_PoleCurve_Pick = _StdPrs.StdPrs_PoleCurve_Pick

class StdPrs_ShadedShape(OCC.Core.Prs3d.Prs3d_Root):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined")
    __repr__ = _swig_repr
    Add = _swig_new_static_method(_StdPrs.StdPrs_ShadedShape_Add)
    AddWireframeForFacesWithoutTriangles = _swig_new_static_method(_StdPrs.StdPrs_ShadedShape_AddWireframeForFacesWithoutTriangles)
    AddWireframeForFreeElements = _swig_new_static_method(_StdPrs.StdPrs_ShadedShape_AddWireframeForFreeElements)
    ExploreSolids = _swig_new_static_method(_StdPrs.StdPrs_ShadedShape_ExploreSolids)
    FillFaceBoundaries = _swig_new_static_method(_StdPrs.StdPrs_ShadedShape_FillFaceBoundaries)
    FillTriangles = _swig_new_static_method(_StdPrs.StdPrs_ShadedShape_FillTriangles)

    __repr__ = _dumps_object

    __swig_destroy__ = _StdPrs.delete_StdPrs_ShadedShape

# Register StdPrs_ShadedShape in _StdPrs:
_StdPrs.StdPrs_ShadedShape_swigregister(StdPrs_ShadedShape)
StdPrs_ShadedShape_Add = _StdPrs.StdPrs_ShadedShape_Add
StdPrs_ShadedShape_AddWireframeForFacesWithoutTriangles = _StdPrs.StdPrs_ShadedShape_AddWireframeForFacesWithoutTriangles
StdPrs_ShadedShape_AddWireframeForFreeElements = _StdPrs.StdPrs_ShadedShape_AddWireframeForFreeElements
StdPrs_ShadedShape_ExploreSolids = _StdPrs.StdPrs_ShadedShape_ExploreSolids
StdPrs_ShadedShape_FillFaceBoundaries = _StdPrs.StdPrs_ShadedShape_FillFaceBoundaries
StdPrs_ShadedShape_FillTriangles = _StdPrs.StdPrs_ShadedShape_FillTriangles

class StdPrs_ShadedSurface(OCC.Core.Prs3d.Prs3d_Root):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    Add = _swig_new_static_method(_StdPrs.StdPrs_ShadedSurface_Add)

    __repr__ = _dumps_object


    def __init__(self):
        _StdPrs.StdPrs_ShadedSurface_swiginit(self, _StdPrs.new_StdPrs_ShadedSurface())
    __swig_destroy__ = _StdPrs.delete_StdPrs_ShadedSurface

# Register StdPrs_ShadedSurface in _StdPrs:
_StdPrs.StdPrs_ShadedSurface_swigregister(StdPrs_ShadedSurface)
StdPrs_ShadedSurface_Add = _StdPrs.StdPrs_ShadedSurface_Add

class StdPrs_ToolPoint(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    Coord = _swig_new_static_method(_StdPrs.StdPrs_ToolPoint_Coord)

    __repr__ = _dumps_object


    def __init__(self):
        _StdPrs.StdPrs_ToolPoint_swiginit(self, _StdPrs.new_StdPrs_ToolPoint())
    __swig_destroy__ = _StdPrs.delete_StdPrs_ToolPoint

# Register StdPrs_ToolPoint in _StdPrs:
_StdPrs.StdPrs_ToolPoint_swigregister(StdPrs_ToolPoint)
StdPrs_ToolPoint_Coord = _StdPrs.StdPrs_ToolPoint_Coord

class StdPrs_ToolRFace(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    Init = _swig_new_instance_method(_StdPrs.StdPrs_ToolRFace_Init)
    IsInvalidGeometry = _swig_new_instance_method(_StdPrs.StdPrs_ToolRFace_IsInvalidGeometry)
    IsOriented = _swig_new_instance_method(_StdPrs.StdPrs_ToolRFace_IsOriented)
    More = _swig_new_instance_method(_StdPrs.StdPrs_ToolRFace_More)
    Next = _swig_new_instance_method(_StdPrs.StdPrs_ToolRFace_Next)
    Orientation = _swig_new_instance_method(_StdPrs.StdPrs_ToolRFace_Orientation)

    def __init__(self, *args):
        r"""
        * Empty constructor.
        	:rtype: None* Constructor with initialization.
        	:param aSurface:
        	:type aSurface: BRepAdaptor_HSurface
        	:rtype: None
        """
        _StdPrs.StdPrs_ToolRFace_swiginit(self, _StdPrs.new_StdPrs_ToolRFace(*args))
    Value = _swig_new_instance_method(_StdPrs.StdPrs_ToolRFace_Value)

    __repr__ = _dumps_object

    __swig_destroy__ = _StdPrs.delete_StdPrs_ToolRFace

# Register StdPrs_ToolRFace in _StdPrs:
_StdPrs.StdPrs_ToolRFace_swigregister(StdPrs_ToolRFace)

class StdPrs_ToolTriangulatedShape(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined")
    __repr__ = _swig_repr
    ClearOnOwnDeflectionChange = _swig_new_static_method(_StdPrs.StdPrs_ToolTriangulatedShape_ClearOnOwnDeflectionChange)
    ComputeNormals = _swig_new_static_method(_StdPrs.StdPrs_ToolTriangulatedShape_ComputeNormals)
    IsClosed = _swig_new_static_method(_StdPrs.StdPrs_ToolTriangulatedShape_IsClosed)
    IsTessellated = _swig_new_static_method(_StdPrs.StdPrs_ToolTriangulatedShape_IsTessellated)
    IsTriangulated = _swig_new_static_method(_StdPrs.StdPrs_ToolTriangulatedShape_IsTriangulated)
    Normal = _swig_new_static_method(_StdPrs.StdPrs_ToolTriangulatedShape_Normal)
    Tessellate = _swig_new_static_method(_StdPrs.StdPrs_ToolTriangulatedShape_Tessellate)

    __repr__ = _dumps_object

    __swig_destroy__ = _StdPrs.delete_StdPrs_ToolTriangulatedShape

# Register StdPrs_ToolTriangulatedShape in _StdPrs:
_StdPrs.StdPrs_ToolTriangulatedShape_swigregister(StdPrs_ToolTriangulatedShape)
StdPrs_ToolTriangulatedShape_ClearOnOwnDeflectionChange = _StdPrs.StdPrs_ToolTriangulatedShape_ClearOnOwnDeflectionChange
StdPrs_ToolTriangulatedShape_ComputeNormals = _StdPrs.StdPrs_ToolTriangulatedShape_ComputeNormals
StdPrs_ToolTriangulatedShape_IsClosed = _StdPrs.StdPrs_ToolTriangulatedShape_IsClosed
StdPrs_ToolTriangulatedShape_IsTessellated = _StdPrs.StdPrs_ToolTriangulatedShape_IsTessellated
StdPrs_ToolTriangulatedShape_IsTriangulated = _StdPrs.StdPrs_ToolTriangulatedShape_IsTriangulated
StdPrs_ToolTriangulatedShape_Normal = _StdPrs.StdPrs_ToolTriangulatedShape_Normal
StdPrs_ToolTriangulatedShape_Tessellate = _StdPrs.StdPrs_ToolTriangulatedShape_Tessellate

class StdPrs_ToolVertex(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    Coord = _swig_new_static_method(_StdPrs.StdPrs_ToolVertex_Coord)

    __repr__ = _dumps_object


    def __init__(self):
        _StdPrs.StdPrs_ToolVertex_swiginit(self, _StdPrs.new_StdPrs_ToolVertex())
    __swig_destroy__ = _StdPrs.delete_StdPrs_ToolVertex

# Register StdPrs_ToolVertex in _StdPrs:
_StdPrs.StdPrs_ToolVertex_swigregister(StdPrs_ToolVertex)
StdPrs_ToolVertex_Coord = _StdPrs.StdPrs_ToolVertex_Coord

class StdPrs_WFDeflectionRestrictedFace(OCC.Core.Prs3d.Prs3d_Root):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    Add = _swig_new_static_method(_StdPrs.StdPrs_WFDeflectionRestrictedFace_Add)
    AddUIso = _swig_new_static_method(_StdPrs.StdPrs_WFDeflectionRestrictedFace_AddUIso)
    AddVIso = _swig_new_static_method(_StdPrs.StdPrs_WFDeflectionRestrictedFace_AddVIso)
    Match = _swig_new_static_method(_StdPrs.StdPrs_WFDeflectionRestrictedFace_Match)
    MatchUIso = _swig_new_static_method(_StdPrs.StdPrs_WFDeflectionRestrictedFace_MatchUIso)
    MatchVIso = _swig_new_static_method(_StdPrs.StdPrs_WFDeflectionRestrictedFace_MatchVIso)

    __repr__ = _dumps_object


    def __init__(self):
        _StdPrs.StdPrs_WFDeflectionRestrictedFace_swiginit(self, _StdPrs.new_StdPrs_WFDeflectionRestrictedFace())
    __swig_destroy__ = _StdPrs.delete_StdPrs_WFDeflectionRestrictedFace

# Register StdPrs_WFDeflectionRestrictedFace in _StdPrs:
_StdPrs.StdPrs_WFDeflectionRestrictedFace_swigregister(StdPrs_WFDeflectionRestrictedFace)
StdPrs_WFDeflectionRestrictedFace_Add = _StdPrs.StdPrs_WFDeflectionRestrictedFace_Add
StdPrs_WFDeflectionRestrictedFace_AddUIso = _StdPrs.StdPrs_WFDeflectionRestrictedFace_AddUIso
StdPrs_WFDeflectionRestrictedFace_AddVIso = _StdPrs.StdPrs_WFDeflectionRestrictedFace_AddVIso
StdPrs_WFDeflectionRestrictedFace_Match = _StdPrs.StdPrs_WFDeflectionRestrictedFace_Match
StdPrs_WFDeflectionRestrictedFace_MatchUIso = _StdPrs.StdPrs_WFDeflectionRestrictedFace_MatchUIso
StdPrs_WFDeflectionRestrictedFace_MatchVIso = _StdPrs.StdPrs_WFDeflectionRestrictedFace_MatchVIso

class StdPrs_WFDeflectionSurface(OCC.Core.Prs3d.Prs3d_Root):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    Add = _swig_new_static_method(_StdPrs.StdPrs_WFDeflectionSurface_Add)

    __repr__ = _dumps_object


    def __init__(self):
        _StdPrs.StdPrs_WFDeflectionSurface_swiginit(self, _StdPrs.new_StdPrs_WFDeflectionSurface())
    __swig_destroy__ = _StdPrs.delete_StdPrs_WFDeflectionSurface

# Register StdPrs_WFDeflectionSurface in _StdPrs:
_StdPrs.StdPrs_WFDeflectionSurface_swigregister(StdPrs_WFDeflectionSurface)
StdPrs_WFDeflectionSurface_Add = _StdPrs.StdPrs_WFDeflectionSurface_Add

class StdPrs_WFPoleSurface(OCC.Core.Prs3d.Prs3d_Root):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    Add = _swig_new_static_method(_StdPrs.StdPrs_WFPoleSurface_Add)

    __repr__ = _dumps_object


    def __init__(self):
        _StdPrs.StdPrs_WFPoleSurface_swiginit(self, _StdPrs.new_StdPrs_WFPoleSurface())
    __swig_destroy__ = _StdPrs.delete_StdPrs_WFPoleSurface

# Register StdPrs_WFPoleSurface in _StdPrs:
_StdPrs.StdPrs_WFPoleSurface_swigregister(StdPrs_WFPoleSurface)
StdPrs_WFPoleSurface_Add = _StdPrs.StdPrs_WFPoleSurface_Add

class StdPrs_WFRestrictedFace(OCC.Core.Prs3d.Prs3d_Root):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    Add = _swig_new_static_method(_StdPrs.StdPrs_WFRestrictedFace_Add)
    AddUIso = _swig_new_static_method(_StdPrs.StdPrs_WFRestrictedFace_AddUIso)
    AddVIso = _swig_new_static_method(_StdPrs.StdPrs_WFRestrictedFace_AddVIso)
    Match = _swig_new_static_method(_StdPrs.StdPrs_WFRestrictedFace_Match)
    MatchUIso = _swig_new_static_method(_StdPrs.StdPrs_WFRestrictedFace_MatchUIso)
    MatchVIso = _swig_new_static_method(_StdPrs.StdPrs_WFRestrictedFace_MatchVIso)

    __repr__ = _dumps_object


    def __init__(self):
        _StdPrs.StdPrs_WFRestrictedFace_swiginit(self, _StdPrs.new_StdPrs_WFRestrictedFace())
    __swig_destroy__ = _StdPrs.delete_StdPrs_WFRestrictedFace

# Register StdPrs_WFRestrictedFace in _StdPrs:
_StdPrs.StdPrs_WFRestrictedFace_swigregister(StdPrs_WFRestrictedFace)
StdPrs_WFRestrictedFace_Add = _StdPrs.StdPrs_WFRestrictedFace_Add
StdPrs_WFRestrictedFace_AddUIso = _StdPrs.StdPrs_WFRestrictedFace_AddUIso
StdPrs_WFRestrictedFace_AddVIso = _StdPrs.StdPrs_WFRestrictedFace_AddVIso
StdPrs_WFRestrictedFace_Match = _StdPrs.StdPrs_WFRestrictedFace_Match
StdPrs_WFRestrictedFace_MatchUIso = _StdPrs.StdPrs_WFRestrictedFace_MatchUIso
StdPrs_WFRestrictedFace_MatchVIso = _StdPrs.StdPrs_WFRestrictedFace_MatchVIso

class StdPrs_WFShape(OCC.Core.Prs3d.Prs3d_Root):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined")
    __repr__ = _swig_repr
    Add = _swig_new_static_method(_StdPrs.StdPrs_WFShape_Add)
    AddAllEdges = _swig_new_static_method(_StdPrs.StdPrs_WFShape_AddAllEdges)
    AddEdgesOnTriangulation = _swig_new_static_method(_StdPrs.StdPrs_WFShape_AddEdgesOnTriangulation)
    AddVertexes = _swig_new_static_method(_StdPrs.StdPrs_WFShape_AddVertexes)

    __repr__ = _dumps_object

    __swig_destroy__ = _StdPrs.delete_StdPrs_WFShape

# Register StdPrs_WFShape in _StdPrs:
_StdPrs.StdPrs_WFShape_swigregister(StdPrs_WFShape)
StdPrs_WFShape_Add = _StdPrs.StdPrs_WFShape_Add
StdPrs_WFShape_AddAllEdges = _StdPrs.StdPrs_WFShape_AddAllEdges
StdPrs_WFShape_AddEdgesOnTriangulation = _StdPrs.StdPrs_WFShape_AddEdgesOnTriangulation
StdPrs_WFShape_AddVertexes = _StdPrs.StdPrs_WFShape_AddVertexes

class StdPrs_WFSurface(OCC.Core.Prs3d.Prs3d_Root):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    Add = _swig_new_static_method(_StdPrs.StdPrs_WFSurface_Add)

    __repr__ = _dumps_object


    def __init__(self):
        _StdPrs.StdPrs_WFSurface_swiginit(self, _StdPrs.new_StdPrs_WFSurface())
    __swig_destroy__ = _StdPrs.delete_StdPrs_WFSurface

# Register StdPrs_WFSurface in _StdPrs:
_StdPrs.StdPrs_WFSurface_swigregister(StdPrs_WFSurface)
StdPrs_WFSurface_Add = _StdPrs.StdPrs_WFSurface_Add



