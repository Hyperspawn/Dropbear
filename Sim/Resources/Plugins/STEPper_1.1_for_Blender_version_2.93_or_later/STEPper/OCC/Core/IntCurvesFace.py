# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.12
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

"""
IntCurvesFace module, see official documentation at
https://www.opencascade.com/doc/occt-7.4.0/refman/html/package_intcurvesface.html
"""


from sys import version_info as _swig_python_version_info
if _swig_python_version_info >= (3, 0, 0):
    new_instancemethod = lambda func, inst, cls: _IntCurvesFace.SWIG_PyInstanceMethod_New(func)
else:
    from new import instancemethod as new_instancemethod
if _swig_python_version_info >= (2, 7, 0):
    def swig_import_helper():
        import importlib
        pkg = __name__.rpartition('.')[0]
        mname = '.'.join((pkg, '_IntCurvesFace')).lstrip('.')
        try:
            return importlib.import_module(mname)
        except ImportError:
            return importlib.import_module('_IntCurvesFace')
    _IntCurvesFace = swig_import_helper()
    del swig_import_helper
elif _swig_python_version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_IntCurvesFace', [dirname(__file__)])
        except ImportError:
            import _IntCurvesFace
            return _IntCurvesFace
        try:
            _mod = imp.load_module('_IntCurvesFace', fp, pathname, description)
        finally:
            if fp is not None:
                fp.close()
        return _mod
    _IntCurvesFace = swig_import_helper()
    del swig_import_helper
else:
    import _IntCurvesFace
del _swig_python_version_info

try:
    _swig_property = property
except NameError:
    pass  # Python < 2.2 doesn't have 'property'.

try:
    import builtins as __builtin__
except ImportError:
    import __builtin__

def _swig_setattr_nondynamic(self, class_type, name, value, static=1):
    if (name == "thisown"):
        return self.this.own(value)
    if (name == "this"):
        if type(value).__name__ == 'SwigPyObject':
            self.__dict__[name] = value
            return
    method = class_type.__swig_setmethods__.get(name, None)
    if method:
        return method(self, value)
    if (not static):
        object.__setattr__(self, name, value)
    else:
        raise AttributeError("You cannot add attributes to %s" % self)


def _swig_setattr(self, class_type, name, value):
    return _swig_setattr_nondynamic(self, class_type, name, value, 0)


def _swig_getattr(self, class_type, name):
    if (name == "thisown"):
        return self.this.own()
    method = class_type.__swig_getmethods__.get(name, None)
    if method:
        return method(self)
    raise AttributeError("'%s' object has no attribute '%s'" % (class_type.__name__, name))


def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except __builtin__.Exception:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)


def _swig_setattr_nondynamic_method(set):
    def set_attr(self, name, value):
        if (name == "thisown"):
            return self.this.own(value)
        if hasattr(self, name) or (name == "this"):
            set(self, name, value)
        else:
            raise AttributeError("You cannot add attributes to %s" % self)
    return set_attr


class SwigPyIterator(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined - class is abstract")
    __repr__ = _swig_repr
    __swig_destroy__ = _IntCurvesFace.delete_SwigPyIterator
    def __iter__(self):
        return self
SwigPyIterator.value = new_instancemethod(_IntCurvesFace.SwigPyIterator_value, None, SwigPyIterator)
SwigPyIterator.incr = new_instancemethod(_IntCurvesFace.SwigPyIterator_incr, None, SwigPyIterator)
SwigPyIterator.decr = new_instancemethod(_IntCurvesFace.SwigPyIterator_decr, None, SwigPyIterator)
SwigPyIterator.distance = new_instancemethod(_IntCurvesFace.SwigPyIterator_distance, None, SwigPyIterator)
SwigPyIterator.equal = new_instancemethod(_IntCurvesFace.SwigPyIterator_equal, None, SwigPyIterator)
SwigPyIterator.copy = new_instancemethod(_IntCurvesFace.SwigPyIterator_copy, None, SwigPyIterator)
SwigPyIterator.next = new_instancemethod(_IntCurvesFace.SwigPyIterator_next, None, SwigPyIterator)
SwigPyIterator.__next__ = new_instancemethod(_IntCurvesFace.SwigPyIterator___next__, None, SwigPyIterator)
SwigPyIterator.previous = new_instancemethod(_IntCurvesFace.SwigPyIterator_previous, None, SwigPyIterator)
SwigPyIterator.advance = new_instancemethod(_IntCurvesFace.SwigPyIterator_advance, None, SwigPyIterator)
SwigPyIterator.__eq__ = new_instancemethod(_IntCurvesFace.SwigPyIterator___eq__, None, SwigPyIterator)
SwigPyIterator.__ne__ = new_instancemethod(_IntCurvesFace.SwigPyIterator___ne__, None, SwigPyIterator)
SwigPyIterator.__iadd__ = new_instancemethod(_IntCurvesFace.SwigPyIterator___iadd__, None, SwigPyIterator)
SwigPyIterator.__isub__ = new_instancemethod(_IntCurvesFace.SwigPyIterator___isub__, None, SwigPyIterator)
SwigPyIterator.__add__ = new_instancemethod(_IntCurvesFace.SwigPyIterator___add__, None, SwigPyIterator)
SwigPyIterator.__sub__ = new_instancemethod(_IntCurvesFace.SwigPyIterator___sub__, None, SwigPyIterator)
SwigPyIterator_swigregister = _IntCurvesFace.SwigPyIterator_swigregister
SwigPyIterator_swigregister(SwigPyIterator)


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


def process_exception(error: 'Standard_Failure const &', method_name: 'std::string', class_name: 'std::string') -> "void":
    return _IntCurvesFace.process_exception(error, method_name, class_name)
process_exception = _IntCurvesFace.process_exception

from six import with_metaclass
import warnings
from OCC.Wrapper.wrapper_utils import Proxy, deprecated

import OCC.Core.Standard
import OCC.Core.NCollection
import OCC.Core.Bnd
import OCC.Core.gp
import OCC.Core.TColStd
import OCC.Core.TCollection
import OCC.Core.TColgp
import OCC.Core.BVH
import OCC.Core.TopAbs
import OCC.Core.TopoDS
import OCC.Core.Message
import OCC.Core.TopLoc
import OCC.Core.Adaptor3d
import OCC.Core.Geom
import OCC.Core.GeomAbs
import OCC.Core.Adaptor2d
import OCC.Core.Geom2d
import OCC.Core.math
import OCC.Core.IntCurveSurface
import OCC.Core.Intf
import OCC.Core.IntSurf
class IntCurvesFace_Intersector(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def Bounding(self, *args) -> "Bnd_Box":
        """:rtype: Bnd_Box"""
        return _IntCurvesFace.IntCurvesFace_Intersector_Bounding(self, *args)


    def ClassifyUVPoint(self, *args) -> "TopAbs_State":
        """
        :param Puv:
        	:type Puv: gp_Pnt2d
        	:rtype: TopAbs_State
        """
        return _IntCurvesFace.IntCurvesFace_Intersector_ClassifyUVPoint(self, *args)


    def Destroy(self, *args) -> "void":
        """:rtype: None"""
        return _IntCurvesFace.IntCurvesFace_Intersector_Destroy(self, *args)


    def Face(self, *args) -> "TopoDS_Face const":
        """
        * Returns the significant face used to determine the intersection.
        	:rtype: TopoDS_Face
        """
        return _IntCurvesFace.IntCurvesFace_Intersector_Face(self, *args)


    def GetUseBoundToler(self, *args) -> "Standard_Boolean":
        """
        * Returns the boundary tolerance flag
        	:rtype: bool
        """
        return _IntCurvesFace.IntCurvesFace_Intersector_GetUseBoundToler(self, *args)


    def __init__(self, *args):
        """
        * Load a Face. //! The Tolerance <Tol> is used to determine if the first point of the segment is near the face. In that case, the parameter of the intersection point on the line can be a negative value (greater than -Tol). If aRestr = true UV bounding box of face is used to restrict it's underlined surface, otherwise surface is not restricted. If UseBToler = false then the 2d-point of intersection is classified with null-tolerance (relative to face); otherwise it's using maximium between input tolerance(aTol) and tolerances of face bounds (edges).
        	:param F:
        	:type F: TopoDS_Face
        	:param aTol:
        	:type aTol: float
        	:param aRestr: default value is Standard_True
        	:type aRestr: bool
        	:param UseBToler: default value is Standard_True
        	:type UseBToler: bool
        	:rtype: None
        """
        _IntCurvesFace.IntCurvesFace_Intersector_swiginit(self, _IntCurvesFace.new_IntCurvesFace_Intersector(*args))

    def IsDone(self, *args) -> "Standard_Boolean":
        """
        * True is returned when the intersection have been computed.
        	:rtype: bool
        """
        return _IntCurvesFace.IntCurvesFace_Intersector_IsDone(self, *args)


    def IsParallel(self, *args) -> "Standard_Boolean":
        """
        * Returns true if curve is parallel or belongs face surface This case is recognized only for some pairs of analytical curves and surfaces (plane - line, ...)
        	:rtype: bool
        """
        return _IntCurvesFace.IntCurvesFace_Intersector_IsParallel(self, *args)


    def NbPnt(self, *args) -> "Standard_Integer":
        """:rtype: int"""
        return _IntCurvesFace.IntCurvesFace_Intersector_NbPnt(self, *args)


    def Perform(self, *args) -> "void":
        """
        * Perform the intersection between the segment L and the loaded face. //! PInf is the smallest parameter on the line PSup is the highest parmaeter on the line //! For an infinite line PInf and PSup can be +/- RealLast.
        	:param L:
        	:type L: gp_Lin
        	:param PInf:
        	:type PInf: float
        	:param PSup:
        	:type PSup: float
        	:rtype: None
        * same method for a HCurve from Adaptor3d. PInf an PSup can also be - and + INF.
        	:param HCu:
        	:type HCu: Adaptor3d_HCurve
        	:param PInf:
        	:type PInf: float
        	:param PSup:
        	:type PSup: float
        	:rtype: None
        """
        return _IntCurvesFace.IntCurvesFace_Intersector_Perform(self, *args)


    def Pnt(self, *args) -> "gp_Pnt const":
        """
        * Returns the geometric point of the ith intersection between the line and the surface.
        	:param I:
        	:type I: int
        	:rtype: gp_Pnt
        """
        return _IntCurvesFace.IntCurvesFace_Intersector_Pnt(self, *args)


    def SetUseBoundToler(self, *args) -> "void":
        """
        * Sets the boundary tolerance flag
        	:param UseBToler:
        	:type UseBToler: bool
        	:rtype: None
        """
        return _IntCurvesFace.IntCurvesFace_Intersector_SetUseBoundToler(self, *args)


    def State(self, *args) -> "TopAbs_State":
        """
        * Returns the ith state of the point on the face. The values can be either TopAbs_IN ( the point is in the face) or TopAbs_ON ( the point is on a boudary of the face).
        	:param I:
        	:type I: int
        	:rtype: TopAbs_State
        """
        return _IntCurvesFace.IntCurvesFace_Intersector_State(self, *args)


    def SurfaceType(self, *args) -> "GeomAbs_SurfaceType":
        """
        * Return the surface type
        	:rtype: GeomAbs_SurfaceType
        """
        return _IntCurvesFace.IntCurvesFace_Intersector_SurfaceType(self, *args)


    def Transition(self, *args) -> "IntCurveSurface_TransitionOnCurve":
        """
        * Returns the ith transition of the line on the surface.
        	:param I:
        	:type I: int
        	:rtype: IntCurveSurface_TransitionOnCurve
        """
        return _IntCurvesFace.IntCurvesFace_Intersector_Transition(self, *args)


    def UParameter(self, *args) -> "Standard_Real":
        """
        * Returns the U parameter of the ith intersection point on the surface.
        	:param I:
        	:type I: int
        	:rtype: float
        """
        return _IntCurvesFace.IntCurvesFace_Intersector_UParameter(self, *args)


    def VParameter(self, *args) -> "Standard_Real":
        """
        * Returns the V parameter of the ith intersection point on the surface.
        	:param I:
        	:type I: int
        	:rtype: float
        """
        return _IntCurvesFace.IntCurvesFace_Intersector_VParameter(self, *args)


    def WParameter(self, *args) -> "Standard_Real":
        """
        * Returns the parameter of the ith intersection point on the line.
        	:param I:
        	:type I: int
        	:rtype: float
        """
        return _IntCurvesFace.IntCurvesFace_Intersector_WParameter(self, *args)


    __repr__ = _dumps_object

    __swig_destroy__ = _IntCurvesFace.delete_IntCurvesFace_Intersector
IntCurvesFace_Intersector.Bounding = new_instancemethod(_IntCurvesFace.IntCurvesFace_Intersector_Bounding, None, IntCurvesFace_Intersector)
IntCurvesFace_Intersector.ClassifyUVPoint = new_instancemethod(_IntCurvesFace.IntCurvesFace_Intersector_ClassifyUVPoint, None, IntCurvesFace_Intersector)
IntCurvesFace_Intersector.Destroy = new_instancemethod(_IntCurvesFace.IntCurvesFace_Intersector_Destroy, None, IntCurvesFace_Intersector)
IntCurvesFace_Intersector.Face = new_instancemethod(_IntCurvesFace.IntCurvesFace_Intersector_Face, None, IntCurvesFace_Intersector)
IntCurvesFace_Intersector.GetUseBoundToler = new_instancemethod(_IntCurvesFace.IntCurvesFace_Intersector_GetUseBoundToler, None, IntCurvesFace_Intersector)
IntCurvesFace_Intersector.IsDone = new_instancemethod(_IntCurvesFace.IntCurvesFace_Intersector_IsDone, None, IntCurvesFace_Intersector)
IntCurvesFace_Intersector.IsParallel = new_instancemethod(_IntCurvesFace.IntCurvesFace_Intersector_IsParallel, None, IntCurvesFace_Intersector)
IntCurvesFace_Intersector.NbPnt = new_instancemethod(_IntCurvesFace.IntCurvesFace_Intersector_NbPnt, None, IntCurvesFace_Intersector)
IntCurvesFace_Intersector.Perform = new_instancemethod(_IntCurvesFace.IntCurvesFace_Intersector_Perform, None, IntCurvesFace_Intersector)
IntCurvesFace_Intersector.Pnt = new_instancemethod(_IntCurvesFace.IntCurvesFace_Intersector_Pnt, None, IntCurvesFace_Intersector)
IntCurvesFace_Intersector.SetUseBoundToler = new_instancemethod(_IntCurvesFace.IntCurvesFace_Intersector_SetUseBoundToler, None, IntCurvesFace_Intersector)
IntCurvesFace_Intersector.State = new_instancemethod(_IntCurvesFace.IntCurvesFace_Intersector_State, None, IntCurvesFace_Intersector)
IntCurvesFace_Intersector.SurfaceType = new_instancemethod(_IntCurvesFace.IntCurvesFace_Intersector_SurfaceType, None, IntCurvesFace_Intersector)
IntCurvesFace_Intersector.Transition = new_instancemethod(_IntCurvesFace.IntCurvesFace_Intersector_Transition, None, IntCurvesFace_Intersector)
IntCurvesFace_Intersector.UParameter = new_instancemethod(_IntCurvesFace.IntCurvesFace_Intersector_UParameter, None, IntCurvesFace_Intersector)
IntCurvesFace_Intersector.VParameter = new_instancemethod(_IntCurvesFace.IntCurvesFace_Intersector_VParameter, None, IntCurvesFace_Intersector)
IntCurvesFace_Intersector.WParameter = new_instancemethod(_IntCurvesFace.IntCurvesFace_Intersector_WParameter, None, IntCurvesFace_Intersector)
IntCurvesFace_Intersector_swigregister = _IntCurvesFace.IntCurvesFace_Intersector_swigregister
IntCurvesFace_Intersector_swigregister(IntCurvesFace_Intersector)

class IntCurvesFace_ShapeIntersector(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def Destroy(self, *args) -> "void":
        """:rtype: None"""
        return _IntCurvesFace.IntCurvesFace_ShapeIntersector_Destroy(self, *args)


    def Face(self, *args) -> "TopoDS_Face const":
        """
        * Returns the significant face used to determine the intersection.
        	:param I:
        	:type I: int
        	:rtype: TopoDS_Face
        """
        return _IntCurvesFace.IntCurvesFace_ShapeIntersector_Face(self, *args)


    def __init__(self, *args):
        """:rtype: None"""
        _IntCurvesFace.IntCurvesFace_ShapeIntersector_swiginit(self, _IntCurvesFace.new_IntCurvesFace_ShapeIntersector(*args))

    def IsDone(self, *args) -> "Standard_Boolean":
        """
        * True is returned when the intersection have been computed.
        	:rtype: bool
        """
        return _IntCurvesFace.IntCurvesFace_ShapeIntersector_IsDone(self, *args)


    def Load(self, *args) -> "void":
        """
        :param Sh:
        	:type Sh: TopoDS_Shape
        	:param Tol:
        	:type Tol: float
        	:rtype: None
        """
        return _IntCurvesFace.IntCurvesFace_ShapeIntersector_Load(self, *args)


    def NbPnt(self, *args) -> "Standard_Integer":
        """:rtype: int"""
        return _IntCurvesFace.IntCurvesFace_ShapeIntersector_NbPnt(self, *args)


    def Perform(self, *args) -> "void":
        """
        * Perform the intersection between the segment L and the loaded shape. //! PInf is the smallest parameter on the line PSup is the highest parammter on the line //! For an infinite line PInf and PSup can be +/- RealLast.
        	:param L:
        	:type L: gp_Lin
        	:param PInf:
        	:type PInf: float
        	:param PSup:
        	:type PSup: float
        	:rtype: None
        * same method for a HCurve from Adaptor3d. PInf an PSup can also be - and + INF.
        	:param HCu:
        	:type HCu: Adaptor3d_HCurve
        	:param PInf:
        	:type PInf: float
        	:param PSup:
        	:type PSup: float
        	:rtype: None
        """
        return _IntCurvesFace.IntCurvesFace_ShapeIntersector_Perform(self, *args)


    def PerformNearest(self, *args) -> "void":
        """
        * Perform the intersection between the segment L and the loaded shape. //! PInf is the smallest parameter on the line PSup is the highest parammter on the line //! For an infinite line PInf and PSup can be +/- RealLast.
        	:param L:
        	:type L: gp_Lin
        	:param PInf:
        	:type PInf: float
        	:param PSup:
        	:type PSup: float
        	:rtype: None
        """
        return _IntCurvesFace.IntCurvesFace_ShapeIntersector_PerformNearest(self, *args)


    def Pnt(self, *args) -> "gp_Pnt const":
        """
        * Returns the geometric point of the ith intersection between the line and the surface.
        	:param I:
        	:type I: int
        	:rtype: gp_Pnt
        """
        return _IntCurvesFace.IntCurvesFace_ShapeIntersector_Pnt(self, *args)


    def SortResult(self, *args) -> "void":
        """
        * Internal method. Sort the result on the Curve parameter.
        	:rtype: None
        """
        return _IntCurvesFace.IntCurvesFace_ShapeIntersector_SortResult(self, *args)


    def State(self, *args) -> "TopAbs_State":
        """
        * Returns the ith state of the point on the face. The values can be either TopAbs_IN ( the point is in the face) or TopAbs_ON ( the point is on a boudary of the face).
        	:param I:
        	:type I: int
        	:rtype: TopAbs_State
        """
        return _IntCurvesFace.IntCurvesFace_ShapeIntersector_State(self, *args)


    def Transition(self, *args) -> "IntCurveSurface_TransitionOnCurve":
        """
        * Returns the ith transition of the line on the surface.
        	:param I:
        	:type I: int
        	:rtype: IntCurveSurface_TransitionOnCurve
        """
        return _IntCurvesFace.IntCurvesFace_ShapeIntersector_Transition(self, *args)


    def UParameter(self, *args) -> "Standard_Real":
        """
        * Returns the U parameter of the ith intersection point on the surface.
        	:param I:
        	:type I: int
        	:rtype: float
        """
        return _IntCurvesFace.IntCurvesFace_ShapeIntersector_UParameter(self, *args)


    def VParameter(self, *args) -> "Standard_Real":
        """
        * Returns the V parameter of the ith intersection point on the surface.
        	:param I:
        	:type I: int
        	:rtype: float
        """
        return _IntCurvesFace.IntCurvesFace_ShapeIntersector_VParameter(self, *args)


    def WParameter(self, *args) -> "Standard_Real":
        """
        * Returns the parameter of the ith intersection point on the line.
        	:param I:
        	:type I: int
        	:rtype: float
        """
        return _IntCurvesFace.IntCurvesFace_ShapeIntersector_WParameter(self, *args)


    __repr__ = _dumps_object

    __swig_destroy__ = _IntCurvesFace.delete_IntCurvesFace_ShapeIntersector
IntCurvesFace_ShapeIntersector.Destroy = new_instancemethod(_IntCurvesFace.IntCurvesFace_ShapeIntersector_Destroy, None, IntCurvesFace_ShapeIntersector)
IntCurvesFace_ShapeIntersector.Face = new_instancemethod(_IntCurvesFace.IntCurvesFace_ShapeIntersector_Face, None, IntCurvesFace_ShapeIntersector)
IntCurvesFace_ShapeIntersector.IsDone = new_instancemethod(_IntCurvesFace.IntCurvesFace_ShapeIntersector_IsDone, None, IntCurvesFace_ShapeIntersector)
IntCurvesFace_ShapeIntersector.Load = new_instancemethod(_IntCurvesFace.IntCurvesFace_ShapeIntersector_Load, None, IntCurvesFace_ShapeIntersector)
IntCurvesFace_ShapeIntersector.NbPnt = new_instancemethod(_IntCurvesFace.IntCurvesFace_ShapeIntersector_NbPnt, None, IntCurvesFace_ShapeIntersector)
IntCurvesFace_ShapeIntersector.Perform = new_instancemethod(_IntCurvesFace.IntCurvesFace_ShapeIntersector_Perform, None, IntCurvesFace_ShapeIntersector)
IntCurvesFace_ShapeIntersector.PerformNearest = new_instancemethod(_IntCurvesFace.IntCurvesFace_ShapeIntersector_PerformNearest, None, IntCurvesFace_ShapeIntersector)
IntCurvesFace_ShapeIntersector.Pnt = new_instancemethod(_IntCurvesFace.IntCurvesFace_ShapeIntersector_Pnt, None, IntCurvesFace_ShapeIntersector)
IntCurvesFace_ShapeIntersector.SortResult = new_instancemethod(_IntCurvesFace.IntCurvesFace_ShapeIntersector_SortResult, None, IntCurvesFace_ShapeIntersector)
IntCurvesFace_ShapeIntersector.State = new_instancemethod(_IntCurvesFace.IntCurvesFace_ShapeIntersector_State, None, IntCurvesFace_ShapeIntersector)
IntCurvesFace_ShapeIntersector.Transition = new_instancemethod(_IntCurvesFace.IntCurvesFace_ShapeIntersector_Transition, None, IntCurvesFace_ShapeIntersector)
IntCurvesFace_ShapeIntersector.UParameter = new_instancemethod(_IntCurvesFace.IntCurvesFace_ShapeIntersector_UParameter, None, IntCurvesFace_ShapeIntersector)
IntCurvesFace_ShapeIntersector.VParameter = new_instancemethod(_IntCurvesFace.IntCurvesFace_ShapeIntersector_VParameter, None, IntCurvesFace_ShapeIntersector)
IntCurvesFace_ShapeIntersector.WParameter = new_instancemethod(_IntCurvesFace.IntCurvesFace_ShapeIntersector_WParameter, None, IntCurvesFace_ShapeIntersector)
IntCurvesFace_ShapeIntersector_swigregister = _IntCurvesFace.IntCurvesFace_ShapeIntersector_swigregister
IntCurvesFace_ShapeIntersector_swigregister(IntCurvesFace_ShapeIntersector)


