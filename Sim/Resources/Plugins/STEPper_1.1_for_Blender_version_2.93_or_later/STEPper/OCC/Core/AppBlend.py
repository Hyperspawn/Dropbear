# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.12
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

"""
AppBlend module, see official documentation at
https://www.opencascade.com/doc/occt-7.4.0/refman/html/package_appblend.html
"""


from sys import version_info as _swig_python_version_info
if _swig_python_version_info >= (3, 0, 0):
    new_instancemethod = lambda func, inst, cls: _AppBlend.SWIG_PyInstanceMethod_New(func)
else:
    from new import instancemethod as new_instancemethod
if _swig_python_version_info >= (2, 7, 0):
    def swig_import_helper():
        import importlib
        pkg = __name__.rpartition('.')[0]
        mname = '.'.join((pkg, '_AppBlend')).lstrip('.')
        try:
            return importlib.import_module(mname)
        except ImportError:
            return importlib.import_module('_AppBlend')
    _AppBlend = swig_import_helper()
    del swig_import_helper
elif _swig_python_version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_AppBlend', [dirname(__file__)])
        except ImportError:
            import _AppBlend
            return _AppBlend
        try:
            _mod = imp.load_module('_AppBlend', fp, pathname, description)
        finally:
            if fp is not None:
                fp.close()
        return _mod
    _AppBlend = swig_import_helper()
    del swig_import_helper
else:
    import _AppBlend
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
    __swig_destroy__ = _AppBlend.delete_SwigPyIterator
    def __iter__(self):
        return self
SwigPyIterator.value = new_instancemethod(_AppBlend.SwigPyIterator_value, None, SwigPyIterator)
SwigPyIterator.incr = new_instancemethod(_AppBlend.SwigPyIterator_incr, None, SwigPyIterator)
SwigPyIterator.decr = new_instancemethod(_AppBlend.SwigPyIterator_decr, None, SwigPyIterator)
SwigPyIterator.distance = new_instancemethod(_AppBlend.SwigPyIterator_distance, None, SwigPyIterator)
SwigPyIterator.equal = new_instancemethod(_AppBlend.SwigPyIterator_equal, None, SwigPyIterator)
SwigPyIterator.copy = new_instancemethod(_AppBlend.SwigPyIterator_copy, None, SwigPyIterator)
SwigPyIterator.next = new_instancemethod(_AppBlend.SwigPyIterator_next, None, SwigPyIterator)
SwigPyIterator.__next__ = new_instancemethod(_AppBlend.SwigPyIterator___next__, None, SwigPyIterator)
SwigPyIterator.previous = new_instancemethod(_AppBlend.SwigPyIterator_previous, None, SwigPyIterator)
SwigPyIterator.advance = new_instancemethod(_AppBlend.SwigPyIterator_advance, None, SwigPyIterator)
SwigPyIterator.__eq__ = new_instancemethod(_AppBlend.SwigPyIterator___eq__, None, SwigPyIterator)
SwigPyIterator.__ne__ = new_instancemethod(_AppBlend.SwigPyIterator___ne__, None, SwigPyIterator)
SwigPyIterator.__iadd__ = new_instancemethod(_AppBlend.SwigPyIterator___iadd__, None, SwigPyIterator)
SwigPyIterator.__isub__ = new_instancemethod(_AppBlend.SwigPyIterator___isub__, None, SwigPyIterator)
SwigPyIterator.__add__ = new_instancemethod(_AppBlend.SwigPyIterator___add__, None, SwigPyIterator)
SwigPyIterator.__sub__ = new_instancemethod(_AppBlend.SwigPyIterator___sub__, None, SwigPyIterator)
SwigPyIterator_swigregister = _AppBlend.SwigPyIterator_swigregister
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
    return _AppBlend.process_exception(error, method_name, class_name)
process_exception = _AppBlend.process_exception

from six import with_metaclass
import warnings
from OCC.Wrapper.wrapper_utils import Proxy, deprecated

import OCC.Core.Standard
import OCC.Core.NCollection
import OCC.Core.TColgp
import OCC.Core.TColStd
import OCC.Core.TCollection
class AppBlend_Approx(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined")
    __repr__ = _swig_repr

    def Curve2d(self, *args) -> "void":
        """
        :param Index:
        	:type Index: int
        	:param TPoles:
        	:type TPoles: TColgp_Array1OfPnt2d
        	:param TKnots:
        	:type TKnots: TColStd_Array1OfReal
        	:param TMults:
        	:type TMults: TColStd_Array1OfInteger
        	:rtype: void
        """
        return _AppBlend.AppBlend_Approx_Curve2d(self, *args)


    def Curve2dPoles(self, *args) -> "TColgp_Array1OfPnt2d const &":
        """
        :param Index:
        	:type Index: int
        	:rtype: TColgp_Array1OfPnt2d
        """
        return _AppBlend.AppBlend_Approx_Curve2dPoles(self, *args)


    def Curves2dDegree(self, *args) -> "Standard_Integer":
        """:rtype: int"""
        return _AppBlend.AppBlend_Approx_Curves2dDegree(self, *args)


    def Curves2dKnots(self, *args) -> "TColStd_Array1OfReal const &":
        """:rtype: TColStd_Array1OfReal"""
        return _AppBlend.AppBlend_Approx_Curves2dKnots(self, *args)


    def Curves2dMults(self, *args) -> "TColStd_Array1OfInteger const &":
        """:rtype: TColStd_Array1OfInteger"""
        return _AppBlend.AppBlend_Approx_Curves2dMults(self, *args)


    def Curves2dShape(self, *args) -> "void":
        """
        :param Degree:
        	:type Degree: int
        	:param NbPoles:
        	:type NbPoles: int
        	:param NbKnots:
        	:type NbKnots: int
        	:rtype: void
        """
        return _AppBlend.AppBlend_Approx_Curves2dShape(self, *args)


    def IsDone(self, *args) -> "Standard_Boolean":
        """:rtype: bool"""
        return _AppBlend.AppBlend_Approx_IsDone(self, *args)


    def NbCurves2d(self, *args) -> "Standard_Integer":
        """:rtype: int"""
        return _AppBlend.AppBlend_Approx_NbCurves2d(self, *args)


    def SurfPoles(self, *args) -> "TColgp_Array2OfPnt const &":
        """:rtype: TColgp_Array2OfPnt"""
        return _AppBlend.AppBlend_Approx_SurfPoles(self, *args)


    def SurfShape(self, *args) -> "void":
        """
        :param UDegree:
        	:type UDegree: int
        	:param VDegree:
        	:type VDegree: int
        	:param NbUPoles:
        	:type NbUPoles: int
        	:param NbVPoles:
        	:type NbVPoles: int
        	:param NbUKnots:
        	:type NbUKnots: int
        	:param NbVKnots:
        	:type NbVKnots: int
        	:rtype: void
        """
        return _AppBlend.AppBlend_Approx_SurfShape(self, *args)


    def SurfUKnots(self, *args) -> "TColStd_Array1OfReal const &":
        """:rtype: TColStd_Array1OfReal"""
        return _AppBlend.AppBlend_Approx_SurfUKnots(self, *args)


    def SurfUMults(self, *args) -> "TColStd_Array1OfInteger const &":
        """:rtype: TColStd_Array1OfInteger"""
        return _AppBlend.AppBlend_Approx_SurfUMults(self, *args)


    def SurfVKnots(self, *args) -> "TColStd_Array1OfReal const &":
        """:rtype: TColStd_Array1OfReal"""
        return _AppBlend.AppBlend_Approx_SurfVKnots(self, *args)


    def SurfVMults(self, *args) -> "TColStd_Array1OfInteger const &":
        """:rtype: TColStd_Array1OfInteger"""
        return _AppBlend.AppBlend_Approx_SurfVMults(self, *args)


    def SurfWeights(self, *args) -> "TColStd_Array2OfReal const &":
        """:rtype: TColStd_Array2OfReal"""
        return _AppBlend.AppBlend_Approx_SurfWeights(self, *args)


    def Surface(self, *args) -> "void":
        """
        :param TPoles:
        	:type TPoles: TColgp_Array2OfPnt
        	:param TWeights:
        	:type TWeights: TColStd_Array2OfReal
        	:param TUKnots:
        	:type TUKnots: TColStd_Array1OfReal
        	:param TVKnots:
        	:type TVKnots: TColStd_Array1OfReal
        	:param TUMults:
        	:type TUMults: TColStd_Array1OfInteger
        	:param TVMults:
        	:type TVMults: TColStd_Array1OfInteger
        	:rtype: void
        """
        return _AppBlend.AppBlend_Approx_Surface(self, *args)


    def TolCurveOnSurf(self, *args) -> "Standard_Real":
        """
        :param Index:
        	:type Index: int
        	:rtype: float
        """
        return _AppBlend.AppBlend_Approx_TolCurveOnSurf(self, *args)


    def TolReached(self, *args) -> "void":
        """
        :param Tol3d:
        	:type Tol3d: float
        	:param Tol2d:
        	:type Tol2d: float
        	:rtype: void
        """
        return _AppBlend.AppBlend_Approx_TolReached(self, *args)


    def UDegree(self, *args) -> "Standard_Integer":
        """:rtype: int"""
        return _AppBlend.AppBlend_Approx_UDegree(self, *args)


    def VDegree(self, *args) -> "Standard_Integer":
        """:rtype: int"""
        return _AppBlend.AppBlend_Approx_VDegree(self, *args)


    __repr__ = _dumps_object

    __swig_destroy__ = _AppBlend.delete_AppBlend_Approx
AppBlend_Approx.Curve2d = new_instancemethod(_AppBlend.AppBlend_Approx_Curve2d, None, AppBlend_Approx)
AppBlend_Approx.Curve2dPoles = new_instancemethod(_AppBlend.AppBlend_Approx_Curve2dPoles, None, AppBlend_Approx)
AppBlend_Approx.Curves2dDegree = new_instancemethod(_AppBlend.AppBlend_Approx_Curves2dDegree, None, AppBlend_Approx)
AppBlend_Approx.Curves2dKnots = new_instancemethod(_AppBlend.AppBlend_Approx_Curves2dKnots, None, AppBlend_Approx)
AppBlend_Approx.Curves2dMults = new_instancemethod(_AppBlend.AppBlend_Approx_Curves2dMults, None, AppBlend_Approx)
AppBlend_Approx.Curves2dShape = new_instancemethod(_AppBlend.AppBlend_Approx_Curves2dShape, None, AppBlend_Approx)
AppBlend_Approx.IsDone = new_instancemethod(_AppBlend.AppBlend_Approx_IsDone, None, AppBlend_Approx)
AppBlend_Approx.NbCurves2d = new_instancemethod(_AppBlend.AppBlend_Approx_NbCurves2d, None, AppBlend_Approx)
AppBlend_Approx.SurfPoles = new_instancemethod(_AppBlend.AppBlend_Approx_SurfPoles, None, AppBlend_Approx)
AppBlend_Approx.SurfShape = new_instancemethod(_AppBlend.AppBlend_Approx_SurfShape, None, AppBlend_Approx)
AppBlend_Approx.SurfUKnots = new_instancemethod(_AppBlend.AppBlend_Approx_SurfUKnots, None, AppBlend_Approx)
AppBlend_Approx.SurfUMults = new_instancemethod(_AppBlend.AppBlend_Approx_SurfUMults, None, AppBlend_Approx)
AppBlend_Approx.SurfVKnots = new_instancemethod(_AppBlend.AppBlend_Approx_SurfVKnots, None, AppBlend_Approx)
AppBlend_Approx.SurfVMults = new_instancemethod(_AppBlend.AppBlend_Approx_SurfVMults, None, AppBlend_Approx)
AppBlend_Approx.SurfWeights = new_instancemethod(_AppBlend.AppBlend_Approx_SurfWeights, None, AppBlend_Approx)
AppBlend_Approx.Surface = new_instancemethod(_AppBlend.AppBlend_Approx_Surface, None, AppBlend_Approx)
AppBlend_Approx.TolCurveOnSurf = new_instancemethod(_AppBlend.AppBlend_Approx_TolCurveOnSurf, None, AppBlend_Approx)
AppBlend_Approx.TolReached = new_instancemethod(_AppBlend.AppBlend_Approx_TolReached, None, AppBlend_Approx)
AppBlend_Approx.UDegree = new_instancemethod(_AppBlend.AppBlend_Approx_UDegree, None, AppBlend_Approx)
AppBlend_Approx.VDegree = new_instancemethod(_AppBlend.AppBlend_Approx_VDegree, None, AppBlend_Approx)
AppBlend_Approx_swigregister = _AppBlend.AppBlend_Approx_swigregister
AppBlend_Approx_swigregister(AppBlend_Approx)



