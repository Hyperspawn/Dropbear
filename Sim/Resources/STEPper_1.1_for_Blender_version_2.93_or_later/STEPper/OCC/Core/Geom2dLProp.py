# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.12
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

"""
Geom2dLProp module, see official documentation at
https://www.opencascade.com/doc/occt-7.4.0/refman/html/package_geom2dlprop.html
"""


from sys import version_info as _swig_python_version_info
if _swig_python_version_info >= (3, 0, 0):
    new_instancemethod = lambda func, inst, cls: _Geom2dLProp.SWIG_PyInstanceMethod_New(func)
else:
    from new import instancemethod as new_instancemethod
if _swig_python_version_info >= (2, 7, 0):
    def swig_import_helper():
        import importlib
        pkg = __name__.rpartition('.')[0]
        mname = '.'.join((pkg, '_Geom2dLProp')).lstrip('.')
        try:
            return importlib.import_module(mname)
        except ImportError:
            return importlib.import_module('_Geom2dLProp')
    _Geom2dLProp = swig_import_helper()
    del swig_import_helper
elif _swig_python_version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_Geom2dLProp', [dirname(__file__)])
        except ImportError:
            import _Geom2dLProp
            return _Geom2dLProp
        try:
            _mod = imp.load_module('_Geom2dLProp', fp, pathname, description)
        finally:
            if fp is not None:
                fp.close()
        return _mod
    _Geom2dLProp = swig_import_helper()
    del swig_import_helper
else:
    import _Geom2dLProp
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
    __swig_destroy__ = _Geom2dLProp.delete_SwigPyIterator
    def __iter__(self):
        return self
SwigPyIterator.value = new_instancemethod(_Geom2dLProp.SwigPyIterator_value, None, SwigPyIterator)
SwigPyIterator.incr = new_instancemethod(_Geom2dLProp.SwigPyIterator_incr, None, SwigPyIterator)
SwigPyIterator.decr = new_instancemethod(_Geom2dLProp.SwigPyIterator_decr, None, SwigPyIterator)
SwigPyIterator.distance = new_instancemethod(_Geom2dLProp.SwigPyIterator_distance, None, SwigPyIterator)
SwigPyIterator.equal = new_instancemethod(_Geom2dLProp.SwigPyIterator_equal, None, SwigPyIterator)
SwigPyIterator.copy = new_instancemethod(_Geom2dLProp.SwigPyIterator_copy, None, SwigPyIterator)
SwigPyIterator.next = new_instancemethod(_Geom2dLProp.SwigPyIterator_next, None, SwigPyIterator)
SwigPyIterator.__next__ = new_instancemethod(_Geom2dLProp.SwigPyIterator___next__, None, SwigPyIterator)
SwigPyIterator.previous = new_instancemethod(_Geom2dLProp.SwigPyIterator_previous, None, SwigPyIterator)
SwigPyIterator.advance = new_instancemethod(_Geom2dLProp.SwigPyIterator_advance, None, SwigPyIterator)
SwigPyIterator.__eq__ = new_instancemethod(_Geom2dLProp.SwigPyIterator___eq__, None, SwigPyIterator)
SwigPyIterator.__ne__ = new_instancemethod(_Geom2dLProp.SwigPyIterator___ne__, None, SwigPyIterator)
SwigPyIterator.__iadd__ = new_instancemethod(_Geom2dLProp.SwigPyIterator___iadd__, None, SwigPyIterator)
SwigPyIterator.__isub__ = new_instancemethod(_Geom2dLProp.SwigPyIterator___isub__, None, SwigPyIterator)
SwigPyIterator.__add__ = new_instancemethod(_Geom2dLProp.SwigPyIterator___add__, None, SwigPyIterator)
SwigPyIterator.__sub__ = new_instancemethod(_Geom2dLProp.SwigPyIterator___sub__, None, SwigPyIterator)
SwigPyIterator_swigregister = _Geom2dLProp.SwigPyIterator_swigregister
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
    return _Geom2dLProp.process_exception(error, method_name, class_name)
process_exception = _Geom2dLProp.process_exception

from six import with_metaclass
import warnings
from OCC.Wrapper.wrapper_utils import Proxy, deprecated

import OCC.Core.Standard
import OCC.Core.NCollection
import OCC.Core.gp
import OCC.Core.Geom2d
import OCC.Core.GeomAbs
import OCC.Core.TColgp
import OCC.Core.TColStd
import OCC.Core.TCollection
import OCC.Core.LProp
import OCC.Core.math
import OCC.Core.Message
class Geom2dLProp_CLProps2d(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def CentreOfCurvature(self, *args) -> "void":
        """
        * Returns the centre of curvature <P>.
        	:param P:
        	:type P: gp_Pnt2d
        	:rtype: None
        """
        return _Geom2dLProp.Geom2dLProp_CLProps2d_CentreOfCurvature(self, *args)


    def Curvature(self, *args) -> "Standard_Real":
        """
        * Returns the curvature.
        	:rtype: float
        """
        return _Geom2dLProp.Geom2dLProp_CLProps2d_Curvature(self, *args)


    def D1(self, *args) -> "gp_Vec2d const":
        """
        * Returns the first derivative. The derivative is computed if it has not been yet.
        	:rtype: gp_Vec2d
        """
        return _Geom2dLProp.Geom2dLProp_CLProps2d_D1(self, *args)


    def D2(self, *args) -> "gp_Vec2d const":
        """
        * Returns the second derivative. The derivative is computed if it has not been yet.
        	:rtype: gp_Vec2d
        """
        return _Geom2dLProp.Geom2dLProp_CLProps2d_D2(self, *args)


    def D3(self, *args) -> "gp_Vec2d const":
        """
        * Returns the third derivative. The derivative is computed if it has not been yet.
        	:rtype: gp_Vec2d
        """
        return _Geom2dLProp.Geom2dLProp_CLProps2d_D3(self, *args)


    def __init__(self, *args):
        """
        * Initializes the local properties of the curve <C> The current point and the derivatives are computed at the same time, which allows an optimization of the computation time. <N> indicates the maximum number of derivations to be done (0, 1, 2 or 3). For example, to compute only the tangent, N should be equal to 1. <Resolution> is the linear tolerance (it is used to test if a vector is null).
        	:param C:
        	:type C: Geom2d_Curve
        	:param N:
        	:type N: int
        	:param Resolution:
        	:type Resolution: float
        	:rtype: None
        * Same as previous constructor but here the parameter is set to the value <U>. All the computations done will be related to <C> and <U>.
        	:param C:
        	:type C: Geom2d_Curve
        	:param U:
        	:type U: float
        	:param N:
        	:type N: int
        	:param Resolution:
        	:type Resolution: float
        	:rtype: None
        * Same as previous constructor but here the parameter is set to the value <U> and the curve is set with SetCurve. the curve can have a empty constructor All the computations done will be related to <C> and <U> when the functions 'set' will be done.
        	:param N:
        	:type N: int
        	:param Resolution:
        	:type Resolution: float
        	:rtype: None
        """
        _Geom2dLProp.Geom2dLProp_CLProps2d_swiginit(self, _Geom2dLProp.new_Geom2dLProp_CLProps2d(*args))

    def IsTangentDefined(self, *args) -> "Standard_Boolean":
        """
        * Returns True if the tangent is defined. For example, the tangent is not defined if the three first derivatives are all null.
        	:rtype: bool
        """
        return _Geom2dLProp.Geom2dLProp_CLProps2d_IsTangentDefined(self, *args)


    def Normal(self, *args) -> "void":
        """
        * Returns the normal direction <N>.
        	:param N:
        	:type N: gp_Dir2d
        	:rtype: None
        """
        return _Geom2dLProp.Geom2dLProp_CLProps2d_Normal(self, *args)


    def SetCurve(self, *args) -> "void":
        """
        * Initializes the local properties of the curve for the new curve.
        	:param C:
        	:type C: Geom2d_Curve
        	:rtype: None
        """
        return _Geom2dLProp.Geom2dLProp_CLProps2d_SetCurve(self, *args)


    def SetParameter(self, *args) -> "void":
        """
        * Initializes the local properties of the curve for the parameter value <U>.
        	:param U:
        	:type U: float
        	:rtype: None
        """
        return _Geom2dLProp.Geom2dLProp_CLProps2d_SetParameter(self, *args)


    def Tangent(self, *args) -> "void":
        """
        * output the tangent direction <D>
        	:param D:
        	:type D: gp_Dir2d
        	:rtype: None
        """
        return _Geom2dLProp.Geom2dLProp_CLProps2d_Tangent(self, *args)


    def Value(self, *args) -> "gp_Pnt2d const":
        """
        * Returns the Point.
        	:rtype: gp_Pnt2d
        """
        return _Geom2dLProp.Geom2dLProp_CLProps2d_Value(self, *args)


    __repr__ = _dumps_object

    __swig_destroy__ = _Geom2dLProp.delete_Geom2dLProp_CLProps2d
Geom2dLProp_CLProps2d.CentreOfCurvature = new_instancemethod(_Geom2dLProp.Geom2dLProp_CLProps2d_CentreOfCurvature, None, Geom2dLProp_CLProps2d)
Geom2dLProp_CLProps2d.Curvature = new_instancemethod(_Geom2dLProp.Geom2dLProp_CLProps2d_Curvature, None, Geom2dLProp_CLProps2d)
Geom2dLProp_CLProps2d.D1 = new_instancemethod(_Geom2dLProp.Geom2dLProp_CLProps2d_D1, None, Geom2dLProp_CLProps2d)
Geom2dLProp_CLProps2d.D2 = new_instancemethod(_Geom2dLProp.Geom2dLProp_CLProps2d_D2, None, Geom2dLProp_CLProps2d)
Geom2dLProp_CLProps2d.D3 = new_instancemethod(_Geom2dLProp.Geom2dLProp_CLProps2d_D3, None, Geom2dLProp_CLProps2d)
Geom2dLProp_CLProps2d.IsTangentDefined = new_instancemethod(_Geom2dLProp.Geom2dLProp_CLProps2d_IsTangentDefined, None, Geom2dLProp_CLProps2d)
Geom2dLProp_CLProps2d.Normal = new_instancemethod(_Geom2dLProp.Geom2dLProp_CLProps2d_Normal, None, Geom2dLProp_CLProps2d)
Geom2dLProp_CLProps2d.SetCurve = new_instancemethod(_Geom2dLProp.Geom2dLProp_CLProps2d_SetCurve, None, Geom2dLProp_CLProps2d)
Geom2dLProp_CLProps2d.SetParameter = new_instancemethod(_Geom2dLProp.Geom2dLProp_CLProps2d_SetParameter, None, Geom2dLProp_CLProps2d)
Geom2dLProp_CLProps2d.Tangent = new_instancemethod(_Geom2dLProp.Geom2dLProp_CLProps2d_Tangent, None, Geom2dLProp_CLProps2d)
Geom2dLProp_CLProps2d.Value = new_instancemethod(_Geom2dLProp.Geom2dLProp_CLProps2d_Value, None, Geom2dLProp_CLProps2d)
Geom2dLProp_CLProps2d_swigregister = _Geom2dLProp.Geom2dLProp_CLProps2d_swigregister
Geom2dLProp_CLProps2d_swigregister(Geom2dLProp_CLProps2d)

class Geom2dLProp_CurAndInf2d(OCC.Core.LProp.LProp_CurAndInf):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def __init__(self, *args):
        """
        * Initializes the framework. Note: The curve on which the local properties are computed is defined using one of the following functions: Perform, PerformCurExt or PerformInf.
        	:rtype: None
        """
        _Geom2dLProp.Geom2dLProp_CurAndInf2d_swiginit(self, _Geom2dLProp.new_Geom2dLProp_CurAndInf2d(*args))

    def IsDone(self, *args) -> "Standard_Boolean":
        """
        * True if the solutions are found.
        	:rtype: bool
        """
        return _Geom2dLProp.Geom2dLProp_CurAndInf2d_IsDone(self, *args)


    def Perform(self, *args) -> "void":
        """
        * For the curve C, Computes both the inflection points and the maximum and minimum curvatures.
        	:param C:
        	:type C: Geom2d_Curve
        	:rtype: None
        """
        return _Geom2dLProp.Geom2dLProp_CurAndInf2d_Perform(self, *args)


    def PerformCurExt(self, *args) -> "void":
        """
        * For the curve C, Computes the locals extremas of curvature.
        	:param C:
        	:type C: Geom2d_Curve
        	:rtype: None
        """
        return _Geom2dLProp.Geom2dLProp_CurAndInf2d_PerformCurExt(self, *args)


    def PerformInf(self, *args) -> "void":
        """
        * For the curve C, Computes the inflections. After computation, the following functions can be used: - IsDone to check if the computation was successful - NbPoints to obtain the number of computed particular points - Parameter to obtain the parameter on the curve for each particular point - Type to check if the point is an inflection point or an extremum of curvature of the curve C. Warning These functions can be used to analyze a series of curves, however it is necessary to clear the table of results between each computation.
        	:param C:
        	:type C: Geom2d_Curve
        	:rtype: None
        """
        return _Geom2dLProp.Geom2dLProp_CurAndInf2d_PerformInf(self, *args)


    __repr__ = _dumps_object

    __swig_destroy__ = _Geom2dLProp.delete_Geom2dLProp_CurAndInf2d
Geom2dLProp_CurAndInf2d.IsDone = new_instancemethod(_Geom2dLProp.Geom2dLProp_CurAndInf2d_IsDone, None, Geom2dLProp_CurAndInf2d)
Geom2dLProp_CurAndInf2d.Perform = new_instancemethod(_Geom2dLProp.Geom2dLProp_CurAndInf2d_Perform, None, Geom2dLProp_CurAndInf2d)
Geom2dLProp_CurAndInf2d.PerformCurExt = new_instancemethod(_Geom2dLProp.Geom2dLProp_CurAndInf2d_PerformCurExt, None, Geom2dLProp_CurAndInf2d)
Geom2dLProp_CurAndInf2d.PerformInf = new_instancemethod(_Geom2dLProp.Geom2dLProp_CurAndInf2d_PerformInf, None, Geom2dLProp_CurAndInf2d)
Geom2dLProp_CurAndInf2d_swigregister = _Geom2dLProp.Geom2dLProp_CurAndInf2d_swigregister
Geom2dLProp_CurAndInf2d_swigregister(Geom2dLProp_CurAndInf2d)

class Geom2dLProp_Curve2dTool(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def Continuity(*args) -> "Standard_Integer":
        """
        * returns the order of continuity of the curve <C>. returns 1 : first derivative only is computable returns 2 : first and second derivative only are computable. returns 3 : first, second and third are computable.
        	:param C:
        	:type C: Geom2d_Curve
        	:rtype: int
        """
        return _Geom2dLProp.Geom2dLProp_Curve2dTool_Continuity(*args)

    Continuity = staticmethod(Continuity)

    def D1(*args) -> "void":
        """
        * Computes the point <P> and first derivative <V1> of parameter <U> on the curve <C>.
        	:param C:
        	:type C: Geom2d_Curve
        	:param U:
        	:type U: float
        	:param P:
        	:type P: gp_Pnt2d
        	:param V1:
        	:type V1: gp_Vec2d
        	:rtype: void
        """
        return _Geom2dLProp.Geom2dLProp_Curve2dTool_D1(*args)

    D1 = staticmethod(D1)

    def D2(*args) -> "void":
        """
        * Computes the point <P>, the first derivative <V1> and second derivative <V2> of parameter <U> on the curve <C>.
        	:param C:
        	:type C: Geom2d_Curve
        	:param U:
        	:type U: float
        	:param P:
        	:type P: gp_Pnt2d
        	:param V1:
        	:type V1: gp_Vec2d
        	:param V2:
        	:type V2: gp_Vec2d
        	:rtype: void
        """
        return _Geom2dLProp.Geom2dLProp_Curve2dTool_D2(*args)

    D2 = staticmethod(D2)

    def D3(*args) -> "void":
        """
        * Computes the point <P>, the first derivative <V1>, the second derivative <V2> and third derivative <V3> of parameter <U> on the curve <C>.
        	:param C:
        	:type C: Geom2d_Curve
        	:param U:
        	:type U: float
        	:param P:
        	:type P: gp_Pnt2d
        	:param V1:
        	:type V1: gp_Vec2d
        	:param V2:
        	:type V2: gp_Vec2d
        	:param V3:
        	:type V3: gp_Vec2d
        	:rtype: void
        """
        return _Geom2dLProp.Geom2dLProp_Curve2dTool_D3(*args)

    D3 = staticmethod(D3)

    def FirstParameter(*args) -> "Standard_Real":
        """
        * returns the first parameter bound of the curve.
        	:param C:
        	:type C: Geom2d_Curve
        	:rtype: float
        """
        return _Geom2dLProp.Geom2dLProp_Curve2dTool_FirstParameter(*args)

    FirstParameter = staticmethod(FirstParameter)

    def LastParameter(*args) -> "Standard_Real":
        """
        * returns the last parameter bound of the curve. FirstParameter must be less than LastParameter.
        	:param C:
        	:type C: Geom2d_Curve
        	:rtype: float
        """
        return _Geom2dLProp.Geom2dLProp_Curve2dTool_LastParameter(*args)

    LastParameter = staticmethod(LastParameter)

    def Value(*args) -> "void":
        """
        * Computes the point <P> of parameter <U> on the curve <C>.
        	:param C:
        	:type C: Geom2d_Curve
        	:param U:
        	:type U: float
        	:param P:
        	:type P: gp_Pnt2d
        	:rtype: void
        """
        return _Geom2dLProp.Geom2dLProp_Curve2dTool_Value(*args)

    Value = staticmethod(Value)

    __repr__ = _dumps_object


    def __init__(self):
        _Geom2dLProp.Geom2dLProp_Curve2dTool_swiginit(self, _Geom2dLProp.new_Geom2dLProp_Curve2dTool())
    __swig_destroy__ = _Geom2dLProp.delete_Geom2dLProp_Curve2dTool
Geom2dLProp_Curve2dTool_swigregister = _Geom2dLProp.Geom2dLProp_Curve2dTool_swigregister
Geom2dLProp_Curve2dTool_swigregister(Geom2dLProp_Curve2dTool)

def Geom2dLProp_Curve2dTool_Continuity(*args) -> "Standard_Integer":
    """
    * returns the order of continuity of the curve <C>. returns 1 : first derivative only is computable returns 2 : first and second derivative only are computable. returns 3 : first, second and third are computable.
    	:param C:
    	:type C: Geom2d_Curve
    	:rtype: int
    """
    return _Geom2dLProp.Geom2dLProp_Curve2dTool_Continuity(*args)

def Geom2dLProp_Curve2dTool_D1(*args) -> "void":
    """
    * Computes the point <P> and first derivative <V1> of parameter <U> on the curve <C>.
    	:param C:
    	:type C: Geom2d_Curve
    	:param U:
    	:type U: float
    	:param P:
    	:type P: gp_Pnt2d
    	:param V1:
    	:type V1: gp_Vec2d
    	:rtype: void
    """
    return _Geom2dLProp.Geom2dLProp_Curve2dTool_D1(*args)

def Geom2dLProp_Curve2dTool_D2(*args) -> "void":
    """
    * Computes the point <P>, the first derivative <V1> and second derivative <V2> of parameter <U> on the curve <C>.
    	:param C:
    	:type C: Geom2d_Curve
    	:param U:
    	:type U: float
    	:param P:
    	:type P: gp_Pnt2d
    	:param V1:
    	:type V1: gp_Vec2d
    	:param V2:
    	:type V2: gp_Vec2d
    	:rtype: void
    """
    return _Geom2dLProp.Geom2dLProp_Curve2dTool_D2(*args)

def Geom2dLProp_Curve2dTool_D3(*args) -> "void":
    """
    * Computes the point <P>, the first derivative <V1>, the second derivative <V2> and third derivative <V3> of parameter <U> on the curve <C>.
    	:param C:
    	:type C: Geom2d_Curve
    	:param U:
    	:type U: float
    	:param P:
    	:type P: gp_Pnt2d
    	:param V1:
    	:type V1: gp_Vec2d
    	:param V2:
    	:type V2: gp_Vec2d
    	:param V3:
    	:type V3: gp_Vec2d
    	:rtype: void
    """
    return _Geom2dLProp.Geom2dLProp_Curve2dTool_D3(*args)

def Geom2dLProp_Curve2dTool_FirstParameter(*args) -> "Standard_Real":
    """
    * returns the first parameter bound of the curve.
    	:param C:
    	:type C: Geom2d_Curve
    	:rtype: float
    """
    return _Geom2dLProp.Geom2dLProp_Curve2dTool_FirstParameter(*args)

def Geom2dLProp_Curve2dTool_LastParameter(*args) -> "Standard_Real":
    """
    * returns the last parameter bound of the curve. FirstParameter must be less than LastParameter.
    	:param C:
    	:type C: Geom2d_Curve
    	:rtype: float
    """
    return _Geom2dLProp.Geom2dLProp_Curve2dTool_LastParameter(*args)

def Geom2dLProp_Curve2dTool_Value(*args) -> "void":
    """
    * Computes the point <P> of parameter <U> on the curve <C>.
    	:param C:
    	:type C: Geom2d_Curve
    	:param U:
    	:type U: float
    	:param P:
    	:type P: gp_Pnt2d
    	:rtype: void
    """
    return _Geom2dLProp.Geom2dLProp_Curve2dTool_Value(*args)

class Geom2dLProp_FuncCurExt(OCC.Core.math.math_FunctionWithDerivative):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def __init__(self, *args):
        """
        :param C:
        	:type C: Geom2d_Curve
        	:param Tol:
        	:type Tol: float
        	:rtype: None
        """
        _Geom2dLProp.Geom2dLProp_FuncCurExt_swiginit(self, _Geom2dLProp.new_Geom2dLProp_FuncCurExt(*args))

    def IsMinKC(self, *args) -> "Standard_Boolean":
        """
        * True if Param corresponds to a minus of the radius of curvature.
        	:param Param:
        	:type Param: float
        	:rtype: bool
        """
        return _Geom2dLProp.Geom2dLProp_FuncCurExt_IsMinKC(self, *args)


    __repr__ = _dumps_object

    __swig_destroy__ = _Geom2dLProp.delete_Geom2dLProp_FuncCurExt
Geom2dLProp_FuncCurExt.IsMinKC = new_instancemethod(_Geom2dLProp.Geom2dLProp_FuncCurExt_IsMinKC, None, Geom2dLProp_FuncCurExt)
Geom2dLProp_FuncCurExt_swigregister = _Geom2dLProp.Geom2dLProp_FuncCurExt_swigregister
Geom2dLProp_FuncCurExt_swigregister(Geom2dLProp_FuncCurExt)

class Geom2dLProp_FuncCurNul(OCC.Core.math.math_FunctionWithDerivative):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def __init__(self, *args):
        """
        :param C:
        	:type C: Geom2d_Curve
        	:rtype: None
        """
        _Geom2dLProp.Geom2dLProp_FuncCurNul_swiginit(self, _Geom2dLProp.new_Geom2dLProp_FuncCurNul(*args))

    __repr__ = _dumps_object

    __swig_destroy__ = _Geom2dLProp.delete_Geom2dLProp_FuncCurNul
Geom2dLProp_FuncCurNul_swigregister = _Geom2dLProp.Geom2dLProp_FuncCurNul_swigregister
Geom2dLProp_FuncCurNul_swigregister(Geom2dLProp_FuncCurNul)

class Geom2dLProp_NumericCurInf2d(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def __init__(self, *args):
        """:rtype: None"""
        _Geom2dLProp.Geom2dLProp_NumericCurInf2d_swiginit(self, _Geom2dLProp.new_Geom2dLProp_NumericCurInf2d(*args))

    def IsDone(self, *args) -> "Standard_Boolean":
        """
        * True if the solutions are found.
        	:rtype: bool
        """
        return _Geom2dLProp.Geom2dLProp_NumericCurInf2d_IsDone(self, *args)


    def PerformCurExt(self, *args) -> "void":
        """
        * Computes the locals extremas of curvature.
        	:param C:
        	:type C: Geom2d_Curve
        	:param Result:
        	:type Result: LProp_CurAndInf
        	:rtype: None
        * Computes the locals extremas of curvature. in the interval of parmeters [UMin,UMax].
        	:param C:
        	:type C: Geom2d_Curve
        	:param UMin:
        	:type UMin: float
        	:param UMax:
        	:type UMax: float
        	:param Result:
        	:type Result: LProp_CurAndInf
        	:rtype: None
        """
        return _Geom2dLProp.Geom2dLProp_NumericCurInf2d_PerformCurExt(self, *args)


    def PerformInf(self, *args) -> "void":
        """
        * Computes the inflections.
        	:param C:
        	:type C: Geom2d_Curve
        	:param Result:
        	:type Result: LProp_CurAndInf
        	:rtype: None
        * Computes the inflections in the interval of parmeters [UMin,UMax].
        	:param C:
        	:type C: Geom2d_Curve
        	:param UMin:
        	:type UMin: float
        	:param UMax:
        	:type UMax: float
        	:param Result:
        	:type Result: LProp_CurAndInf
        	:rtype: None
        """
        return _Geom2dLProp.Geom2dLProp_NumericCurInf2d_PerformInf(self, *args)


    __repr__ = _dumps_object

    __swig_destroy__ = _Geom2dLProp.delete_Geom2dLProp_NumericCurInf2d
Geom2dLProp_NumericCurInf2d.IsDone = new_instancemethod(_Geom2dLProp.Geom2dLProp_NumericCurInf2d_IsDone, None, Geom2dLProp_NumericCurInf2d)
Geom2dLProp_NumericCurInf2d.PerformCurExt = new_instancemethod(_Geom2dLProp.Geom2dLProp_NumericCurInf2d_PerformCurExt, None, Geom2dLProp_NumericCurInf2d)
Geom2dLProp_NumericCurInf2d.PerformInf = new_instancemethod(_Geom2dLProp.Geom2dLProp_NumericCurInf2d_PerformInf, None, Geom2dLProp_NumericCurInf2d)
Geom2dLProp_NumericCurInf2d_swigregister = _Geom2dLProp.Geom2dLProp_NumericCurInf2d_swigregister
Geom2dLProp_NumericCurInf2d_swigregister(Geom2dLProp_NumericCurInf2d)


