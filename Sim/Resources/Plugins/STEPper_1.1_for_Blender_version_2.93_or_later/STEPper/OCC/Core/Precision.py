# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.12
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

"""
Precision module, see official documentation at
https://www.opencascade.com/doc/occt-7.4.0/refman/html/package_precision.html
"""


from sys import version_info as _swig_python_version_info
if _swig_python_version_info >= (3, 0, 0):
    new_instancemethod = lambda func, inst, cls: _Precision.SWIG_PyInstanceMethod_New(func)
else:
    from new import instancemethod as new_instancemethod
if _swig_python_version_info >= (2, 7, 0):
    def swig_import_helper():
        import importlib
        pkg = __name__.rpartition('.')[0]
        mname = '.'.join((pkg, '_Precision')).lstrip('.')
        try:
            return importlib.import_module(mname)
        except ImportError:
            return importlib.import_module('_Precision')
    _Precision = swig_import_helper()
    del swig_import_helper
elif _swig_python_version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_Precision', [dirname(__file__)])
        except ImportError:
            import _Precision
            return _Precision
        try:
            _mod = imp.load_module('_Precision', fp, pathname, description)
        finally:
            if fp is not None:
                fp.close()
        return _mod
    _Precision = swig_import_helper()
    del swig_import_helper
else:
    import _Precision
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
    __swig_destroy__ = _Precision.delete_SwigPyIterator
    def __iter__(self):
        return self
SwigPyIterator.value = new_instancemethod(_Precision.SwigPyIterator_value, None, SwigPyIterator)
SwigPyIterator.incr = new_instancemethod(_Precision.SwigPyIterator_incr, None, SwigPyIterator)
SwigPyIterator.decr = new_instancemethod(_Precision.SwigPyIterator_decr, None, SwigPyIterator)
SwigPyIterator.distance = new_instancemethod(_Precision.SwigPyIterator_distance, None, SwigPyIterator)
SwigPyIterator.equal = new_instancemethod(_Precision.SwigPyIterator_equal, None, SwigPyIterator)
SwigPyIterator.copy = new_instancemethod(_Precision.SwigPyIterator_copy, None, SwigPyIterator)
SwigPyIterator.next = new_instancemethod(_Precision.SwigPyIterator_next, None, SwigPyIterator)
SwigPyIterator.__next__ = new_instancemethod(_Precision.SwigPyIterator___next__, None, SwigPyIterator)
SwigPyIterator.previous = new_instancemethod(_Precision.SwigPyIterator_previous, None, SwigPyIterator)
SwigPyIterator.advance = new_instancemethod(_Precision.SwigPyIterator_advance, None, SwigPyIterator)
SwigPyIterator.__eq__ = new_instancemethod(_Precision.SwigPyIterator___eq__, None, SwigPyIterator)
SwigPyIterator.__ne__ = new_instancemethod(_Precision.SwigPyIterator___ne__, None, SwigPyIterator)
SwigPyIterator.__iadd__ = new_instancemethod(_Precision.SwigPyIterator___iadd__, None, SwigPyIterator)
SwigPyIterator.__isub__ = new_instancemethod(_Precision.SwigPyIterator___isub__, None, SwigPyIterator)
SwigPyIterator.__add__ = new_instancemethod(_Precision.SwigPyIterator___add__, None, SwigPyIterator)
SwigPyIterator.__sub__ = new_instancemethod(_Precision.SwigPyIterator___sub__, None, SwigPyIterator)
SwigPyIterator_swigregister = _Precision.SwigPyIterator_swigregister
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
    return _Precision.process_exception(error, method_name, class_name)
process_exception = _Precision.process_exception

from six import with_metaclass
import warnings
from OCC.Wrapper.wrapper_utils import Proxy, deprecated

import OCC.Core.Standard
import OCC.Core.NCollection
class precision(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def Angular(*args) -> "Standard_Real":
        """
        * Returns the recommended precision value when checking the equality of two angles (given in radians). Standard_Real Angle1 = ... , Angle2 = ... ; If ( Abs( Angle2 - Angle1 ) < Precision::Angular() ) ... The tolerance of angular equality may be used to check the parallelism of two vectors : gp_Vec V1, V2 ; V1 = ... V2 = ... If ( V1.IsParallel (V2, Precision::Angular() ) ) ... The tolerance of angular equality is equal to 1.e-12. Note : The tolerance of angular equality can be used when working with scalar products or cross products since sines and angles are equivalent for small angles. Therefore, in order to check whether two unit vectors are perpendicular : gp_Dir D1, D2 ; D1 = ... D2 = ... you can use : If ( Abs( D1.D2 ) < Precision::Angular() ) ... (although the function IsNormal does exist).
        	:rtype: float
        """
        return _Precision.precision_Angular(*args)

    Angular = staticmethod(Angular)

    def Approximation(*args) -> "Standard_Real":
        """
        * Returns the precision value in real space, frequently used by approximation algorithms. This function provides an acceptable level of precision for an approximation process to define adjustment limits. The tolerance of approximation is designed to ensure an acceptable computation time when performing an approximation process. That is why the tolerance of approximation is greater than the tolerance of confusion. The tolerance of approximation is equal to : Precision::Confusion() * 10. (that is, 1.e-6). You may use a smaller tolerance in an approximation algorithm, but this option might be costly.
        	:rtype: float
        """
        return _Precision.precision_Approximation(*args)

    Approximation = staticmethod(Approximation)

    def Confusion(*args) -> "Standard_Real":
        """
        * Returns the recommended precision value when checking coincidence of two points in real space. The tolerance of confusion is used for testing a 3D distance : - Two points are considered to be coincident if their distance is smaller than the tolerance of confusion. gp_Pnt P1, P2 ; P1 = ... P2 = ... if ( P1.IsEqual ( P2 , Precision::Confusion() ) ) then ... - A vector is considered to be null if it has a null length : gp_Vec V ; V = ... if ( V.Magnitude() < Precision::Confusion() ) then ... The tolerance of confusion is equal to 1.e-7. The value of the tolerance of confusion is also used to define : - the tolerance of intersection, and - the tolerance of approximation. Note : As a rule, coordinate values in Cas.Cade are not dimensioned, so 1. represents one user unit, whatever value the unit may have : the millimeter, the meter, the inch, or any other unit. Let's say that Cas.Cade algorithms are written to be tuned essentially with mechanical design applications, on the basis of the millimeter. However, these algorithms may be used with any other unit but the tolerance criterion does no longer have the same signification. So pay particular attention to the type of your application, in relation with the impact of your unit on the precision criterion. - For example in mechanical design, if the unit is the millimeter, the tolerance of confusion corresponds to a distance of 1 / 10000 micron, which is rather difficult to measure. - However in other types of applications, such as cartography, where the kilometer is frequently used, the tolerance of confusion corresponds to a greater distance (1 / 10 millimeter). This distance becomes easily measurable, but only within a restricted space which contains some small objects of the complete scene.
        	:rtype: float
        """
        return _Precision.precision_Confusion(*args)

    Confusion = staticmethod(Confusion)

    def Infinite(*args) -> "Standard_Real":
        """
        * Returns a big number that can be considered as infinite. Use -Infinite() for a negative big number.
        	:rtype: float
        """
        return _Precision.precision_Infinite(*args)

    Infinite = staticmethod(Infinite)

    def Intersection(*args) -> "Standard_Real":
        """
        * Returns the precision value in real space, frequently used by intersection algorithms to decide that a solution is reached. This function provides an acceptable level of precision for an intersection process to define the adjustment limits. The tolerance of intersection is designed to ensure that a point computed by an iterative algorithm as the intersection between two curves is indeed on the intersection. It is obvious that two tangent curves are close to each other, on a large distance. An iterative algorithm of intersection may find points on these curves within the scope of the confusion tolerance, but still far from the true intersection point. In order to force the intersection algorithm to continue the iteration process until a correct point is found on the tangent objects, the tolerance of intersection must be smaller than the tolerance of confusion. On the other hand, the tolerance of intersection must be large enough to minimize the time required by the process to converge to a solution. The tolerance of intersection is equal to : Precision::Confusion() / 100. (that is, 1.e-9).
        	:rtype: float
        """
        return _Precision.precision_Intersection(*args)

    Intersection = staticmethod(Intersection)

    def IsInfinite(*args) -> "Standard_Boolean":
        """
        * Returns True if R may be considered as an infinite number. Currently Abs(R) > 1e100
        	:param R:
        	:type R: float
        	:rtype: bool
        """
        return _Precision.precision_IsInfinite(*args)

    IsInfinite = staticmethod(IsInfinite)

    def IsNegativeInfinite(*args) -> "Standard_Boolean":
        """
        * Returns True if R may be considered as a negative infinite number. Currently R < -1e100
        	:param R:
        	:type R: float
        	:rtype: bool
        """
        return _Precision.precision_IsNegativeInfinite(*args)

    IsNegativeInfinite = staticmethod(IsNegativeInfinite)

    def IsPositiveInfinite(*args) -> "Standard_Boolean":
        """
        * Returns True if R may be considered as a positive infinite number. Currently R > 1e100
        	:param R:
        	:type R: float
        	:rtype: bool
        """
        return _Precision.precision_IsPositiveInfinite(*args)

    IsPositiveInfinite = staticmethod(IsPositiveInfinite)

    def PApproximation(*args) -> "Standard_Real":
        """
        * Returns a precision value in parametric space, which may be used by approximation algorithms. The purpose of this function is to provide an acceptable level of precision in parametric space, for an approximation process to define the adjustment limits. The parametric tolerance of approximation is designed to give a mean value in relation with the dimension of the curve or the surface. It considers that a variation of parameter equal to 1. along a curve (or an isoparametric curve of a surface) generates a segment whose length is equal to 100. (default value), or T. The parametric tolerance of intersection is equal to : - Precision::Approximation() / 100., or Precision::Approximation() / T.
        	:param T:
        	:type T: float
        	:rtype: float
        * Used for Approximations in parametric space on a default curve. //! This is Precision::Parametric(Precision::Approximation())
        	:rtype: float
        """
        return _Precision.precision_PApproximation(*args)

    PApproximation = staticmethod(PApproximation)

    def PConfusion(*args) -> "Standard_Real":
        """
        * Returns a precision value in parametric space, which may be used : - to test the coincidence of two points in the real space, by using parameter values, or - to test the equality of two parameter values in a parametric space. The parametric tolerance of confusion is designed to give a mean value in relation with the dimension of the curve or the surface. It considers that a variation of parameter equal to 1. along a curve (or an isoparametric curve of a surface) generates a segment whose length is equal to 100. (default value), or T. The parametric tolerance of confusion is equal to : - Precision::Confusion() / 100., or Precision::Confusion() / T. The value of the parametric tolerance of confusion is also used to define : - the parametric tolerance of intersection, and - the parametric tolerance of approximation. Warning It is rather difficult to define a unique precision value in parametric space. - First consider a curve (c) ; if M is the point of parameter u and M' the point of parameter u+du on the curve, call 'parametric tangent' at point M, for the variation du of the parameter, the quantity : T(u,du)=MM'/du (where MM' represents the distance between the two points M and M', in the real space). - Consider the other curve resulting from a scaling transformation of (c) with a scale factor equal to 10. The 'parametric tangent' at the point of parameter u of this curve is ten times greater than the previous one. This shows that for two different curves, the distance between two points on the curve, resulting from the same variation of parameter du, may vary considerably. - Moreover, the variation of the parameter along the curve is generally not proportional to the curvilinear abscissa along the curve. So the distance between two points resulting from the same variation of parameter du, at two different points of a curve, may completely differ. - Moreover, the parameterization of a surface may generate two quite different 'parametric tangent' values in the u or in the v parametric direction. - Last, close to the poles of a sphere (the points which correspond to the values -Pi/2. and Pi/2. of the v parameter) the u parameter may change from 0 to 2.Pi without impacting on the resulting point. Therefore, take great care when adjusting a parametric tolerance to your own algorithm.
        	:param T:
        	:type T: float
        	:rtype: float
        * Used to test distances in parametric space on a default curve. //! This is Precision::Parametric(Precision::Confusion())
        	:rtype: float
        """
        return _Precision.precision_PConfusion(*args)

    PConfusion = staticmethod(PConfusion)

    def PIntersection(*args) -> "Standard_Real":
        """
        * Returns a precision value in parametric space, which may be used by intersection algorithms, to decide that a solution is reached. The purpose of this function is to provide an acceptable level of precision in parametric space, for an intersection process to define the adjustment limits. The parametric tolerance of intersection is designed to give a mean value in relation with the dimension of the curve or the surface. It considers that a variation of parameter equal to 1. along a curve (or an isoparametric curve of a surface) generates a segment whose length is equal to 100. (default value), or T. The parametric tolerance of intersection is equal to : - Precision::Intersection() / 100., or Precision::Intersection() / T.
        	:param T:
        	:type T: float
        	:rtype: float
        * Used for Intersections in parametric space on a default curve. //! This is Precision::Parametric(Precision::Intersection())
        	:rtype: float
        """
        return _Precision.precision_PIntersection(*args)

    PIntersection = staticmethod(PIntersection)

    def Parametric(*args) -> "Standard_Real":
        """
        * Convert a real space precision to a parametric space precision. <T> is the mean value of the length of the tangent of the curve or the surface. //! Value is P / T
        	:param P:
        	:type P: float
        	:param T:
        	:type T: float
        	:rtype: float
        * Convert a real space precision to a parametric space precision on a default curve. //! Value is Parametric(P,1.e+2)
        	:param P:
        	:type P: float
        	:rtype: float
        """
        return _Precision.precision_Parametric(*args)

    Parametric = staticmethod(Parametric)

    def SquareConfusion(*args) -> "Standard_Real":
        """
        * Returns square of Confusion. Created for speed and convenience.
        	:rtype: float
        """
        return _Precision.precision_SquareConfusion(*args)

    SquareConfusion = staticmethod(SquareConfusion)

    def SquarePConfusion(*args) -> "Standard_Real":
        """
        * Returns square of PConfusion. Created for speed and convenience.
        	:rtype: float
        """
        return _Precision.precision_SquarePConfusion(*args)

    SquarePConfusion = staticmethod(SquarePConfusion)

    __repr__ = _dumps_object


    def __init__(self):
        _Precision.precision_swiginit(self, _Precision.new_precision())
    __swig_destroy__ = _Precision.delete_precision
precision_swigregister = _Precision.precision_swigregister
precision_swigregister(precision)

def precision_Angular(*args) -> "Standard_Real":
    """
    * Returns the recommended precision value when checking the equality of two angles (given in radians). Standard_Real Angle1 = ... , Angle2 = ... ; If ( Abs( Angle2 - Angle1 ) < Precision::Angular() ) ... The tolerance of angular equality may be used to check the parallelism of two vectors : gp_Vec V1, V2 ; V1 = ... V2 = ... If ( V1.IsParallel (V2, Precision::Angular() ) ) ... The tolerance of angular equality is equal to 1.e-12. Note : The tolerance of angular equality can be used when working with scalar products or cross products since sines and angles are equivalent for small angles. Therefore, in order to check whether two unit vectors are perpendicular : gp_Dir D1, D2 ; D1 = ... D2 = ... you can use : If ( Abs( D1.D2 ) < Precision::Angular() ) ... (although the function IsNormal does exist).
    	:rtype: float
    """
    return _Precision.precision_Angular(*args)

def precision_Approximation(*args) -> "Standard_Real":
    """
    * Returns the precision value in real space, frequently used by approximation algorithms. This function provides an acceptable level of precision for an approximation process to define adjustment limits. The tolerance of approximation is designed to ensure an acceptable computation time when performing an approximation process. That is why the tolerance of approximation is greater than the tolerance of confusion. The tolerance of approximation is equal to : Precision::Confusion() * 10. (that is, 1.e-6). You may use a smaller tolerance in an approximation algorithm, but this option might be costly.
    	:rtype: float
    """
    return _Precision.precision_Approximation(*args)

def precision_Confusion(*args) -> "Standard_Real":
    """
    * Returns the recommended precision value when checking coincidence of two points in real space. The tolerance of confusion is used for testing a 3D distance : - Two points are considered to be coincident if their distance is smaller than the tolerance of confusion. gp_Pnt P1, P2 ; P1 = ... P2 = ... if ( P1.IsEqual ( P2 , Precision::Confusion() ) ) then ... - A vector is considered to be null if it has a null length : gp_Vec V ; V = ... if ( V.Magnitude() < Precision::Confusion() ) then ... The tolerance of confusion is equal to 1.e-7. The value of the tolerance of confusion is also used to define : - the tolerance of intersection, and - the tolerance of approximation. Note : As a rule, coordinate values in Cas.Cade are not dimensioned, so 1. represents one user unit, whatever value the unit may have : the millimeter, the meter, the inch, or any other unit. Let's say that Cas.Cade algorithms are written to be tuned essentially with mechanical design applications, on the basis of the millimeter. However, these algorithms may be used with any other unit but the tolerance criterion does no longer have the same signification. So pay particular attention to the type of your application, in relation with the impact of your unit on the precision criterion. - For example in mechanical design, if the unit is the millimeter, the tolerance of confusion corresponds to a distance of 1 / 10000 micron, which is rather difficult to measure. - However in other types of applications, such as cartography, where the kilometer is frequently used, the tolerance of confusion corresponds to a greater distance (1 / 10 millimeter). This distance becomes easily measurable, but only within a restricted space which contains some small objects of the complete scene.
    	:rtype: float
    """
    return _Precision.precision_Confusion(*args)

def precision_Infinite(*args) -> "Standard_Real":
    """
    * Returns a big number that can be considered as infinite. Use -Infinite() for a negative big number.
    	:rtype: float
    """
    return _Precision.precision_Infinite(*args)

def precision_Intersection(*args) -> "Standard_Real":
    """
    * Returns the precision value in real space, frequently used by intersection algorithms to decide that a solution is reached. This function provides an acceptable level of precision for an intersection process to define the adjustment limits. The tolerance of intersection is designed to ensure that a point computed by an iterative algorithm as the intersection between two curves is indeed on the intersection. It is obvious that two tangent curves are close to each other, on a large distance. An iterative algorithm of intersection may find points on these curves within the scope of the confusion tolerance, but still far from the true intersection point. In order to force the intersection algorithm to continue the iteration process until a correct point is found on the tangent objects, the tolerance of intersection must be smaller than the tolerance of confusion. On the other hand, the tolerance of intersection must be large enough to minimize the time required by the process to converge to a solution. The tolerance of intersection is equal to : Precision::Confusion() / 100. (that is, 1.e-9).
    	:rtype: float
    """
    return _Precision.precision_Intersection(*args)

def precision_IsInfinite(*args) -> "Standard_Boolean":
    """
    * Returns True if R may be considered as an infinite number. Currently Abs(R) > 1e100
    	:param R:
    	:type R: float
    	:rtype: bool
    """
    return _Precision.precision_IsInfinite(*args)

def precision_IsNegativeInfinite(*args) -> "Standard_Boolean":
    """
    * Returns True if R may be considered as a negative infinite number. Currently R < -1e100
    	:param R:
    	:type R: float
    	:rtype: bool
    """
    return _Precision.precision_IsNegativeInfinite(*args)

def precision_IsPositiveInfinite(*args) -> "Standard_Boolean":
    """
    * Returns True if R may be considered as a positive infinite number. Currently R > 1e100
    	:param R:
    	:type R: float
    	:rtype: bool
    """
    return _Precision.precision_IsPositiveInfinite(*args)

def precision_PApproximation(*args) -> "Standard_Real":
    """
    * Returns a precision value in parametric space, which may be used by approximation algorithms. The purpose of this function is to provide an acceptable level of precision in parametric space, for an approximation process to define the adjustment limits. The parametric tolerance of approximation is designed to give a mean value in relation with the dimension of the curve or the surface. It considers that a variation of parameter equal to 1. along a curve (or an isoparametric curve of a surface) generates a segment whose length is equal to 100. (default value), or T. The parametric tolerance of intersection is equal to : - Precision::Approximation() / 100., or Precision::Approximation() / T.
    	:param T:
    	:type T: float
    	:rtype: float
    * Used for Approximations in parametric space on a default curve. //! This is Precision::Parametric(Precision::Approximation())
    	:rtype: float
    """
    return _Precision.precision_PApproximation(*args)

def precision_PConfusion(*args) -> "Standard_Real":
    """
    * Returns a precision value in parametric space, which may be used : - to test the coincidence of two points in the real space, by using parameter values, or - to test the equality of two parameter values in a parametric space. The parametric tolerance of confusion is designed to give a mean value in relation with the dimension of the curve or the surface. It considers that a variation of parameter equal to 1. along a curve (or an isoparametric curve of a surface) generates a segment whose length is equal to 100. (default value), or T. The parametric tolerance of confusion is equal to : - Precision::Confusion() / 100., or Precision::Confusion() / T. The value of the parametric tolerance of confusion is also used to define : - the parametric tolerance of intersection, and - the parametric tolerance of approximation. Warning It is rather difficult to define a unique precision value in parametric space. - First consider a curve (c) ; if M is the point of parameter u and M' the point of parameter u+du on the curve, call 'parametric tangent' at point M, for the variation du of the parameter, the quantity : T(u,du)=MM'/du (where MM' represents the distance between the two points M and M', in the real space). - Consider the other curve resulting from a scaling transformation of (c) with a scale factor equal to 10. The 'parametric tangent' at the point of parameter u of this curve is ten times greater than the previous one. This shows that for two different curves, the distance between two points on the curve, resulting from the same variation of parameter du, may vary considerably. - Moreover, the variation of the parameter along the curve is generally not proportional to the curvilinear abscissa along the curve. So the distance between two points resulting from the same variation of parameter du, at two different points of a curve, may completely differ. - Moreover, the parameterization of a surface may generate two quite different 'parametric tangent' values in the u or in the v parametric direction. - Last, close to the poles of a sphere (the points which correspond to the values -Pi/2. and Pi/2. of the v parameter) the u parameter may change from 0 to 2.Pi without impacting on the resulting point. Therefore, take great care when adjusting a parametric tolerance to your own algorithm.
    	:param T:
    	:type T: float
    	:rtype: float
    * Used to test distances in parametric space on a default curve. //! This is Precision::Parametric(Precision::Confusion())
    	:rtype: float
    """
    return _Precision.precision_PConfusion(*args)

def precision_PIntersection(*args) -> "Standard_Real":
    """
    * Returns a precision value in parametric space, which may be used by intersection algorithms, to decide that a solution is reached. The purpose of this function is to provide an acceptable level of precision in parametric space, for an intersection process to define the adjustment limits. The parametric tolerance of intersection is designed to give a mean value in relation with the dimension of the curve or the surface. It considers that a variation of parameter equal to 1. along a curve (or an isoparametric curve of a surface) generates a segment whose length is equal to 100. (default value), or T. The parametric tolerance of intersection is equal to : - Precision::Intersection() / 100., or Precision::Intersection() / T.
    	:param T:
    	:type T: float
    	:rtype: float
    * Used for Intersections in parametric space on a default curve. //! This is Precision::Parametric(Precision::Intersection())
    	:rtype: float
    """
    return _Precision.precision_PIntersection(*args)

def precision_Parametric(*args) -> "Standard_Real":
    """
    * Convert a real space precision to a parametric space precision. <T> is the mean value of the length of the tangent of the curve or the surface. //! Value is P / T
    	:param P:
    	:type P: float
    	:param T:
    	:type T: float
    	:rtype: float
    * Convert a real space precision to a parametric space precision on a default curve. //! Value is Parametric(P,1.e+2)
    	:param P:
    	:type P: float
    	:rtype: float
    """
    return _Precision.precision_Parametric(*args)

def precision_SquareConfusion(*args) -> "Standard_Real":
    """
    * Returns square of Confusion. Created for speed and convenience.
    	:rtype: float
    """
    return _Precision.precision_SquareConfusion(*args)

def precision_SquarePConfusion(*args) -> "Standard_Real":
    """
    * Returns square of PConfusion. Created for speed and convenience.
    	:rtype: float
    """
    return _Precision.precision_SquarePConfusion(*args)



