# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.12
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

"""
BRepAdaptor module, see official documentation at
https://www.opencascade.com/doc/occt-7.4.0/refman/html/package_brepadaptor.html
"""


from sys import version_info as _swig_python_version_info
if _swig_python_version_info >= (3, 0, 0):
    new_instancemethod = lambda func, inst, cls: _BRepAdaptor.SWIG_PyInstanceMethod_New(func)
else:
    from new import instancemethod as new_instancemethod
if _swig_python_version_info >= (2, 7, 0):
    def swig_import_helper():
        import importlib
        pkg = __name__.rpartition('.')[0]
        mname = '.'.join((pkg, '_BRepAdaptor')).lstrip('.')
        try:
            return importlib.import_module(mname)
        except ImportError:
            return importlib.import_module('_BRepAdaptor')
    _BRepAdaptor = swig_import_helper()
    del swig_import_helper
elif _swig_python_version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_BRepAdaptor', [dirname(__file__)])
        except ImportError:
            import _BRepAdaptor
            return _BRepAdaptor
        try:
            _mod = imp.load_module('_BRepAdaptor', fp, pathname, description)
        finally:
            if fp is not None:
                fp.close()
        return _mod
    _BRepAdaptor = swig_import_helper()
    del swig_import_helper
else:
    import _BRepAdaptor
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
    __swig_destroy__ = _BRepAdaptor.delete_SwigPyIterator
    def __iter__(self):
        return self
SwigPyIterator.value = new_instancemethod(_BRepAdaptor.SwigPyIterator_value, None, SwigPyIterator)
SwigPyIterator.incr = new_instancemethod(_BRepAdaptor.SwigPyIterator_incr, None, SwigPyIterator)
SwigPyIterator.decr = new_instancemethod(_BRepAdaptor.SwigPyIterator_decr, None, SwigPyIterator)
SwigPyIterator.distance = new_instancemethod(_BRepAdaptor.SwigPyIterator_distance, None, SwigPyIterator)
SwigPyIterator.equal = new_instancemethod(_BRepAdaptor.SwigPyIterator_equal, None, SwigPyIterator)
SwigPyIterator.copy = new_instancemethod(_BRepAdaptor.SwigPyIterator_copy, None, SwigPyIterator)
SwigPyIterator.next = new_instancemethod(_BRepAdaptor.SwigPyIterator_next, None, SwigPyIterator)
SwigPyIterator.__next__ = new_instancemethod(_BRepAdaptor.SwigPyIterator___next__, None, SwigPyIterator)
SwigPyIterator.previous = new_instancemethod(_BRepAdaptor.SwigPyIterator_previous, None, SwigPyIterator)
SwigPyIterator.advance = new_instancemethod(_BRepAdaptor.SwigPyIterator_advance, None, SwigPyIterator)
SwigPyIterator.__eq__ = new_instancemethod(_BRepAdaptor.SwigPyIterator___eq__, None, SwigPyIterator)
SwigPyIterator.__ne__ = new_instancemethod(_BRepAdaptor.SwigPyIterator___ne__, None, SwigPyIterator)
SwigPyIterator.__iadd__ = new_instancemethod(_BRepAdaptor.SwigPyIterator___iadd__, None, SwigPyIterator)
SwigPyIterator.__isub__ = new_instancemethod(_BRepAdaptor.SwigPyIterator___isub__, None, SwigPyIterator)
SwigPyIterator.__add__ = new_instancemethod(_BRepAdaptor.SwigPyIterator___add__, None, SwigPyIterator)
SwigPyIterator.__sub__ = new_instancemethod(_BRepAdaptor.SwigPyIterator___sub__, None, SwigPyIterator)
SwigPyIterator_swigregister = _BRepAdaptor.SwigPyIterator_swigregister
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
    return _BRepAdaptor.process_exception(error, method_name, class_name)
process_exception = _BRepAdaptor.process_exception

from six import with_metaclass
import warnings
from OCC.Wrapper.wrapper_utils import Proxy, deprecated

import OCC.Core.Standard
import OCC.Core.NCollection
import OCC.Core.Adaptor3d
import OCC.Core.Geom
import OCC.Core.gp
import OCC.Core.GeomAbs
import OCC.Core.TColgp
import OCC.Core.TColStd
import OCC.Core.TCollection
import OCC.Core.TopAbs
import OCC.Core.Adaptor2d
import OCC.Core.Geom2d
import OCC.Core.math
import OCC.Core.Message
import OCC.Core.TopoDS
import OCC.Core.TopLoc
import OCC.Core.GeomAdaptor
import OCC.Core.Geom2dAdaptor

def Handle_BRepAdaptor_HCompCurve_Create() -> "opencascade::handle< BRepAdaptor_HCompCurve >":
    return _BRepAdaptor.Handle_BRepAdaptor_HCompCurve_Create()
Handle_BRepAdaptor_HCompCurve_Create = _BRepAdaptor.Handle_BRepAdaptor_HCompCurve_Create

def Handle_BRepAdaptor_HCompCurve_DownCast(t: 'opencascade::handle< Standard_Transient > const &') -> "opencascade::handle< BRepAdaptor_HCompCurve >":
    return _BRepAdaptor.Handle_BRepAdaptor_HCompCurve_DownCast(t)
Handle_BRepAdaptor_HCompCurve_DownCast = _BRepAdaptor.Handle_BRepAdaptor_HCompCurve_DownCast

def Handle_BRepAdaptor_HCompCurve_IsNull(t: 'opencascade::handle< BRepAdaptor_HCompCurve > const &') -> "bool":
    return _BRepAdaptor.Handle_BRepAdaptor_HCompCurve_IsNull(t)
Handle_BRepAdaptor_HCompCurve_IsNull = _BRepAdaptor.Handle_BRepAdaptor_HCompCurve_IsNull

def Handle_BRepAdaptor_HCurve_Create() -> "opencascade::handle< BRepAdaptor_HCurve >":
    return _BRepAdaptor.Handle_BRepAdaptor_HCurve_Create()
Handle_BRepAdaptor_HCurve_Create = _BRepAdaptor.Handle_BRepAdaptor_HCurve_Create

def Handle_BRepAdaptor_HCurve_DownCast(t: 'opencascade::handle< Standard_Transient > const &') -> "opencascade::handle< BRepAdaptor_HCurve >":
    return _BRepAdaptor.Handle_BRepAdaptor_HCurve_DownCast(t)
Handle_BRepAdaptor_HCurve_DownCast = _BRepAdaptor.Handle_BRepAdaptor_HCurve_DownCast

def Handle_BRepAdaptor_HCurve_IsNull(t: 'opencascade::handle< BRepAdaptor_HCurve > const &') -> "bool":
    return _BRepAdaptor.Handle_BRepAdaptor_HCurve_IsNull(t)
Handle_BRepAdaptor_HCurve_IsNull = _BRepAdaptor.Handle_BRepAdaptor_HCurve_IsNull

def Handle_BRepAdaptor_HCurve2d_Create() -> "opencascade::handle< BRepAdaptor_HCurve2d >":
    return _BRepAdaptor.Handle_BRepAdaptor_HCurve2d_Create()
Handle_BRepAdaptor_HCurve2d_Create = _BRepAdaptor.Handle_BRepAdaptor_HCurve2d_Create

def Handle_BRepAdaptor_HCurve2d_DownCast(t: 'opencascade::handle< Standard_Transient > const &') -> "opencascade::handle< BRepAdaptor_HCurve2d >":
    return _BRepAdaptor.Handle_BRepAdaptor_HCurve2d_DownCast(t)
Handle_BRepAdaptor_HCurve2d_DownCast = _BRepAdaptor.Handle_BRepAdaptor_HCurve2d_DownCast

def Handle_BRepAdaptor_HCurve2d_IsNull(t: 'opencascade::handle< BRepAdaptor_HCurve2d > const &') -> "bool":
    return _BRepAdaptor.Handle_BRepAdaptor_HCurve2d_IsNull(t)
Handle_BRepAdaptor_HCurve2d_IsNull = _BRepAdaptor.Handle_BRepAdaptor_HCurve2d_IsNull

def Handle_BRepAdaptor_HSurface_Create() -> "opencascade::handle< BRepAdaptor_HSurface >":
    return _BRepAdaptor.Handle_BRepAdaptor_HSurface_Create()
Handle_BRepAdaptor_HSurface_Create = _BRepAdaptor.Handle_BRepAdaptor_HSurface_Create

def Handle_BRepAdaptor_HSurface_DownCast(t: 'opencascade::handle< Standard_Transient > const &') -> "opencascade::handle< BRepAdaptor_HSurface >":
    return _BRepAdaptor.Handle_BRepAdaptor_HSurface_DownCast(t)
Handle_BRepAdaptor_HSurface_DownCast = _BRepAdaptor.Handle_BRepAdaptor_HSurface_DownCast

def Handle_BRepAdaptor_HSurface_IsNull(t: 'opencascade::handle< BRepAdaptor_HSurface > const &') -> "bool":
    return _BRepAdaptor.Handle_BRepAdaptor_HSurface_IsNull(t)
Handle_BRepAdaptor_HSurface_IsNull = _BRepAdaptor.Handle_BRepAdaptor_HSurface_IsNull

def Handle_BRepAdaptor_HArray1OfCurve_Create() -> "opencascade::handle< BRepAdaptor_HArray1OfCurve >":
    return _BRepAdaptor.Handle_BRepAdaptor_HArray1OfCurve_Create()
Handle_BRepAdaptor_HArray1OfCurve_Create = _BRepAdaptor.Handle_BRepAdaptor_HArray1OfCurve_Create

def Handle_BRepAdaptor_HArray1OfCurve_DownCast(t: 'opencascade::handle< Standard_Transient > const &') -> "opencascade::handle< BRepAdaptor_HArray1OfCurve >":
    return _BRepAdaptor.Handle_BRepAdaptor_HArray1OfCurve_DownCast(t)
Handle_BRepAdaptor_HArray1OfCurve_DownCast = _BRepAdaptor.Handle_BRepAdaptor_HArray1OfCurve_DownCast

def Handle_BRepAdaptor_HArray1OfCurve_IsNull(t: 'opencascade::handle< BRepAdaptor_HArray1OfCurve > const &') -> "bool":
    return _BRepAdaptor.Handle_BRepAdaptor_HArray1OfCurve_IsNull(t)
Handle_BRepAdaptor_HArray1OfCurve_IsNull = _BRepAdaptor.Handle_BRepAdaptor_HArray1OfCurve_IsNull
class BRepAdaptor_Array1OfCurve(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def __init__(self, *args):
        _BRepAdaptor.BRepAdaptor_Array1OfCurve_swiginit(self, _BRepAdaptor.new_BRepAdaptor_Array1OfCurve(*args))
    __swig_destroy__ = _BRepAdaptor.delete_BRepAdaptor_Array1OfCurve

    def __getitem__(self, index):
        if index + self.Lower() > self.Upper():
            raise IndexError("index out of range")
        else:
            return self.Value(index + self.Lower())

    def __setitem__(self, index, value):
        if index + self.Lower() > self.Upper():
            raise IndexError("index out of range")
        else:
            self.SetValue(index + self.Lower(), value)

    def __len__(self):
        return self.Length()

    def __iter__(self):
        self.low = self.Lower()
        self.up = self.Upper()
        self.current = self.Lower() - 1
        return self

    def next(self):
        if self.current >= self.Upper():
            raise StopIteration
        else:
            self.current += 1
        return self.Value(self.current)

    __next__ = next

BRepAdaptor_Array1OfCurve.begin = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_begin, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.end = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_end, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.cbegin = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_cbegin, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.cend = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_cend, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.Init = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_Init, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.Size = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_Size, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.Length = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_Length, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.IsEmpty = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_IsEmpty, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.Lower = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_Lower, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.Upper = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_Upper, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.IsDeletable = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_IsDeletable, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.IsAllocated = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_IsAllocated, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.Assign = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_Assign, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.Move = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_Move, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.Set = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_Set, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.First = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_First, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.ChangeFirst = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_ChangeFirst, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.Last = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_Last, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.ChangeLast = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_ChangeLast, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.Value = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_Value, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.ChangeValue = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_ChangeValue, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.__call__ = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve___call__, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.SetValue = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_SetValue, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve.Resize = new_instancemethod(_BRepAdaptor.BRepAdaptor_Array1OfCurve_Resize, None, BRepAdaptor_Array1OfCurve)
BRepAdaptor_Array1OfCurve_swigregister = _BRepAdaptor.BRepAdaptor_Array1OfCurve_swigregister
BRepAdaptor_Array1OfCurve_swigregister(BRepAdaptor_Array1OfCurve)

class BRepAdaptor_CompCurve(OCC.Core.Adaptor3d.Adaptor3d_Curve):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def __init__(self, *args):
        """
        * Creates an undefined Curve with no Wire loaded.
        	:rtype: None
        :param W:
        	:type W: TopoDS_Wire
        	:param KnotByCurvilinearAbcissa: default value is Standard_False
        	:type KnotByCurvilinearAbcissa: bool
        	:rtype: None
        * Creates a Curve to acces to the geometry of edge <W>.
        	:param W:
        	:type W: TopoDS_Wire
        	:param KnotByCurvilinearAbcissa:
        	:type KnotByCurvilinearAbcissa: bool
        	:param First:
        	:type First: float
        	:param Last:
        	:type Last: float
        	:param Tol:
        	:type Tol: float
        	:rtype: None
        """
        _BRepAdaptor.BRepAdaptor_CompCurve_swiginit(self, _BRepAdaptor.new_BRepAdaptor_CompCurve(*args))

    def Edge(self, *args) -> "void":
        """
        * returns an edge and one parameter on them corresponding to the parameter U.
        	:param U:
        	:type U: float
        	:param E:
        	:type E: TopoDS_Edge
        	:param UonE:
        	:type UonE: float
        	:rtype: None
        """
        return _BRepAdaptor.BRepAdaptor_CompCurve_Edge(self, *args)


    def Initialize(self, *args) -> "void":
        """
        * Sets the wire <W>.
        	:param W:
        	:type W: TopoDS_Wire
        	:param KnotByCurvilinearAbcissa:
        	:type KnotByCurvilinearAbcissa: bool
        	:rtype: None
        * Sets wire <W> and trimmed parameter.
        	:param W:
        	:type W: TopoDS_Wire
        	:param KnotByCurvilinearAbcissa:
        	:type KnotByCurvilinearAbcissa: bool
        	:param First:
        	:type First: float
        	:param Last:
        	:type Last: float
        	:param Tol:
        	:type Tol: float
        	:rtype: None
        """
        return _BRepAdaptor.BRepAdaptor_CompCurve_Initialize(self, *args)


    def Wire(self, *args) -> "TopoDS_Wire const":
        """
        * Returns the wire.
        	:rtype: TopoDS_Wire
        """
        return _BRepAdaptor.BRepAdaptor_CompCurve_Wire(self, *args)


    __repr__ = _dumps_object

    __swig_destroy__ = _BRepAdaptor.delete_BRepAdaptor_CompCurve
BRepAdaptor_CompCurve.Edge = new_instancemethod(_BRepAdaptor.BRepAdaptor_CompCurve_Edge, None, BRepAdaptor_CompCurve)
BRepAdaptor_CompCurve.Initialize = new_instancemethod(_BRepAdaptor.BRepAdaptor_CompCurve_Initialize, None, BRepAdaptor_CompCurve)
BRepAdaptor_CompCurve.Wire = new_instancemethod(_BRepAdaptor.BRepAdaptor_CompCurve_Wire, None, BRepAdaptor_CompCurve)
BRepAdaptor_CompCurve_swigregister = _BRepAdaptor.BRepAdaptor_CompCurve_swigregister
BRepAdaptor_CompCurve_swigregister(BRepAdaptor_CompCurve)

class BRepAdaptor_Curve(OCC.Core.Adaptor3d.Adaptor3d_Curve):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def __init__(self, *args):
        """
        * Creates an undefined Curve with no Edge loaded.
        	:rtype: None
        * Creates a Curve to acces to the geometry of edge <E>.
        	:param E:
        	:type E: TopoDS_Edge
        	:rtype: None
        * Creates a Curve to acces to the geometry of edge <E>. The geometry will be computed using the parametric curve of <E> on the face <F>. An Error is raised if the edge does not have a pcurve on the face.
        	:param E:
        	:type E: TopoDS_Edge
        	:param F:
        	:type F: TopoDS_Face
        	:rtype: None
        """
        _BRepAdaptor.BRepAdaptor_Curve_swiginit(self, _BRepAdaptor.new_BRepAdaptor_Curve(*args))

    def Curve(self, *args) -> "GeomAdaptor_Curve const &":
        """
        * Returns the Curve of the edge.
        	:rtype: GeomAdaptor_Curve
        """
        return _BRepAdaptor.BRepAdaptor_Curve_Curve(self, *args)


    def CurveOnSurface(self, *args) -> "Adaptor3d_CurveOnSurface const &":
        """
        * Returns the CurveOnSurface of the edge.
        	:rtype: Adaptor3d_CurveOnSurface
        """
        return _BRepAdaptor.BRepAdaptor_Curve_CurveOnSurface(self, *args)


    def Edge(self, *args) -> "TopoDS_Edge const":
        """
        * Returns the edge.
        	:rtype: TopoDS_Edge
        """
        return _BRepAdaptor.BRepAdaptor_Curve_Edge(self, *args)


    def Initialize(self, *args) -> "void":
        """
        * Sets the Curve <self> to acces to the geometry of edge <E>.
        	:param E:
        	:type E: TopoDS_Edge
        	:rtype: None
        * Sets the Curve <self> to acces to the geometry of edge <E>. The geometry will be computed using the parametric curve of <E> on the face <F>. An Error is raised if the edge does not have a pcurve on the face.
        	:param E:
        	:type E: TopoDS_Edge
        	:param F:
        	:type F: TopoDS_Face
        	:rtype: None
        """
        return _BRepAdaptor.BRepAdaptor_Curve_Initialize(self, *args)


    def Is3DCurve(self, *args) -> "Standard_Boolean":
        """
        * Returns True if the edge geometry is computed from a 3D curve.
        	:rtype: bool
        """
        return _BRepAdaptor.BRepAdaptor_Curve_Is3DCurve(self, *args)


    def IsCurveOnSurface(self, *args) -> "Standard_Boolean":
        """
        * Returns True if the edge geometry is computed from a pcurve on a surface.
        	:rtype: bool
        """
        return _BRepAdaptor.BRepAdaptor_Curve_IsCurveOnSurface(self, *args)


    def Reset(self, *args) -> "void":
        """
        * Reset currently loaded curve (undone Load()).
        	:rtype: None
        """
        return _BRepAdaptor.BRepAdaptor_Curve_Reset(self, *args)


    def Tolerance(self, *args) -> "Standard_Real":
        """
        * Returns the edge tolerance.
        	:rtype: float
        """
        return _BRepAdaptor.BRepAdaptor_Curve_Tolerance(self, *args)


    def Trsf(self, *args) -> "gp_Trsf const":
        """
        * Returns the coordinate system of the curve.
        	:rtype: gp_Trsf
        """
        return _BRepAdaptor.BRepAdaptor_Curve_Trsf(self, *args)


    __repr__ = _dumps_object

    __swig_destroy__ = _BRepAdaptor.delete_BRepAdaptor_Curve
BRepAdaptor_Curve.Curve = new_instancemethod(_BRepAdaptor.BRepAdaptor_Curve_Curve, None, BRepAdaptor_Curve)
BRepAdaptor_Curve.CurveOnSurface = new_instancemethod(_BRepAdaptor.BRepAdaptor_Curve_CurveOnSurface, None, BRepAdaptor_Curve)
BRepAdaptor_Curve.Edge = new_instancemethod(_BRepAdaptor.BRepAdaptor_Curve_Edge, None, BRepAdaptor_Curve)
BRepAdaptor_Curve.Initialize = new_instancemethod(_BRepAdaptor.BRepAdaptor_Curve_Initialize, None, BRepAdaptor_Curve)
BRepAdaptor_Curve.Is3DCurve = new_instancemethod(_BRepAdaptor.BRepAdaptor_Curve_Is3DCurve, None, BRepAdaptor_Curve)
BRepAdaptor_Curve.IsCurveOnSurface = new_instancemethod(_BRepAdaptor.BRepAdaptor_Curve_IsCurveOnSurface, None, BRepAdaptor_Curve)
BRepAdaptor_Curve.Reset = new_instancemethod(_BRepAdaptor.BRepAdaptor_Curve_Reset, None, BRepAdaptor_Curve)
BRepAdaptor_Curve.Tolerance = new_instancemethod(_BRepAdaptor.BRepAdaptor_Curve_Tolerance, None, BRepAdaptor_Curve)
BRepAdaptor_Curve.Trsf = new_instancemethod(_BRepAdaptor.BRepAdaptor_Curve_Trsf, None, BRepAdaptor_Curve)
BRepAdaptor_Curve_swigregister = _BRepAdaptor.BRepAdaptor_Curve_swigregister
BRepAdaptor_Curve_swigregister(BRepAdaptor_Curve)

class BRepAdaptor_Curve2d(OCC.Core.Geom2dAdaptor.Geom2dAdaptor_Curve):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def __init__(self, *args):
        """
        * Creates an uninitialized curve2d.
        	:rtype: None
        * Creates with the pcurve of <E> on <F>.
        	:param E:
        	:type E: TopoDS_Edge
        	:param F:
        	:type F: TopoDS_Face
        	:rtype: None
        """
        _BRepAdaptor.BRepAdaptor_Curve2d_swiginit(self, _BRepAdaptor.new_BRepAdaptor_Curve2d(*args))

    def Edge(self, *args) -> "TopoDS_Edge const":
        """
        * Returns the Edge.
        	:rtype: TopoDS_Edge
        """
        return _BRepAdaptor.BRepAdaptor_Curve2d_Edge(self, *args)


    def Face(self, *args) -> "TopoDS_Face const":
        """
        * Returns the Face.
        	:rtype: TopoDS_Face
        """
        return _BRepAdaptor.BRepAdaptor_Curve2d_Face(self, *args)


    def Initialize(self, *args) -> "void":
        """
        * Initialize with the pcurve of <E> on <F>.
        	:param E:
        	:type E: TopoDS_Edge
        	:param F:
        	:type F: TopoDS_Face
        	:rtype: None
        """
        return _BRepAdaptor.BRepAdaptor_Curve2d_Initialize(self, *args)


    __repr__ = _dumps_object

    __swig_destroy__ = _BRepAdaptor.delete_BRepAdaptor_Curve2d
BRepAdaptor_Curve2d.Edge = new_instancemethod(_BRepAdaptor.BRepAdaptor_Curve2d_Edge, None, BRepAdaptor_Curve2d)
BRepAdaptor_Curve2d.Face = new_instancemethod(_BRepAdaptor.BRepAdaptor_Curve2d_Face, None, BRepAdaptor_Curve2d)
BRepAdaptor_Curve2d.Initialize = new_instancemethod(_BRepAdaptor.BRepAdaptor_Curve2d_Initialize, None, BRepAdaptor_Curve2d)
BRepAdaptor_Curve2d_swigregister = _BRepAdaptor.BRepAdaptor_Curve2d_swigregister
BRepAdaptor_Curve2d_swigregister(BRepAdaptor_Curve2d)

class BRepAdaptor_HCompCurve(OCC.Core.Adaptor3d.Adaptor3d_HCurve):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def __init__(self, *args):
        """
        * Creates an empty GenHCurve.
        	:rtype: None
        * Creates a GenHCurve from a Curve
        	:param C:
        	:type C: BRepAdaptor_CompCurve
        	:rtype: None
        """
        _BRepAdaptor.BRepAdaptor_HCompCurve_swiginit(self, _BRepAdaptor.new_BRepAdaptor_HCompCurve(*args))

    def ChangeCurve(self, *args) -> "BRepAdaptor_CompCurve &":
        """
        * Returns the curve used to create the GenHCurve.
        	:rtype: BRepAdaptor_CompCurve
        """
        return _BRepAdaptor.BRepAdaptor_HCompCurve_ChangeCurve(self, *args)


    def Set(self, *args) -> "void":
        """
        * Sets the field of the GenHCurve.
        	:param C:
        	:type C: BRepAdaptor_CompCurve
        	:rtype: None
        """
        return _BRepAdaptor.BRepAdaptor_HCompCurve_Set(self, *args)



    @staticmethod
    def DownCast(t):
      return Handle_BRepAdaptor_HCompCurve_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _BRepAdaptor.delete_BRepAdaptor_HCompCurve
BRepAdaptor_HCompCurve.ChangeCurve = new_instancemethod(_BRepAdaptor.BRepAdaptor_HCompCurve_ChangeCurve, None, BRepAdaptor_HCompCurve)
BRepAdaptor_HCompCurve.Set = new_instancemethod(_BRepAdaptor.BRepAdaptor_HCompCurve_Set, None, BRepAdaptor_HCompCurve)
BRepAdaptor_HCompCurve_swigregister = _BRepAdaptor.BRepAdaptor_HCompCurve_swigregister
BRepAdaptor_HCompCurve_swigregister(BRepAdaptor_HCompCurve)

class BRepAdaptor_HCurve(OCC.Core.Adaptor3d.Adaptor3d_HCurve):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def __init__(self, *args):
        """
        * Creates an empty GenHCurve.
        	:rtype: None
        * Creates a GenHCurve from a Curve
        	:param C:
        	:type C: BRepAdaptor_Curve
        	:rtype: None
        """
        _BRepAdaptor.BRepAdaptor_HCurve_swiginit(self, _BRepAdaptor.new_BRepAdaptor_HCurve(*args))

    def ChangeCurve(self, *args) -> "BRepAdaptor_Curve &":
        """
        * Returns the curve used to create the GenHCurve.
        	:rtype: BRepAdaptor_Curve
        """
        return _BRepAdaptor.BRepAdaptor_HCurve_ChangeCurve(self, *args)


    def Set(self, *args) -> "void":
        """
        * Sets the field of the GenHCurve.
        	:param C:
        	:type C: BRepAdaptor_Curve
        	:rtype: None
        """
        return _BRepAdaptor.BRepAdaptor_HCurve_Set(self, *args)



    @staticmethod
    def DownCast(t):
      return Handle_BRepAdaptor_HCurve_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _BRepAdaptor.delete_BRepAdaptor_HCurve
BRepAdaptor_HCurve.ChangeCurve = new_instancemethod(_BRepAdaptor.BRepAdaptor_HCurve_ChangeCurve, None, BRepAdaptor_HCurve)
BRepAdaptor_HCurve.Set = new_instancemethod(_BRepAdaptor.BRepAdaptor_HCurve_Set, None, BRepAdaptor_HCurve)
BRepAdaptor_HCurve_swigregister = _BRepAdaptor.BRepAdaptor_HCurve_swigregister
BRepAdaptor_HCurve_swigregister(BRepAdaptor_HCurve)

class BRepAdaptor_HCurve2d(OCC.Core.Adaptor2d.Adaptor2d_HCurve2d):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def __init__(self, *args):
        """
        * Creates an empty GenHCurve2d.
        	:rtype: None
        * Creates a GenHCurve2d from a Curve
        	:param C:
        	:type C: BRepAdaptor_Curve2d
        	:rtype: None
        """
        _BRepAdaptor.BRepAdaptor_HCurve2d_swiginit(self, _BRepAdaptor.new_BRepAdaptor_HCurve2d(*args))

    def ChangeCurve2d(self, *args) -> "BRepAdaptor_Curve2d &":
        """
        * Returns the curve used to create the GenHCurve.
        	:rtype: BRepAdaptor_Curve2d
        """
        return _BRepAdaptor.BRepAdaptor_HCurve2d_ChangeCurve2d(self, *args)


    def Set(self, *args) -> "void":
        """
        * Sets the field of the GenHCurve2d.
        	:param C:
        	:type C: BRepAdaptor_Curve2d
        	:rtype: None
        """
        return _BRepAdaptor.BRepAdaptor_HCurve2d_Set(self, *args)



    @staticmethod
    def DownCast(t):
      return Handle_BRepAdaptor_HCurve2d_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _BRepAdaptor.delete_BRepAdaptor_HCurve2d
BRepAdaptor_HCurve2d.ChangeCurve2d = new_instancemethod(_BRepAdaptor.BRepAdaptor_HCurve2d_ChangeCurve2d, None, BRepAdaptor_HCurve2d)
BRepAdaptor_HCurve2d.Set = new_instancemethod(_BRepAdaptor.BRepAdaptor_HCurve2d_Set, None, BRepAdaptor_HCurve2d)
BRepAdaptor_HCurve2d_swigregister = _BRepAdaptor.BRepAdaptor_HCurve2d_swigregister
BRepAdaptor_HCurve2d_swigregister(BRepAdaptor_HCurve2d)

class BRepAdaptor_HSurface(OCC.Core.Adaptor3d.Adaptor3d_HSurface):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def __init__(self, *args):
        """
        * Creates an empty GenHSurface.
        	:rtype: None
        * Creates a GenHSurface from a Surface.
        	:param S:
        	:type S: BRepAdaptor_Surface
        	:rtype: None
        """
        _BRepAdaptor.BRepAdaptor_HSurface_swiginit(self, _BRepAdaptor.new_BRepAdaptor_HSurface(*args))

    def ChangeSurface(self, *args) -> "BRepAdaptor_Surface &":
        """
        * Returns the surface used to create the GenHSurface.
        	:rtype: BRepAdaptor_Surface
        """
        return _BRepAdaptor.BRepAdaptor_HSurface_ChangeSurface(self, *args)


    def Set(self, *args) -> "void":
        """
        * Sets the field of the GenHSurface.
        	:param S:
        	:type S: BRepAdaptor_Surface
        	:rtype: None
        """
        return _BRepAdaptor.BRepAdaptor_HSurface_Set(self, *args)



    @staticmethod
    def DownCast(t):
      return Handle_BRepAdaptor_HSurface_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _BRepAdaptor.delete_BRepAdaptor_HSurface
BRepAdaptor_HSurface.ChangeSurface = new_instancemethod(_BRepAdaptor.BRepAdaptor_HSurface_ChangeSurface, None, BRepAdaptor_HSurface)
BRepAdaptor_HSurface.Set = new_instancemethod(_BRepAdaptor.BRepAdaptor_HSurface_Set, None, BRepAdaptor_HSurface)
BRepAdaptor_HSurface_swigregister = _BRepAdaptor.BRepAdaptor_HSurface_swigregister
BRepAdaptor_HSurface_swigregister(BRepAdaptor_HSurface)

class BRepAdaptor_Surface(OCC.Core.Adaptor3d.Adaptor3d_Surface):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def __init__(self, *args):
        """
        * Creates an undefined surface with no face loaded.
        	:rtype: None
        * Creates a surface to access the geometry of <F>. If <Restriction> is true the parameter range is the parameter range in the UV space of the restriction.
        	:param F:
        	:type F: TopoDS_Face
        	:param R: default value is Standard_True
        	:type R: bool
        	:rtype: None
        """
        _BRepAdaptor.BRepAdaptor_Surface_swiginit(self, _BRepAdaptor.new_BRepAdaptor_Surface(*args))

    def ChangeSurface(self, *args) -> "GeomAdaptor_Surface &":
        """
        * Returns the surface.
        	:rtype: GeomAdaptor_Surface
        """
        return _BRepAdaptor.BRepAdaptor_Surface_ChangeSurface(self, *args)


    def Face(self, *args) -> "TopoDS_Face const":
        """
        * Returns the face.
        	:rtype: TopoDS_Face
        """
        return _BRepAdaptor.BRepAdaptor_Surface_Face(self, *args)


    def Initialize(self, *args) -> "void":
        """
        * Sets the surface to the geometry of <F>.
        	:param F:
        	:type F: TopoDS_Face
        	:param Restriction: default value is Standard_True
        	:type Restriction: bool
        	:rtype: None
        """
        return _BRepAdaptor.BRepAdaptor_Surface_Initialize(self, *args)


    def Surface(self, *args) -> "GeomAdaptor_Surface const &":
        """
        * Returns the surface.
        	:rtype: GeomAdaptor_Surface
        """
        return _BRepAdaptor.BRepAdaptor_Surface_Surface(self, *args)


    def Tolerance(self, *args) -> "Standard_Real":
        """
        * Returns the face tolerance.
        	:rtype: float
        """
        return _BRepAdaptor.BRepAdaptor_Surface_Tolerance(self, *args)


    def Trsf(self, *args) -> "gp_Trsf const":
        """
        * Returns the surface coordinate system.
        	:rtype: gp_Trsf
        """
        return _BRepAdaptor.BRepAdaptor_Surface_Trsf(self, *args)


    __repr__ = _dumps_object

    __swig_destroy__ = _BRepAdaptor.delete_BRepAdaptor_Surface
BRepAdaptor_Surface.ChangeSurface = new_instancemethod(_BRepAdaptor.BRepAdaptor_Surface_ChangeSurface, None, BRepAdaptor_Surface)
BRepAdaptor_Surface.Face = new_instancemethod(_BRepAdaptor.BRepAdaptor_Surface_Face, None, BRepAdaptor_Surface)
BRepAdaptor_Surface.Initialize = new_instancemethod(_BRepAdaptor.BRepAdaptor_Surface_Initialize, None, BRepAdaptor_Surface)
BRepAdaptor_Surface.Surface = new_instancemethod(_BRepAdaptor.BRepAdaptor_Surface_Surface, None, BRepAdaptor_Surface)
BRepAdaptor_Surface.Tolerance = new_instancemethod(_BRepAdaptor.BRepAdaptor_Surface_Tolerance, None, BRepAdaptor_Surface)
BRepAdaptor_Surface.Trsf = new_instancemethod(_BRepAdaptor.BRepAdaptor_Surface_Trsf, None, BRepAdaptor_Surface)
BRepAdaptor_Surface_swigregister = _BRepAdaptor.BRepAdaptor_Surface_swigregister
BRepAdaptor_Surface_swigregister(BRepAdaptor_Surface)

class BRepAdaptor_HArray1OfCurve(BRepAdaptor_Array1OfCurve, OCC.Core.Standard.Standard_Transient):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def __init__(self, *args):
        _BRepAdaptor.BRepAdaptor_HArray1OfCurve_swiginit(self, _BRepAdaptor.new_BRepAdaptor_HArray1OfCurve(*args))


    @staticmethod
    def DownCast(t):
      return Handle_BRepAdaptor_HArray1OfCurve_DownCast(t)

    __swig_destroy__ = _BRepAdaptor.delete_BRepAdaptor_HArray1OfCurve
BRepAdaptor_HArray1OfCurve.Array1 = new_instancemethod(_BRepAdaptor.BRepAdaptor_HArray1OfCurve_Array1, None, BRepAdaptor_HArray1OfCurve)
BRepAdaptor_HArray1OfCurve.ChangeArray1 = new_instancemethod(_BRepAdaptor.BRepAdaptor_HArray1OfCurve_ChangeArray1, None, BRepAdaptor_HArray1OfCurve)
BRepAdaptor_HArray1OfCurve_swigregister = _BRepAdaptor.BRepAdaptor_HArray1OfCurve_swigregister
BRepAdaptor_HArray1OfCurve_swigregister(BRepAdaptor_HArray1OfCurve)


