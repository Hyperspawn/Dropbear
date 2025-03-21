# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.12
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

"""
BRepProj module, see official documentation at
https://www.opencascade.com/doc/occt-7.4.0/refman/html/package_brepproj.html
"""


from sys import version_info as _swig_python_version_info
if _swig_python_version_info >= (3, 0, 0):
    new_instancemethod = lambda func, inst, cls: _BRepProj.SWIG_PyInstanceMethod_New(func)
else:
    from new import instancemethod as new_instancemethod
if _swig_python_version_info >= (2, 7, 0):
    def swig_import_helper():
        import importlib
        pkg = __name__.rpartition('.')[0]
        mname = '.'.join((pkg, '_BRepProj')).lstrip('.')
        try:
            return importlib.import_module(mname)
        except ImportError:
            return importlib.import_module('_BRepProj')
    _BRepProj = swig_import_helper()
    del swig_import_helper
elif _swig_python_version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_BRepProj', [dirname(__file__)])
        except ImportError:
            import _BRepProj
            return _BRepProj
        try:
            _mod = imp.load_module('_BRepProj', fp, pathname, description)
        finally:
            if fp is not None:
                fp.close()
        return _mod
    _BRepProj = swig_import_helper()
    del swig_import_helper
else:
    import _BRepProj
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
    __swig_destroy__ = _BRepProj.delete_SwigPyIterator
    def __iter__(self):
        return self
SwigPyIterator.value = new_instancemethod(_BRepProj.SwigPyIterator_value, None, SwigPyIterator)
SwigPyIterator.incr = new_instancemethod(_BRepProj.SwigPyIterator_incr, None, SwigPyIterator)
SwigPyIterator.decr = new_instancemethod(_BRepProj.SwigPyIterator_decr, None, SwigPyIterator)
SwigPyIterator.distance = new_instancemethod(_BRepProj.SwigPyIterator_distance, None, SwigPyIterator)
SwigPyIterator.equal = new_instancemethod(_BRepProj.SwigPyIterator_equal, None, SwigPyIterator)
SwigPyIterator.copy = new_instancemethod(_BRepProj.SwigPyIterator_copy, None, SwigPyIterator)
SwigPyIterator.next = new_instancemethod(_BRepProj.SwigPyIterator_next, None, SwigPyIterator)
SwigPyIterator.__next__ = new_instancemethod(_BRepProj.SwigPyIterator___next__, None, SwigPyIterator)
SwigPyIterator.previous = new_instancemethod(_BRepProj.SwigPyIterator_previous, None, SwigPyIterator)
SwigPyIterator.advance = new_instancemethod(_BRepProj.SwigPyIterator_advance, None, SwigPyIterator)
SwigPyIterator.__eq__ = new_instancemethod(_BRepProj.SwigPyIterator___eq__, None, SwigPyIterator)
SwigPyIterator.__ne__ = new_instancemethod(_BRepProj.SwigPyIterator___ne__, None, SwigPyIterator)
SwigPyIterator.__iadd__ = new_instancemethod(_BRepProj.SwigPyIterator___iadd__, None, SwigPyIterator)
SwigPyIterator.__isub__ = new_instancemethod(_BRepProj.SwigPyIterator___isub__, None, SwigPyIterator)
SwigPyIterator.__add__ = new_instancemethod(_BRepProj.SwigPyIterator___add__, None, SwigPyIterator)
SwigPyIterator.__sub__ = new_instancemethod(_BRepProj.SwigPyIterator___sub__, None, SwigPyIterator)
SwigPyIterator_swigregister = _BRepProj.SwigPyIterator_swigregister
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
    return _BRepProj.process_exception(error, method_name, class_name)
process_exception = _BRepProj.process_exception

from six import with_metaclass
import warnings
from OCC.Wrapper.wrapper_utils import Proxy, deprecated

import OCC.Core.Standard
import OCC.Core.NCollection
import OCC.Core.TopoDS
import OCC.Core.Message
import OCC.Core.TopAbs
import OCC.Core.TopLoc
import OCC.Core.gp
class BRepProj_Projection(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def __init__(self, *args):
        """
        * Makes a Cylindrical projection of Wire om Shape
        	:param Wire:
        	:type Wire: TopoDS_Shape
        	:param Shape:
        	:type Shape: TopoDS_Shape
        	:param D:
        	:type D: gp_Dir
        	:rtype: None
        * Makes a Conical projection of Wire om Shape
        	:param Wire:
        	:type Wire: TopoDS_Shape
        	:param Shape:
        	:type Shape: TopoDS_Shape
        	:param P:
        	:type P: gp_Pnt
        	:rtype: None
        """
        _BRepProj.BRepProj_Projection_swiginit(self, _BRepProj.new_BRepProj_Projection(*args))

    def Current(self, *args) -> "TopoDS_Wire":
        """
        * Returns the current result wire.
        	:rtype: TopoDS_Wire
        """
        return _BRepProj.BRepProj_Projection_Current(self, *args)


    def Init(self, *args) -> "void":
        """
        * Resets the iterator by resulting wires.
        	:rtype: None
        """
        return _BRepProj.BRepProj_Projection_Init(self, *args)


    def IsDone(self, *args) -> "Standard_Boolean":
        """
        * returns False if the section failed
        	:rtype: bool
        """
        return _BRepProj.BRepProj_Projection_IsDone(self, *args)


    def More(self, *args) -> "Standard_Boolean":
        """
        * Returns True if there is a current result wire
        	:rtype: bool
        """
        return _BRepProj.BRepProj_Projection_More(self, *args)


    def Next(self, *args) -> "void":
        """
        * Move to the next result wire.
        	:rtype: None
        """
        return _BRepProj.BRepProj_Projection_Next(self, *args)


    def Shape(self, *args) -> "TopoDS_Compound":
        """
        * Returns the complete result as compound of wires.
        	:rtype: TopoDS_Compound
        """
        return _BRepProj.BRepProj_Projection_Shape(self, *args)


    __repr__ = _dumps_object

    __swig_destroy__ = _BRepProj.delete_BRepProj_Projection
BRepProj_Projection.Current = new_instancemethod(_BRepProj.BRepProj_Projection_Current, None, BRepProj_Projection)
BRepProj_Projection.Init = new_instancemethod(_BRepProj.BRepProj_Projection_Init, None, BRepProj_Projection)
BRepProj_Projection.IsDone = new_instancemethod(_BRepProj.BRepProj_Projection_IsDone, None, BRepProj_Projection)
BRepProj_Projection.More = new_instancemethod(_BRepProj.BRepProj_Projection_More, None, BRepProj_Projection)
BRepProj_Projection.Next = new_instancemethod(_BRepProj.BRepProj_Projection_Next, None, BRepProj_Projection)
BRepProj_Projection.Shape = new_instancemethod(_BRepProj.BRepProj_Projection_Shape, None, BRepProj_Projection)
BRepProj_Projection_swigregister = _BRepProj.BRepProj_Projection_swigregister
BRepProj_Projection_swigregister(BRepProj_Projection)



