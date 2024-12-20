# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.12
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

"""
Sweep module, see official documentation at
https://www.opencascade.com/doc/occt-7.4.0/refman/html/package_sweep.html
"""


from sys import version_info as _swig_python_version_info
if _swig_python_version_info >= (3, 0, 0):
    new_instancemethod = lambda func, inst, cls: _Sweep.SWIG_PyInstanceMethod_New(func)
else:
    from new import instancemethod as new_instancemethod
if _swig_python_version_info >= (2, 7, 0):
    def swig_import_helper():
        import importlib
        pkg = __name__.rpartition('.')[0]
        mname = '.'.join((pkg, '_Sweep')).lstrip('.')
        try:
            return importlib.import_module(mname)
        except ImportError:
            return importlib.import_module('_Sweep')
    _Sweep = swig_import_helper()
    del swig_import_helper
elif _swig_python_version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_Sweep', [dirname(__file__)])
        except ImportError:
            import _Sweep
            return _Sweep
        try:
            _mod = imp.load_module('_Sweep', fp, pathname, description)
        finally:
            if fp is not None:
                fp.close()
        return _mod
    _Sweep = swig_import_helper()
    del swig_import_helper
else:
    import _Sweep
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
    __swig_destroy__ = _Sweep.delete_SwigPyIterator
    def __iter__(self):
        return self
SwigPyIterator.value = new_instancemethod(_Sweep.SwigPyIterator_value, None, SwigPyIterator)
SwigPyIterator.incr = new_instancemethod(_Sweep.SwigPyIterator_incr, None, SwigPyIterator)
SwigPyIterator.decr = new_instancemethod(_Sweep.SwigPyIterator_decr, None, SwigPyIterator)
SwigPyIterator.distance = new_instancemethod(_Sweep.SwigPyIterator_distance, None, SwigPyIterator)
SwigPyIterator.equal = new_instancemethod(_Sweep.SwigPyIterator_equal, None, SwigPyIterator)
SwigPyIterator.copy = new_instancemethod(_Sweep.SwigPyIterator_copy, None, SwigPyIterator)
SwigPyIterator.next = new_instancemethod(_Sweep.SwigPyIterator_next, None, SwigPyIterator)
SwigPyIterator.__next__ = new_instancemethod(_Sweep.SwigPyIterator___next__, None, SwigPyIterator)
SwigPyIterator.previous = new_instancemethod(_Sweep.SwigPyIterator_previous, None, SwigPyIterator)
SwigPyIterator.advance = new_instancemethod(_Sweep.SwigPyIterator_advance, None, SwigPyIterator)
SwigPyIterator.__eq__ = new_instancemethod(_Sweep.SwigPyIterator___eq__, None, SwigPyIterator)
SwigPyIterator.__ne__ = new_instancemethod(_Sweep.SwigPyIterator___ne__, None, SwigPyIterator)
SwigPyIterator.__iadd__ = new_instancemethod(_Sweep.SwigPyIterator___iadd__, None, SwigPyIterator)
SwigPyIterator.__isub__ = new_instancemethod(_Sweep.SwigPyIterator___isub__, None, SwigPyIterator)
SwigPyIterator.__add__ = new_instancemethod(_Sweep.SwigPyIterator___add__, None, SwigPyIterator)
SwigPyIterator.__sub__ = new_instancemethod(_Sweep.SwigPyIterator___sub__, None, SwigPyIterator)
SwigPyIterator_swigregister = _Sweep.SwigPyIterator_swigregister
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
    return _Sweep.process_exception(error, method_name, class_name)
process_exception = _Sweep.process_exception

from six import with_metaclass
import warnings
from OCC.Wrapper.wrapper_utils import Proxy, deprecated

import OCC.Core.Standard
import OCC.Core.NCollection
import OCC.Core.TopAbs
class Sweep_NumShape(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def BegInfinite(self, *args) -> "Standard_Boolean":
        """:rtype: bool"""
        return _Sweep.Sweep_NumShape_BegInfinite(self, *args)


    def Closed(self, *args) -> "Standard_Boolean":
        """:rtype: bool"""
        return _Sweep.Sweep_NumShape_Closed(self, *args)


    def EndInfinite(self, *args) -> "Standard_Boolean":
        """:rtype: bool"""
        return _Sweep.Sweep_NumShape_EndInfinite(self, *args)


    def Index(self, *args) -> "Standard_Integer":
        """:rtype: int"""
        return _Sweep.Sweep_NumShape_Index(self, *args)


    def Init(self, *args) -> "void":
        """
        * Reinitialize a simple indexed edge. //! For an Edge : Index is the number of vertices (0, 1 or 2),Type is TopAbs_EDGE, Closed is true if it is a closed edge, BegInf is true if the Edge is infinite at the begenning, EndInf is true if the edge is infinite at the end. //! For a Vertex : Index is the index of the vertex in the edge (1 or 2), Type is TopAbsVERTEX, Closed is true if it is the vertex of a closed edge, all the other fields have no meanning.
        	:param Index:
        	:type Index: int
        	:param Type:
        	:type Type: TopAbs_ShapeEnum
        	:param Closed: default value is Standard_False
        	:type Closed: bool
        	:param BegInf: default value is Standard_False
        	:type BegInf: bool
        	:param EndInf: default value is Standard_False
        	:type EndInf: bool
        	:rtype: None
        """
        return _Sweep.Sweep_NumShape_Init(self, *args)


    def Orientation(self, *args) -> "TopAbs_Orientation":
        """:rtype: TopAbs_Orientation"""
        return _Sweep.Sweep_NumShape_Orientation(self, *args)


    def __init__(self, *args):
        """
        * Creates a dummy indexed edge.
        	:rtype: None
        * Creates a new simple indexed edge. //! For an Edge : Index is the number of vertices (0, 1 or 2),Type is TopAbs_EDGE, Closed is true if it is a closed edge, BegInf is true if the Edge is infinite at the begenning, EndInf is true if the edge is infinite at the end. //! For a Vertex : Index is the index of the vertex in the edge (1 or 2), Type is TopAbsVERTEX, all the other fields have no meanning.
        	:param Index:
        	:type Index: int
        	:param Type:
        	:type Type: TopAbs_ShapeEnum
        	:param Closed: default value is Standard_False
        	:type Closed: bool
        	:param BegInf: default value is Standard_False
        	:type BegInf: bool
        	:param EndInf: default value is Standard_False
        	:type EndInf: bool
        	:rtype: None
        """
        _Sweep.Sweep_NumShape_swiginit(self, _Sweep.new_Sweep_NumShape(*args))

    def Type(self, *args) -> "TopAbs_ShapeEnum":
        """:rtype: TopAbs_ShapeEnum"""
        return _Sweep.Sweep_NumShape_Type(self, *args)


    __repr__ = _dumps_object

    __swig_destroy__ = _Sweep.delete_Sweep_NumShape
Sweep_NumShape.BegInfinite = new_instancemethod(_Sweep.Sweep_NumShape_BegInfinite, None, Sweep_NumShape)
Sweep_NumShape.Closed = new_instancemethod(_Sweep.Sweep_NumShape_Closed, None, Sweep_NumShape)
Sweep_NumShape.EndInfinite = new_instancemethod(_Sweep.Sweep_NumShape_EndInfinite, None, Sweep_NumShape)
Sweep_NumShape.Index = new_instancemethod(_Sweep.Sweep_NumShape_Index, None, Sweep_NumShape)
Sweep_NumShape.Init = new_instancemethod(_Sweep.Sweep_NumShape_Init, None, Sweep_NumShape)
Sweep_NumShape.Orientation = new_instancemethod(_Sweep.Sweep_NumShape_Orientation, None, Sweep_NumShape)
Sweep_NumShape.Type = new_instancemethod(_Sweep.Sweep_NumShape_Type, None, Sweep_NumShape)
Sweep_NumShape_swigregister = _Sweep.Sweep_NumShape_swigregister
Sweep_NumShape_swigregister(Sweep_NumShape)

class Sweep_NumShapeIterator(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def Init(self, *args) -> "void":
        """
        * Resest the NumShapeIterator on sub-shapes of <aShape>.
        	:param aShape:
        	:type aShape: Sweep_NumShape
        	:rtype: None
        """
        return _Sweep.Sweep_NumShapeIterator_Init(self, *args)


    def More(self, *args) -> "Standard_Boolean":
        """
        * Returns True if there is a current sub-shape.
        	:rtype: bool
        """
        return _Sweep.Sweep_NumShapeIterator_More(self, *args)


    def Next(self, *args) -> "void":
        """
        * Moves to the next sub-shape.
        	:rtype: None
        """
        return _Sweep.Sweep_NumShapeIterator_Next(self, *args)


    def Orientation(self, *args) -> "TopAbs_Orientation":
        """
        * Returns the orientation of the current sub-shape.
        	:rtype: TopAbs_Orientation
        """
        return _Sweep.Sweep_NumShapeIterator_Orientation(self, *args)


    def __init__(self, *args):
        """:rtype: None"""
        _Sweep.Sweep_NumShapeIterator_swiginit(self, _Sweep.new_Sweep_NumShapeIterator(*args))

    def Value(self, *args) -> "Sweep_NumShape const &":
        """
        * Returns the current sub-shape.
        	:rtype: Sweep_NumShape
        """
        return _Sweep.Sweep_NumShapeIterator_Value(self, *args)


    __repr__ = _dumps_object

    __swig_destroy__ = _Sweep.delete_Sweep_NumShapeIterator
Sweep_NumShapeIterator.Init = new_instancemethod(_Sweep.Sweep_NumShapeIterator_Init, None, Sweep_NumShapeIterator)
Sweep_NumShapeIterator.More = new_instancemethod(_Sweep.Sweep_NumShapeIterator_More, None, Sweep_NumShapeIterator)
Sweep_NumShapeIterator.Next = new_instancemethod(_Sweep.Sweep_NumShapeIterator_Next, None, Sweep_NumShapeIterator)
Sweep_NumShapeIterator.Orientation = new_instancemethod(_Sweep.Sweep_NumShapeIterator_Orientation, None, Sweep_NumShapeIterator)
Sweep_NumShapeIterator.Value = new_instancemethod(_Sweep.Sweep_NumShapeIterator_Value, None, Sweep_NumShapeIterator)
Sweep_NumShapeIterator_swigregister = _Sweep.Sweep_NumShapeIterator_swigregister
Sweep_NumShapeIterator_swigregister(Sweep_NumShapeIterator)

class Sweep_NumShapeTool(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def FirstVertex(self, *args) -> "Sweep_NumShape":
        """
        * Returns the first vertex.
        	:rtype: Sweep_NumShape
        """
        return _Sweep.Sweep_NumShapeTool_FirstVertex(self, *args)


    def HasFirstVertex(self, *args) -> "Standard_Boolean":
        """
        * Returns true if there is a First Vertex in the Shape.
        	:rtype: bool
        """
        return _Sweep.Sweep_NumShapeTool_HasFirstVertex(self, *args)


    def HasLastVertex(self, *args) -> "Standard_Boolean":
        """
        * Returns true if there is a Last Vertex in the Shape.
        	:rtype: bool
        """
        return _Sweep.Sweep_NumShapeTool_HasLastVertex(self, *args)


    def Index(self, *args) -> "Standard_Integer":
        """
        * Returns the index of <aShape>.
        	:param aShape:
        	:type aShape: Sweep_NumShape
        	:rtype: int
        """
        return _Sweep.Sweep_NumShapeTool_Index(self, *args)


    def LastVertex(self, *args) -> "Sweep_NumShape":
        """
        * Returns the last vertex.
        	:rtype: Sweep_NumShape
        """
        return _Sweep.Sweep_NumShapeTool_LastVertex(self, *args)


    def NbShapes(self, *args) -> "Standard_Integer":
        """
        * Returns the number of subshapes in the shape.
        	:rtype: int
        """
        return _Sweep.Sweep_NumShapeTool_NbShapes(self, *args)


    def Orientation(self, *args) -> "TopAbs_Orientation":
        """
        * Returns the orientation of <aShape>.
        	:param aShape:
        	:type aShape: Sweep_NumShape
        	:rtype: TopAbs_Orientation
        """
        return _Sweep.Sweep_NumShapeTool_Orientation(self, *args)


    def Shape(self, *args) -> "Sweep_NumShape":
        """
        * Returns the Shape at index anIndex
        	:param anIndex:
        	:type anIndex: int
        	:rtype: Sweep_NumShape
        """
        return _Sweep.Sweep_NumShapeTool_Shape(self, *args)


    def __init__(self, *args):
        """
        * Create a new NumShapeTool with <aShape>. The Tool must prepare an indexation for all the subshapes of this shape.
        	:param aShape:
        	:type aShape: Sweep_NumShape
        	:rtype: None
        """
        _Sweep.Sweep_NumShapeTool_swiginit(self, _Sweep.new_Sweep_NumShapeTool(*args))

    def Type(self, *args) -> "TopAbs_ShapeEnum":
        """
        * Returns the type of <aShape>.
        	:param aShape:
        	:type aShape: Sweep_NumShape
        	:rtype: TopAbs_ShapeEnum
        """
        return _Sweep.Sweep_NumShapeTool_Type(self, *args)


    __repr__ = _dumps_object

    __swig_destroy__ = _Sweep.delete_Sweep_NumShapeTool
Sweep_NumShapeTool.FirstVertex = new_instancemethod(_Sweep.Sweep_NumShapeTool_FirstVertex, None, Sweep_NumShapeTool)
Sweep_NumShapeTool.HasFirstVertex = new_instancemethod(_Sweep.Sweep_NumShapeTool_HasFirstVertex, None, Sweep_NumShapeTool)
Sweep_NumShapeTool.HasLastVertex = new_instancemethod(_Sweep.Sweep_NumShapeTool_HasLastVertex, None, Sweep_NumShapeTool)
Sweep_NumShapeTool.Index = new_instancemethod(_Sweep.Sweep_NumShapeTool_Index, None, Sweep_NumShapeTool)
Sweep_NumShapeTool.LastVertex = new_instancemethod(_Sweep.Sweep_NumShapeTool_LastVertex, None, Sweep_NumShapeTool)
Sweep_NumShapeTool.NbShapes = new_instancemethod(_Sweep.Sweep_NumShapeTool_NbShapes, None, Sweep_NumShapeTool)
Sweep_NumShapeTool.Orientation = new_instancemethod(_Sweep.Sweep_NumShapeTool_Orientation, None, Sweep_NumShapeTool)
Sweep_NumShapeTool.Shape = new_instancemethod(_Sweep.Sweep_NumShapeTool_Shape, None, Sweep_NumShapeTool)
Sweep_NumShapeTool.Type = new_instancemethod(_Sweep.Sweep_NumShapeTool_Type, None, Sweep_NumShapeTool)
Sweep_NumShapeTool_swigregister = _Sweep.Sweep_NumShapeTool_swigregister
Sweep_NumShapeTool_swigregister(Sweep_NumShapeTool)



