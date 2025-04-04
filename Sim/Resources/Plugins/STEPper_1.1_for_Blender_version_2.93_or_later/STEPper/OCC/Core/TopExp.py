# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.12
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

"""
TopExp module, see official documentation at
https://www.opencascade.com/doc/occt-7.4.0/refman/html/package_topexp.html
"""


from sys import version_info as _swig_python_version_info
if _swig_python_version_info >= (3, 0, 0):
    new_instancemethod = lambda func, inst, cls: _TopExp.SWIG_PyInstanceMethod_New(func)
else:
    from new import instancemethod as new_instancemethod
if _swig_python_version_info >= (2, 7, 0):
    def swig_import_helper():
        import importlib
        pkg = __name__.rpartition('.')[0]
        mname = '.'.join((pkg, '_TopExp')).lstrip('.')
        try:
            return importlib.import_module(mname)
        except ImportError:
            return importlib.import_module('_TopExp')
    _TopExp = swig_import_helper()
    del swig_import_helper
elif _swig_python_version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_TopExp', [dirname(__file__)])
        except ImportError:
            import _TopExp
            return _TopExp
        try:
            _mod = imp.load_module('_TopExp', fp, pathname, description)
        finally:
            if fp is not None:
                fp.close()
        return _mod
    _TopExp = swig_import_helper()
    del swig_import_helper
else:
    import _TopExp
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
    __swig_destroy__ = _TopExp.delete_SwigPyIterator
    def __iter__(self):
        return self
SwigPyIterator.value = new_instancemethod(_TopExp.SwigPyIterator_value, None, SwigPyIterator)
SwigPyIterator.incr = new_instancemethod(_TopExp.SwigPyIterator_incr, None, SwigPyIterator)
SwigPyIterator.decr = new_instancemethod(_TopExp.SwigPyIterator_decr, None, SwigPyIterator)
SwigPyIterator.distance = new_instancemethod(_TopExp.SwigPyIterator_distance, None, SwigPyIterator)
SwigPyIterator.equal = new_instancemethod(_TopExp.SwigPyIterator_equal, None, SwigPyIterator)
SwigPyIterator.copy = new_instancemethod(_TopExp.SwigPyIterator_copy, None, SwigPyIterator)
SwigPyIterator.next = new_instancemethod(_TopExp.SwigPyIterator_next, None, SwigPyIterator)
SwigPyIterator.__next__ = new_instancemethod(_TopExp.SwigPyIterator___next__, None, SwigPyIterator)
SwigPyIterator.previous = new_instancemethod(_TopExp.SwigPyIterator_previous, None, SwigPyIterator)
SwigPyIterator.advance = new_instancemethod(_TopExp.SwigPyIterator_advance, None, SwigPyIterator)
SwigPyIterator.__eq__ = new_instancemethod(_TopExp.SwigPyIterator___eq__, None, SwigPyIterator)
SwigPyIterator.__ne__ = new_instancemethod(_TopExp.SwigPyIterator___ne__, None, SwigPyIterator)
SwigPyIterator.__iadd__ = new_instancemethod(_TopExp.SwigPyIterator___iadd__, None, SwigPyIterator)
SwigPyIterator.__isub__ = new_instancemethod(_TopExp.SwigPyIterator___isub__, None, SwigPyIterator)
SwigPyIterator.__add__ = new_instancemethod(_TopExp.SwigPyIterator___add__, None, SwigPyIterator)
SwigPyIterator.__sub__ = new_instancemethod(_TopExp.SwigPyIterator___sub__, None, SwigPyIterator)
SwigPyIterator_swigregister = _TopExp.SwigPyIterator_swigregister
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
    return _TopExp.process_exception(error, method_name, class_name)
process_exception = _TopExp.process_exception

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
import OCC.Core.TopTools
import OCC.Core.TCollection
class topexp(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def CommonVertex(*args) -> "Standard_Boolean":
        """
        * Finds the vertex <V> common to the two edges <E1,E2>, returns True if this vertex exists. //! Warning: <V> has sense only if the value <True> is returned
        	:param E1:
        	:type E1: TopoDS_Edge
        	:param E2:
        	:type E2: TopoDS_Edge
        	:param V:
        	:type V: TopoDS_Vertex
        	:rtype: bool
        """
        return _TopExp.topexp_CommonVertex(*args)

    CommonVertex = staticmethod(CommonVertex)

    def FirstVertex(*args) -> "TopoDS_Vertex":
        """
        * Returns the Vertex of orientation FORWARD in E. If there is none returns a Null Shape. CumOri = True : taking account the edge orientation
        	:param E:
        	:type E: TopoDS_Edge
        	:param CumOri: default value is Standard_False
        	:type CumOri: bool
        	:rtype: TopoDS_Vertex
        """
        return _TopExp.topexp_FirstVertex(*args)

    FirstVertex = staticmethod(FirstVertex)

    def LastVertex(*args) -> "TopoDS_Vertex":
        """
        * Returns the Vertex of orientation REVERSED in E. If there is none returns a Null Shape. CumOri = True : taking account the edge orientation
        	:param E:
        	:type E: TopoDS_Edge
        	:param CumOri: default value is Standard_False
        	:type CumOri: bool
        	:rtype: TopoDS_Vertex
        """
        return _TopExp.topexp_LastVertex(*args)

    LastVertex = staticmethod(LastVertex)

    def MapShapes(*args) -> "void":
        """
        * Tool to explore a topological data structure. Stores in the map <M> all the sub-shapes of <S> of type <T>. //! Warning: The map is not cleared at first.
        	:param S:
        	:type S: TopoDS_Shape
        	:param T:
        	:type T: TopAbs_ShapeEnum
        	:param M:
        	:type M: TopTools_IndexedMapOfShape
        	:rtype: void
        * Stores in the map <M> all the sub-shapes of <S>.
        	:param S:
        	:type S: TopoDS_Shape
        	:param M:
        	:type M: TopTools_IndexedMapOfShape
        	:rtype: void
        * Stores in the map <M> all the sub-shapes of <S>.
        	:param S:
        	:type S: TopoDS_Shape
        	:param M:
        	:type M: TopTools_MapOfShape
        	:rtype: void
        """
        return _TopExp.topexp_MapShapes(*args)

    MapShapes = staticmethod(MapShapes)

    def MapShapesAndAncestors(*args) -> "void":
        """
        * Stores in the map <M> all the subshape of <S> of type <TS> for each one append to the list all the ancestors of type <TA>. For example map all the edges and bind the list of faces. Warning: The map is not cleared at first.
        	:param S:
        	:type S: TopoDS_Shape
        	:param TS:
        	:type TS: TopAbs_ShapeEnum
        	:param TA:
        	:type TA: TopAbs_ShapeEnum
        	:param M:
        	:type M: TopTools_IndexedDataMapOfShapeListOfShape
        	:rtype: void
        """
        return _TopExp.topexp_MapShapesAndAncestors(*args)

    MapShapesAndAncestors = staticmethod(MapShapesAndAncestors)

    def MapShapesAndUniqueAncestors(*args) -> "void":
        """
        * Stores in the map <M> all the subshape of <S> of type <TS> for each one append to the list all unique ancestors of type <TA>. For example map all the edges and bind the list of faces. useOrientation = True : taking account the ancestor orientation Warning: The map is not cleared at first.
        	:param S:
        	:type S: TopoDS_Shape
        	:param TS:
        	:type TS: TopAbs_ShapeEnum
        	:param TA:
        	:type TA: TopAbs_ShapeEnum
        	:param M:
        	:type M: TopTools_IndexedDataMapOfShapeListOfShape
        	:param useOrientation: default value is Standard_False
        	:type useOrientation: bool
        	:rtype: void
        """
        return _TopExp.topexp_MapShapesAndUniqueAncestors(*args)

    MapShapesAndUniqueAncestors = staticmethod(MapShapesAndUniqueAncestors)

    def Vertices(*args) -> "void":
        """
        * Returns in Vfirst, Vlast the FORWARD and REVERSED vertices of the edge <E>. May be null shapes. CumOri = True : taking account the edge orientation
        	:param E:
        	:type E: TopoDS_Edge
        	:param Vfirst:
        	:type Vfirst: TopoDS_Vertex
        	:param Vlast:
        	:type Vlast: TopoDS_Vertex
        	:param CumOri: default value is Standard_False
        	:type CumOri: bool
        	:rtype: void
        * Returns in Vfirst, Vlast the first and last vertices of the open wire <W>. May be null shapes. if <W> is closed Vfirst and Vlast are a same vertex on <W>. if <W> is no manifold. VFirst and VLast are null shapes.
        	:param W:
        	:type W: TopoDS_Wire
        	:param Vfirst:
        	:type Vfirst: TopoDS_Vertex
        	:param Vlast:
        	:type Vlast: TopoDS_Vertex
        	:rtype: void
        """
        return _TopExp.topexp_Vertices(*args)

    Vertices = staticmethod(Vertices)

    __repr__ = _dumps_object


    def __init__(self):
        _TopExp.topexp_swiginit(self, _TopExp.new_topexp())
    __swig_destroy__ = _TopExp.delete_topexp
topexp_swigregister = _TopExp.topexp_swigregister
topexp_swigregister(topexp)

def topexp_CommonVertex(*args) -> "Standard_Boolean":
    """
    * Finds the vertex <V> common to the two edges <E1,E2>, returns True if this vertex exists. //! Warning: <V> has sense only if the value <True> is returned
    	:param E1:
    	:type E1: TopoDS_Edge
    	:param E2:
    	:type E2: TopoDS_Edge
    	:param V:
    	:type V: TopoDS_Vertex
    	:rtype: bool
    """
    return _TopExp.topexp_CommonVertex(*args)

def topexp_FirstVertex(*args) -> "TopoDS_Vertex":
    """
    * Returns the Vertex of orientation FORWARD in E. If there is none returns a Null Shape. CumOri = True : taking account the edge orientation
    	:param E:
    	:type E: TopoDS_Edge
    	:param CumOri: default value is Standard_False
    	:type CumOri: bool
    	:rtype: TopoDS_Vertex
    """
    return _TopExp.topexp_FirstVertex(*args)

def topexp_LastVertex(*args) -> "TopoDS_Vertex":
    """
    * Returns the Vertex of orientation REVERSED in E. If there is none returns a Null Shape. CumOri = True : taking account the edge orientation
    	:param E:
    	:type E: TopoDS_Edge
    	:param CumOri: default value is Standard_False
    	:type CumOri: bool
    	:rtype: TopoDS_Vertex
    """
    return _TopExp.topexp_LastVertex(*args)

def topexp_MapShapes(*args) -> "void":
    """
    * Tool to explore a topological data structure. Stores in the map <M> all the sub-shapes of <S> of type <T>. //! Warning: The map is not cleared at first.
    	:param S:
    	:type S: TopoDS_Shape
    	:param T:
    	:type T: TopAbs_ShapeEnum
    	:param M:
    	:type M: TopTools_IndexedMapOfShape
    	:rtype: void
    * Stores in the map <M> all the sub-shapes of <S>.
    	:param S:
    	:type S: TopoDS_Shape
    	:param M:
    	:type M: TopTools_IndexedMapOfShape
    	:rtype: void
    * Stores in the map <M> all the sub-shapes of <S>.
    	:param S:
    	:type S: TopoDS_Shape
    	:param M:
    	:type M: TopTools_MapOfShape
    	:rtype: void
    """
    return _TopExp.topexp_MapShapes(*args)

def topexp_MapShapesAndAncestors(*args) -> "void":
    """
    * Stores in the map <M> all the subshape of <S> of type <TS> for each one append to the list all the ancestors of type <TA>. For example map all the edges and bind the list of faces. Warning: The map is not cleared at first.
    	:param S:
    	:type S: TopoDS_Shape
    	:param TS:
    	:type TS: TopAbs_ShapeEnum
    	:param TA:
    	:type TA: TopAbs_ShapeEnum
    	:param M:
    	:type M: TopTools_IndexedDataMapOfShapeListOfShape
    	:rtype: void
    """
    return _TopExp.topexp_MapShapesAndAncestors(*args)

def topexp_MapShapesAndUniqueAncestors(*args) -> "void":
    """
    * Stores in the map <M> all the subshape of <S> of type <TS> for each one append to the list all unique ancestors of type <TA>. For example map all the edges and bind the list of faces. useOrientation = True : taking account the ancestor orientation Warning: The map is not cleared at first.
    	:param S:
    	:type S: TopoDS_Shape
    	:param TS:
    	:type TS: TopAbs_ShapeEnum
    	:param TA:
    	:type TA: TopAbs_ShapeEnum
    	:param M:
    	:type M: TopTools_IndexedDataMapOfShapeListOfShape
    	:param useOrientation: default value is Standard_False
    	:type useOrientation: bool
    	:rtype: void
    """
    return _TopExp.topexp_MapShapesAndUniqueAncestors(*args)

def topexp_Vertices(*args) -> "void":
    """
    * Returns in Vfirst, Vlast the FORWARD and REVERSED vertices of the edge <E>. May be null shapes. CumOri = True : taking account the edge orientation
    	:param E:
    	:type E: TopoDS_Edge
    	:param Vfirst:
    	:type Vfirst: TopoDS_Vertex
    	:param Vlast:
    	:type Vlast: TopoDS_Vertex
    	:param CumOri: default value is Standard_False
    	:type CumOri: bool
    	:rtype: void
    * Returns in Vfirst, Vlast the first and last vertices of the open wire <W>. May be null shapes. if <W> is closed Vfirst and Vlast are a same vertex on <W>. if <W> is no manifold. VFirst and VLast are null shapes.
    	:param W:
    	:type W: TopoDS_Wire
    	:param Vfirst:
    	:type Vfirst: TopoDS_Vertex
    	:param Vlast:
    	:type Vlast: TopoDS_Vertex
    	:rtype: void
    """
    return _TopExp.topexp_Vertices(*args)

class TopExp_Explorer(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def Clear(self, *args) -> "void":
        """
        * Clears the content of the explorer. It will return False on More().
        	:rtype: None
        """
        return _TopExp.TopExp_Explorer_Clear(self, *args)


    def Current(self, *args) -> "TopoDS_Shape const":
        """
        * Returns the current shape in the exploration. Exceptions Standard_NoSuchObject if this explorer has no more shapes to explore.
        	:rtype: TopoDS_Shape
        """
        return _TopExp.TopExp_Explorer_Current(self, *args)


    def Depth(self, *args) -> "Standard_Integer":
        """
        * Returns the current depth of the exploration. 0 is the shape to explore itself.
        	:rtype: int
        """
        return _TopExp.TopExp_Explorer_Depth(self, *args)


    def Destroy(self, *args) -> "void":
        """:rtype: None"""
        return _TopExp.TopExp_Explorer_Destroy(self, *args)


    def Init(self, *args) -> "void":
        """
        * Resets this explorer on the shape S. It is initialized to search the shape S, for shapes of type ToFind, that are not part of a shape ToAvoid. If the shape ToAvoid is equal to TopAbs_SHAPE, or if it is the same as, or less complex than, the shape ToFind it has no effect on the search.
        	:param S:
        	:type S: TopoDS_Shape
        	:param ToFind:
        	:type ToFind: TopAbs_ShapeEnum
        	:param ToAvoid: default value is TopAbs_SHAPE
        	:type ToAvoid: TopAbs_ShapeEnum
        	:rtype: None
        """
        return _TopExp.TopExp_Explorer_Init(self, *args)


    def More(self, *args) -> "Standard_Boolean":
        """
        * Returns True if there are more shapes in the exploration.
        	:rtype: bool
        """
        return _TopExp.TopExp_Explorer_More(self, *args)


    def Next(self, *args) -> "void":
        """
        * Moves to the next Shape in the exploration. Exceptions Standard_NoMoreObject if there are no more shapes to explore.
        	:rtype: None
        """
        return _TopExp.TopExp_Explorer_Next(self, *args)


    def ReInit(self, *args) -> "void":
        """
        * Reinitialize the exploration with the original arguments.
        	:rtype: None
        """
        return _TopExp.TopExp_Explorer_ReInit(self, *args)


    def __init__(self, *args):
        """
        * Creates an empty explorer, becomes usefull after Init.
        	:rtype: None
        * Creates an Explorer on the Shape <S>. //! <ToFind> is the type of shapes to search. TopAbs_VERTEX, TopAbs_EDGE, ... //! <ToAvoid> is the type of shape to skip in the exploration. If <ToAvoid> is equal or less complex than <ToFind> or if <ToAVoid> is SHAPE it has no effect on the exploration.
        	:param S:
        	:type S: TopoDS_Shape
        	:param ToFind:
        	:type ToFind: TopAbs_ShapeEnum
        	:param ToAvoid: default value is TopAbs_SHAPE
        	:type ToAvoid: TopAbs_ShapeEnum
        	:rtype: None
        """
        _TopExp.TopExp_Explorer_swiginit(self, _TopExp.new_TopExp_Explorer(*args))

    def Value(self, *args) -> "TopoDS_Shape const":
        """
        * Returns the current shape in the exploration. Exceptions Standard_NoSuchObject if this explorer has no more shapes to explore.
        	:rtype: TopoDS_Shape
        """
        return _TopExp.TopExp_Explorer_Value(self, *args)


    __repr__ = _dumps_object

    __swig_destroy__ = _TopExp.delete_TopExp_Explorer
TopExp_Explorer.Clear = new_instancemethod(_TopExp.TopExp_Explorer_Clear, None, TopExp_Explorer)
TopExp_Explorer.Current = new_instancemethod(_TopExp.TopExp_Explorer_Current, None, TopExp_Explorer)
TopExp_Explorer.Depth = new_instancemethod(_TopExp.TopExp_Explorer_Depth, None, TopExp_Explorer)
TopExp_Explorer.Destroy = new_instancemethod(_TopExp.TopExp_Explorer_Destroy, None, TopExp_Explorer)
TopExp_Explorer.Init = new_instancemethod(_TopExp.TopExp_Explorer_Init, None, TopExp_Explorer)
TopExp_Explorer.More = new_instancemethod(_TopExp.TopExp_Explorer_More, None, TopExp_Explorer)
TopExp_Explorer.Next = new_instancemethod(_TopExp.TopExp_Explorer_Next, None, TopExp_Explorer)
TopExp_Explorer.ReInit = new_instancemethod(_TopExp.TopExp_Explorer_ReInit, None, TopExp_Explorer)
TopExp_Explorer.Value = new_instancemethod(_TopExp.TopExp_Explorer_Value, None, TopExp_Explorer)
TopExp_Explorer_swigregister = _TopExp.TopExp_Explorer_swigregister
TopExp_Explorer_swigregister(TopExp_Explorer)



