# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.12
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

"""
IMeshTools module, see official documentation at
https://www.opencascade.com/doc/occt-7.4.0/refman/html/package_imeshtools.html
"""


from sys import version_info as _swig_python_version_info
if _swig_python_version_info >= (3, 0, 0):
    new_instancemethod = lambda func, inst, cls: _IMeshTools.SWIG_PyInstanceMethod_New(func)
else:
    from new import instancemethod as new_instancemethod
if _swig_python_version_info >= (2, 7, 0):
    def swig_import_helper():
        import importlib
        pkg = __name__.rpartition('.')[0]
        mname = '.'.join((pkg, '_IMeshTools')).lstrip('.')
        try:
            return importlib.import_module(mname)
        except ImportError:
            return importlib.import_module('_IMeshTools')
    _IMeshTools = swig_import_helper()
    del swig_import_helper
elif _swig_python_version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_IMeshTools', [dirname(__file__)])
        except ImportError:
            import _IMeshTools
            return _IMeshTools
        try:
            _mod = imp.load_module('_IMeshTools', fp, pathname, description)
        finally:
            if fp is not None:
                fp.close()
        return _mod
    _IMeshTools = swig_import_helper()
    del swig_import_helper
else:
    import _IMeshTools
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
    __swig_destroy__ = _IMeshTools.delete_SwigPyIterator
    def __iter__(self):
        return self
SwigPyIterator.value = new_instancemethod(_IMeshTools.SwigPyIterator_value, None, SwigPyIterator)
SwigPyIterator.incr = new_instancemethod(_IMeshTools.SwigPyIterator_incr, None, SwigPyIterator)
SwigPyIterator.decr = new_instancemethod(_IMeshTools.SwigPyIterator_decr, None, SwigPyIterator)
SwigPyIterator.distance = new_instancemethod(_IMeshTools.SwigPyIterator_distance, None, SwigPyIterator)
SwigPyIterator.equal = new_instancemethod(_IMeshTools.SwigPyIterator_equal, None, SwigPyIterator)
SwigPyIterator.copy = new_instancemethod(_IMeshTools.SwigPyIterator_copy, None, SwigPyIterator)
SwigPyIterator.next = new_instancemethod(_IMeshTools.SwigPyIterator_next, None, SwigPyIterator)
SwigPyIterator.__next__ = new_instancemethod(_IMeshTools.SwigPyIterator___next__, None, SwigPyIterator)
SwigPyIterator.previous = new_instancemethod(_IMeshTools.SwigPyIterator_previous, None, SwigPyIterator)
SwigPyIterator.advance = new_instancemethod(_IMeshTools.SwigPyIterator_advance, None, SwigPyIterator)
SwigPyIterator.__eq__ = new_instancemethod(_IMeshTools.SwigPyIterator___eq__, None, SwigPyIterator)
SwigPyIterator.__ne__ = new_instancemethod(_IMeshTools.SwigPyIterator___ne__, None, SwigPyIterator)
SwigPyIterator.__iadd__ = new_instancemethod(_IMeshTools.SwigPyIterator___iadd__, None, SwigPyIterator)
SwigPyIterator.__isub__ = new_instancemethod(_IMeshTools.SwigPyIterator___isub__, None, SwigPyIterator)
SwigPyIterator.__add__ = new_instancemethod(_IMeshTools.SwigPyIterator___add__, None, SwigPyIterator)
SwigPyIterator.__sub__ = new_instancemethod(_IMeshTools.SwigPyIterator___sub__, None, SwigPyIterator)
SwigPyIterator_swigregister = _IMeshTools.SwigPyIterator_swigregister
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
    return _IMeshTools.process_exception(error, method_name, class_name)
process_exception = _IMeshTools.process_exception

from six import with_metaclass
import warnings
from OCC.Wrapper.wrapper_utils import Proxy, deprecated

import OCC.Core.Standard
import OCC.Core.NCollection
import OCC.Core.IMeshData
import OCC.Core.TopoDS
import OCC.Core.Message
import OCC.Core.TopAbs
import OCC.Core.TopLoc
import OCC.Core.gp
import OCC.Core.BRepAdaptor
import OCC.Core.Adaptor3d
import OCC.Core.Geom
import OCC.Core.GeomAbs
import OCC.Core.TColgp
import OCC.Core.TColStd
import OCC.Core.TCollection
import OCC.Core.Adaptor2d
import OCC.Core.Geom2d
import OCC.Core.math
import OCC.Core.GeomAdaptor
import OCC.Core.Geom2dAdaptor

def Handle_IMeshTools_Context_Create() -> "opencascade::handle< IMeshTools_Context >":
    return _IMeshTools.Handle_IMeshTools_Context_Create()
Handle_IMeshTools_Context_Create = _IMeshTools.Handle_IMeshTools_Context_Create

def Handle_IMeshTools_Context_DownCast(t: 'opencascade::handle< Standard_Transient > const &') -> "opencascade::handle< IMeshTools_Context >":
    return _IMeshTools.Handle_IMeshTools_Context_DownCast(t)
Handle_IMeshTools_Context_DownCast = _IMeshTools.Handle_IMeshTools_Context_DownCast

def Handle_IMeshTools_Context_IsNull(t: 'opencascade::handle< IMeshTools_Context > const &') -> "bool":
    return _IMeshTools.Handle_IMeshTools_Context_IsNull(t)
Handle_IMeshTools_Context_IsNull = _IMeshTools.Handle_IMeshTools_Context_IsNull

def Handle_IMeshTools_CurveTessellator_Create() -> "opencascade::handle< IMeshTools_CurveTessellator >":
    return _IMeshTools.Handle_IMeshTools_CurveTessellator_Create()
Handle_IMeshTools_CurveTessellator_Create = _IMeshTools.Handle_IMeshTools_CurveTessellator_Create

def Handle_IMeshTools_CurveTessellator_DownCast(t: 'opencascade::handle< Standard_Transient > const &') -> "opencascade::handle< IMeshTools_CurveTessellator >":
    return _IMeshTools.Handle_IMeshTools_CurveTessellator_DownCast(t)
Handle_IMeshTools_CurveTessellator_DownCast = _IMeshTools.Handle_IMeshTools_CurveTessellator_DownCast

def Handle_IMeshTools_CurveTessellator_IsNull(t: 'opencascade::handle< IMeshTools_CurveTessellator > const &') -> "bool":
    return _IMeshTools.Handle_IMeshTools_CurveTessellator_IsNull(t)
Handle_IMeshTools_CurveTessellator_IsNull = _IMeshTools.Handle_IMeshTools_CurveTessellator_IsNull

def Handle_IMeshTools_MeshAlgo_Create() -> "opencascade::handle< IMeshTools_MeshAlgo >":
    return _IMeshTools.Handle_IMeshTools_MeshAlgo_Create()
Handle_IMeshTools_MeshAlgo_Create = _IMeshTools.Handle_IMeshTools_MeshAlgo_Create

def Handle_IMeshTools_MeshAlgo_DownCast(t: 'opencascade::handle< Standard_Transient > const &') -> "opencascade::handle< IMeshTools_MeshAlgo >":
    return _IMeshTools.Handle_IMeshTools_MeshAlgo_DownCast(t)
Handle_IMeshTools_MeshAlgo_DownCast = _IMeshTools.Handle_IMeshTools_MeshAlgo_DownCast

def Handle_IMeshTools_MeshAlgo_IsNull(t: 'opencascade::handle< IMeshTools_MeshAlgo > const &') -> "bool":
    return _IMeshTools.Handle_IMeshTools_MeshAlgo_IsNull(t)
Handle_IMeshTools_MeshAlgo_IsNull = _IMeshTools.Handle_IMeshTools_MeshAlgo_IsNull

def Handle_IMeshTools_MeshAlgoFactory_Create() -> "opencascade::handle< IMeshTools_MeshAlgoFactory >":
    return _IMeshTools.Handle_IMeshTools_MeshAlgoFactory_Create()
Handle_IMeshTools_MeshAlgoFactory_Create = _IMeshTools.Handle_IMeshTools_MeshAlgoFactory_Create

def Handle_IMeshTools_MeshAlgoFactory_DownCast(t: 'opencascade::handle< Standard_Transient > const &') -> "opencascade::handle< IMeshTools_MeshAlgoFactory >":
    return _IMeshTools.Handle_IMeshTools_MeshAlgoFactory_DownCast(t)
Handle_IMeshTools_MeshAlgoFactory_DownCast = _IMeshTools.Handle_IMeshTools_MeshAlgoFactory_DownCast

def Handle_IMeshTools_MeshAlgoFactory_IsNull(t: 'opencascade::handle< IMeshTools_MeshAlgoFactory > const &') -> "bool":
    return _IMeshTools.Handle_IMeshTools_MeshAlgoFactory_IsNull(t)
Handle_IMeshTools_MeshAlgoFactory_IsNull = _IMeshTools.Handle_IMeshTools_MeshAlgoFactory_IsNull

def Handle_IMeshTools_ModelAlgo_Create() -> "opencascade::handle< IMeshTools_ModelAlgo >":
    return _IMeshTools.Handle_IMeshTools_ModelAlgo_Create()
Handle_IMeshTools_ModelAlgo_Create = _IMeshTools.Handle_IMeshTools_ModelAlgo_Create

def Handle_IMeshTools_ModelAlgo_DownCast(t: 'opencascade::handle< Standard_Transient > const &') -> "opencascade::handle< IMeshTools_ModelAlgo >":
    return _IMeshTools.Handle_IMeshTools_ModelAlgo_DownCast(t)
Handle_IMeshTools_ModelAlgo_DownCast = _IMeshTools.Handle_IMeshTools_ModelAlgo_DownCast

def Handle_IMeshTools_ModelAlgo_IsNull(t: 'opencascade::handle< IMeshTools_ModelAlgo > const &') -> "bool":
    return _IMeshTools.Handle_IMeshTools_ModelAlgo_IsNull(t)
Handle_IMeshTools_ModelAlgo_IsNull = _IMeshTools.Handle_IMeshTools_ModelAlgo_IsNull

def Handle_IMeshTools_ShapeExplorer_Create() -> "opencascade::handle< IMeshTools_ShapeExplorer >":
    return _IMeshTools.Handle_IMeshTools_ShapeExplorer_Create()
Handle_IMeshTools_ShapeExplorer_Create = _IMeshTools.Handle_IMeshTools_ShapeExplorer_Create

def Handle_IMeshTools_ShapeExplorer_DownCast(t: 'opencascade::handle< Standard_Transient > const &') -> "opencascade::handle< IMeshTools_ShapeExplorer >":
    return _IMeshTools.Handle_IMeshTools_ShapeExplorer_DownCast(t)
Handle_IMeshTools_ShapeExplorer_DownCast = _IMeshTools.Handle_IMeshTools_ShapeExplorer_DownCast

def Handle_IMeshTools_ShapeExplorer_IsNull(t: 'opencascade::handle< IMeshTools_ShapeExplorer > const &') -> "bool":
    return _IMeshTools.Handle_IMeshTools_ShapeExplorer_IsNull(t)
Handle_IMeshTools_ShapeExplorer_IsNull = _IMeshTools.Handle_IMeshTools_ShapeExplorer_IsNull

def Handle_IMeshTools_ShapeVisitor_Create() -> "opencascade::handle< IMeshTools_ShapeVisitor >":
    return _IMeshTools.Handle_IMeshTools_ShapeVisitor_Create()
Handle_IMeshTools_ShapeVisitor_Create = _IMeshTools.Handle_IMeshTools_ShapeVisitor_Create

def Handle_IMeshTools_ShapeVisitor_DownCast(t: 'opencascade::handle< Standard_Transient > const &') -> "opencascade::handle< IMeshTools_ShapeVisitor >":
    return _IMeshTools.Handle_IMeshTools_ShapeVisitor_DownCast(t)
Handle_IMeshTools_ShapeVisitor_DownCast = _IMeshTools.Handle_IMeshTools_ShapeVisitor_DownCast

def Handle_IMeshTools_ShapeVisitor_IsNull(t: 'opencascade::handle< IMeshTools_ShapeVisitor > const &') -> "bool":
    return _IMeshTools.Handle_IMeshTools_ShapeVisitor_IsNull(t)
Handle_IMeshTools_ShapeVisitor_IsNull = _IMeshTools.Handle_IMeshTools_ShapeVisitor_IsNull
class IMeshTools_Context(OCC.Core.IMeshData.IMeshData_Shape):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def BuildModel(self, *args) -> "Standard_Boolean":
        """
        * Builds model using assined model builder. returns True on success, False elsewhere.
        	:rtype: bool
        """
        return _IMeshTools.IMeshTools_Context_BuildModel(self, *args)


    def ChangeParameters(self, *args) -> "IMeshTools_Parameters &":
        """
        * Gets reference to parameters to be used for meshing.
        	:rtype: inline IMeshTools_Parameters
        """
        return _IMeshTools.IMeshTools_Context_ChangeParameters(self, *args)


    def Clean(self, *args) -> "void":
        """
        * Cleans temporary context data.
        	:rtype: void
        """
        return _IMeshTools.IMeshTools_Context_Clean(self, *args)


    def DiscretizeEdges(self, *args) -> "Standard_Boolean":
        """
        * Performs discretization of model edges using assigned edge discret algorithm. returns True on success, False elsewhere.
        	:rtype: bool
        """
        return _IMeshTools.IMeshTools_Context_DiscretizeEdges(self, *args)


    def DiscretizeFaces(self, *args) -> "Standard_Boolean":
        """
        * Performs meshing of faces of discrete model using assigned meshing algorithm. returns True on success, False elsewhere.
        	:rtype: bool
        """
        return _IMeshTools.IMeshTools_Context_DiscretizeFaces(self, *args)


    def GetEdgeDiscret(self, *args) -> "opencascade::handle< IMeshTools_ModelAlgo > const &":
        """
        * Gets instance of a tool to be used to discretize edges of a model.
        	:rtype: inline  opencascade::handle<IMeshTools_ModelAlgo>
        """
        return _IMeshTools.IMeshTools_Context_GetEdgeDiscret(self, *args)


    def GetFaceDiscret(self, *args) -> "opencascade::handle< IMeshTools_ModelAlgo > const &":
        """
        * Gets instance of meshing algorithm.
        	:rtype: inline  opencascade::handle<IMeshTools_ModelAlgo>
        """
        return _IMeshTools.IMeshTools_Context_GetFaceDiscret(self, *args)


    def GetModel(self, *args) -> "opencascade::handle< IMeshData_Model > const &":
        """
        * Returns discrete model of a shape.
        	:rtype: inline  opencascade::handle<IMeshData_Model>
        """
        return _IMeshTools.IMeshTools_Context_GetModel(self, *args)


    def GetModelBuilder(self, *args) -> "opencascade::handle< IMeshTools_ModelBuilder > const &":
        """
        * Gets instance of a tool to be used to build discrete model.
        	:rtype: inline  opencascade::handle<IMeshTools_ModelBuilder>
        """
        return _IMeshTools.IMeshTools_Context_GetModelBuilder(self, *args)


    def GetModelHealer(self, *args) -> "opencascade::handle< IMeshTools_ModelAlgo > const &":
        """
        * Gets instance of a tool to be used to heal discrete model.
        	:rtype: inline  opencascade::handle<IMeshTools_ModelAlgo>
        """
        return _IMeshTools.IMeshTools_Context_GetModelHealer(self, *args)


    def GetParameters(self, *args) -> "IMeshTools_Parameters const &":
        """
        * Gets parameters to be used for meshing.
        	:rtype: inline  IMeshTools_Parameters
        """
        return _IMeshTools.IMeshTools_Context_GetParameters(self, *args)


    def GetPostProcessor(self, *args) -> "opencascade::handle< IMeshTools_ModelAlgo > const &":
        """
        * Gets instance of post-processing algorithm.
        	:rtype: inline  opencascade::handle<IMeshTools_ModelAlgo>
        """
        return _IMeshTools.IMeshTools_Context_GetPostProcessor(self, *args)


    def GetPreProcessor(self, *args) -> "opencascade::handle< IMeshTools_ModelAlgo > const &":
        """
        * Gets instance of pre-processing algorithm.
        	:rtype: inline  opencascade::handle<IMeshTools_ModelAlgo>
        """
        return _IMeshTools.IMeshTools_Context_GetPreProcessor(self, *args)


    def HealModel(self, *args) -> "Standard_Boolean":
        """
        * Performs healing of discrete model built by DiscretizeEdges() method using assigned healing algorithm. returns True on success, False elsewhere.
        	:rtype: bool
        """
        return _IMeshTools.IMeshTools_Context_HealModel(self, *args)


    def __init__(self, *args):
        """
        * Constructor.
        	:rtype: None
        """
        _IMeshTools.IMeshTools_Context_swiginit(self, _IMeshTools.new_IMeshTools_Context(*args))

    def PostProcessModel(self, *args) -> "Standard_Boolean":
        """
        * Performs post-processing of discrete model using assigned algorithm. returns True on success, False elsewhere.
        	:rtype: bool
        """
        return _IMeshTools.IMeshTools_Context_PostProcessModel(self, *args)


    def PreProcessModel(self, *args) -> "Standard_Boolean":
        """
        * Performs pre-processing of discrete model using assigned algorithm. Performs auxiliary actions such as cleaning shape from old triangulation. returns True on success, False elsewhere.
        	:rtype: bool
        """
        return _IMeshTools.IMeshTools_Context_PreProcessModel(self, *args)


    def SetEdgeDiscret(self, *args) -> "void":
        """
        * Sets instance of a tool to be used to discretize edges of a model.
        	:param theEdgeDiscret:
        	:type theEdgeDiscret: IMeshTools_ModelAlgo
        	:rtype: inline void
        """
        return _IMeshTools.IMeshTools_Context_SetEdgeDiscret(self, *args)


    def SetFaceDiscret(self, *args) -> "void":
        """
        * Sets instance of meshing algorithm.
        	:param theFaceDiscret:
        	:type theFaceDiscret: IMeshTools_ModelAlgo
        	:rtype: inline void
        """
        return _IMeshTools.IMeshTools_Context_SetFaceDiscret(self, *args)


    def SetModelBuilder(self, *args) -> "void":
        """
        * Sets instance of a tool to be used to build discrete model.
        	:param theBuilder:
        	:type theBuilder: IMeshTools_ModelBuilder
        	:rtype: inline void
        """
        return _IMeshTools.IMeshTools_Context_SetModelBuilder(self, *args)


    def SetModelHealer(self, *args) -> "void":
        """
        * Sets instance of a tool to be used to heal discrete model.
        	:param theModelHealer:
        	:type theModelHealer: IMeshTools_ModelAlgo
        	:rtype: inline void
        """
        return _IMeshTools.IMeshTools_Context_SetModelHealer(self, *args)


    def SetPostProcessor(self, *args) -> "void":
        """
        * Sets instance of post-processing algorithm.
        	:param thePostProcessor:
        	:type thePostProcessor: IMeshTools_ModelAlgo
        	:rtype: inline void
        """
        return _IMeshTools.IMeshTools_Context_SetPostProcessor(self, *args)


    def SetPreProcessor(self, *args) -> "void":
        """
        * Sets instance of pre-processing algorithm.
        	:param thePreProcessor:
        	:type thePreProcessor: IMeshTools_ModelAlgo
        	:rtype: inline void
        """
        return _IMeshTools.IMeshTools_Context_SetPreProcessor(self, *args)



    @staticmethod
    def DownCast(t):
      return Handle_IMeshTools_Context_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _IMeshTools.delete_IMeshTools_Context
IMeshTools_Context.BuildModel = new_instancemethod(_IMeshTools.IMeshTools_Context_BuildModel, None, IMeshTools_Context)
IMeshTools_Context.ChangeParameters = new_instancemethod(_IMeshTools.IMeshTools_Context_ChangeParameters, None, IMeshTools_Context)
IMeshTools_Context.Clean = new_instancemethod(_IMeshTools.IMeshTools_Context_Clean, None, IMeshTools_Context)
IMeshTools_Context.DiscretizeEdges = new_instancemethod(_IMeshTools.IMeshTools_Context_DiscretizeEdges, None, IMeshTools_Context)
IMeshTools_Context.DiscretizeFaces = new_instancemethod(_IMeshTools.IMeshTools_Context_DiscretizeFaces, None, IMeshTools_Context)
IMeshTools_Context.GetEdgeDiscret = new_instancemethod(_IMeshTools.IMeshTools_Context_GetEdgeDiscret, None, IMeshTools_Context)
IMeshTools_Context.GetFaceDiscret = new_instancemethod(_IMeshTools.IMeshTools_Context_GetFaceDiscret, None, IMeshTools_Context)
IMeshTools_Context.GetModel = new_instancemethod(_IMeshTools.IMeshTools_Context_GetModel, None, IMeshTools_Context)
IMeshTools_Context.GetModelBuilder = new_instancemethod(_IMeshTools.IMeshTools_Context_GetModelBuilder, None, IMeshTools_Context)
IMeshTools_Context.GetModelHealer = new_instancemethod(_IMeshTools.IMeshTools_Context_GetModelHealer, None, IMeshTools_Context)
IMeshTools_Context.GetParameters = new_instancemethod(_IMeshTools.IMeshTools_Context_GetParameters, None, IMeshTools_Context)
IMeshTools_Context.GetPostProcessor = new_instancemethod(_IMeshTools.IMeshTools_Context_GetPostProcessor, None, IMeshTools_Context)
IMeshTools_Context.GetPreProcessor = new_instancemethod(_IMeshTools.IMeshTools_Context_GetPreProcessor, None, IMeshTools_Context)
IMeshTools_Context.HealModel = new_instancemethod(_IMeshTools.IMeshTools_Context_HealModel, None, IMeshTools_Context)
IMeshTools_Context.PostProcessModel = new_instancemethod(_IMeshTools.IMeshTools_Context_PostProcessModel, None, IMeshTools_Context)
IMeshTools_Context.PreProcessModel = new_instancemethod(_IMeshTools.IMeshTools_Context_PreProcessModel, None, IMeshTools_Context)
IMeshTools_Context.SetEdgeDiscret = new_instancemethod(_IMeshTools.IMeshTools_Context_SetEdgeDiscret, None, IMeshTools_Context)
IMeshTools_Context.SetFaceDiscret = new_instancemethod(_IMeshTools.IMeshTools_Context_SetFaceDiscret, None, IMeshTools_Context)
IMeshTools_Context.SetModelBuilder = new_instancemethod(_IMeshTools.IMeshTools_Context_SetModelBuilder, None, IMeshTools_Context)
IMeshTools_Context.SetModelHealer = new_instancemethod(_IMeshTools.IMeshTools_Context_SetModelHealer, None, IMeshTools_Context)
IMeshTools_Context.SetPostProcessor = new_instancemethod(_IMeshTools.IMeshTools_Context_SetPostProcessor, None, IMeshTools_Context)
IMeshTools_Context.SetPreProcessor = new_instancemethod(_IMeshTools.IMeshTools_Context_SetPreProcessor, None, IMeshTools_Context)
IMeshTools_Context_swigregister = _IMeshTools.IMeshTools_Context_swigregister
IMeshTools_Context_swigregister(IMeshTools_Context)

class IMeshTools_CurveTessellator(OCC.Core.Standard.Standard_Transient):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined")
    __repr__ = _swig_repr

    def PointsNb(self, *args) -> "Standard_Integer":
        """
        * Returns number of tessellation points.
        	:rtype: int
        """
        return _IMeshTools.IMeshTools_CurveTessellator_PointsNb(self, *args)


    def Value(self, *args) -> "Standard_Boolean":
        """
        * Returns parameters of solution with the given index. @param theIndex index of tessellation point. @param thePoint tessellation point. @param theParameter parameters on PCurve corresponded to the solution. returns True in case of valid result, false elewhere.
        	:param theIndex:
        	:type theIndex: int
        	:param thePoint:
        	:type thePoint: gp_Pnt
        	:param theParameter:
        	:type theParameter: float
        	:rtype: bool
        """
        return _IMeshTools.IMeshTools_CurveTessellator_Value(self, *args)



    @staticmethod
    def DownCast(t):
      return Handle_IMeshTools_CurveTessellator_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _IMeshTools.delete_IMeshTools_CurveTessellator
IMeshTools_CurveTessellator.PointsNb = new_instancemethod(_IMeshTools.IMeshTools_CurveTessellator_PointsNb, None, IMeshTools_CurveTessellator)
IMeshTools_CurveTessellator.Value = new_instancemethod(_IMeshTools.IMeshTools_CurveTessellator_Value, None, IMeshTools_CurveTessellator)
IMeshTools_CurveTessellator_swigregister = _IMeshTools.IMeshTools_CurveTessellator_swigregister
IMeshTools_CurveTessellator_swigregister(IMeshTools_CurveTessellator)

class IMeshTools_MeshAlgo(OCC.Core.Standard.Standard_Transient):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined")
    __repr__ = _swig_repr

    def Perform(self, *args) -> "void":
        """
        * Performs processing of the given face.
        	:param theDFace:
        	:type theDFace: IMeshData::IFaceHandle
        	:param theParameters:
        	:type theParameters: IMeshTools_Parameters
        	:rtype: void
        """
        return _IMeshTools.IMeshTools_MeshAlgo_Perform(self, *args)



    @staticmethod
    def DownCast(t):
      return Handle_IMeshTools_MeshAlgo_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _IMeshTools.delete_IMeshTools_MeshAlgo
IMeshTools_MeshAlgo.Perform = new_instancemethod(_IMeshTools.IMeshTools_MeshAlgo_Perform, None, IMeshTools_MeshAlgo)
IMeshTools_MeshAlgo_swigregister = _IMeshTools.IMeshTools_MeshAlgo_swigregister
IMeshTools_MeshAlgo_swigregister(IMeshTools_MeshAlgo)

class IMeshTools_MeshAlgoFactory(OCC.Core.Standard.Standard_Transient):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined")
    __repr__ = _swig_repr

    def GetAlgo(self, *args) -> "opencascade::handle< IMeshTools_MeshAlgo >":
        """
        * Creates instance of meshing algorithm for the given type of surface.
        	:param theSurfaceType:
        	:type theSurfaceType: GeomAbs_SurfaceType
        	:param theParameters:
        	:type theParameters: IMeshTools_Parameters
        	:rtype: opencascade::handle<IMeshTools_MeshAlgo>
        """
        return _IMeshTools.IMeshTools_MeshAlgoFactory_GetAlgo(self, *args)



    @staticmethod
    def DownCast(t):
      return Handle_IMeshTools_MeshAlgoFactory_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _IMeshTools.delete_IMeshTools_MeshAlgoFactory
IMeshTools_MeshAlgoFactory.GetAlgo = new_instancemethod(_IMeshTools.IMeshTools_MeshAlgoFactory_GetAlgo, None, IMeshTools_MeshAlgoFactory)
IMeshTools_MeshAlgoFactory_swigregister = _IMeshTools.IMeshTools_MeshAlgoFactory_swigregister
IMeshTools_MeshAlgoFactory_swigregister(IMeshTools_MeshAlgoFactory)

class IMeshTools_MeshBuilder(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def GetContext(self, *args) -> "opencascade::handle< IMeshTools_Context > const &":
        """
        * Gets context of algorithm.
        	:rtype: inline  opencascade::handle<IMeshTools_Context>
        """
        return _IMeshTools.IMeshTools_MeshBuilder_GetContext(self, *args)


    def __init__(self, *args):
        """
        * Constructor.
        	:rtype: None
        * Constructor.
        	:param theContext:
        	:type theContext: IMeshTools_Context
        	:rtype: None
        """
        _IMeshTools.IMeshTools_MeshBuilder_swiginit(self, _IMeshTools.new_IMeshTools_MeshBuilder(*args))

    def Perform(self, *args) -> "void":
        """
        * Performs meshing ot the shape using current context.
        	:rtype: void
        """
        return _IMeshTools.IMeshTools_MeshBuilder_Perform(self, *args)


    def SetContext(self, *args) -> "void":
        """
        * Sets context for algorithm.
        	:param theContext:
        	:type theContext: IMeshTools_Context
        	:rtype: inline void
        """
        return _IMeshTools.IMeshTools_MeshBuilder_SetContext(self, *args)


    __repr__ = _dumps_object

    __swig_destroy__ = _IMeshTools.delete_IMeshTools_MeshBuilder
IMeshTools_MeshBuilder.GetContext = new_instancemethod(_IMeshTools.IMeshTools_MeshBuilder_GetContext, None, IMeshTools_MeshBuilder)
IMeshTools_MeshBuilder.Perform = new_instancemethod(_IMeshTools.IMeshTools_MeshBuilder_Perform, None, IMeshTools_MeshBuilder)
IMeshTools_MeshBuilder.SetContext = new_instancemethod(_IMeshTools.IMeshTools_MeshBuilder_SetContext, None, IMeshTools_MeshBuilder)
IMeshTools_MeshBuilder_swigregister = _IMeshTools.IMeshTools_MeshBuilder_swigregister
IMeshTools_MeshBuilder_swigregister(IMeshTools_MeshBuilder)

class IMeshTools_ModelAlgo(OCC.Core.Standard.Standard_Transient):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined")
    __repr__ = _swig_repr

    def Perform(self, *args) -> "Standard_Boolean":
        """
        * Exceptions protected processing of the given model.
        	:param theModel:
        	:type theModel: IMeshData_Model
        	:param theParameters:
        	:type theParameters: IMeshTools_Parameters
        	:rtype: bool
        """
        return _IMeshTools.IMeshTools_ModelAlgo_Perform(self, *args)



    @staticmethod
    def DownCast(t):
      return Handle_IMeshTools_ModelAlgo_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _IMeshTools.delete_IMeshTools_ModelAlgo
IMeshTools_ModelAlgo.Perform = new_instancemethod(_IMeshTools.IMeshTools_ModelAlgo_Perform, None, IMeshTools_ModelAlgo)
IMeshTools_ModelAlgo_swigregister = _IMeshTools.IMeshTools_ModelAlgo_swigregister
IMeshTools_ModelAlgo_swigregister(IMeshTools_ModelAlgo)

class IMeshTools_ModelBuilder(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined")
    __repr__ = _swig_repr

    def Perform(self, *args) -> "opencascade::handle< IMeshData_Model >":
        """
        * Exceptions protected method to create discrete model for the given shape. Returns nullptr in case of failure.
        	:param theShape:
        	:type theShape: TopoDS_Shape
        	:param theParameters:
        	:type theParameters: IMeshTools_Parameters
        	:rtype: opencascade::handle<IMeshData_Model>
        """
        return _IMeshTools.IMeshTools_ModelBuilder_Perform(self, *args)


    __repr__ = _dumps_object

    __swig_destroy__ = _IMeshTools.delete_IMeshTools_ModelBuilder
IMeshTools_ModelBuilder.Perform = new_instancemethod(_IMeshTools.IMeshTools_ModelBuilder_Perform, None, IMeshTools_ModelBuilder)
IMeshTools_ModelBuilder_swigregister = _IMeshTools.IMeshTools_ModelBuilder_swigregister
IMeshTools_ModelBuilder_swigregister(IMeshTools_ModelBuilder)

class IMeshTools_Parameters(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr
    Angle = _swig_property(_IMeshTools.IMeshTools_Parameters_Angle_get, _IMeshTools.IMeshTools_Parameters_Angle_set)
    Deflection = _swig_property(_IMeshTools.IMeshTools_Parameters_Deflection_get, _IMeshTools.IMeshTools_Parameters_Deflection_set)
    AngleInterior = _swig_property(_IMeshTools.IMeshTools_Parameters_AngleInterior_get, _IMeshTools.IMeshTools_Parameters_AngleInterior_set)
    DeflectionInterior = _swig_property(_IMeshTools.IMeshTools_Parameters_DeflectionInterior_get, _IMeshTools.IMeshTools_Parameters_DeflectionInterior_set)
    MinSize = _swig_property(_IMeshTools.IMeshTools_Parameters_MinSize_get, _IMeshTools.IMeshTools_Parameters_MinSize_set)
    InParallel = _swig_property(_IMeshTools.IMeshTools_Parameters_InParallel_get, _IMeshTools.IMeshTools_Parameters_InParallel_set)
    Relative = _swig_property(_IMeshTools.IMeshTools_Parameters_Relative_get, _IMeshTools.IMeshTools_Parameters_Relative_set)
    InternalVerticesMode = _swig_property(_IMeshTools.IMeshTools_Parameters_InternalVerticesMode_get, _IMeshTools.IMeshTools_Parameters_InternalVerticesMode_set)
    ControlSurfaceDeflection = _swig_property(_IMeshTools.IMeshTools_Parameters_ControlSurfaceDeflection_get, _IMeshTools.IMeshTools_Parameters_ControlSurfaceDeflection_set)
    CleanModel = _swig_property(_IMeshTools.IMeshTools_Parameters_CleanModel_get, _IMeshTools.IMeshTools_Parameters_CleanModel_set)
    AdjustMinSize = _swig_property(_IMeshTools.IMeshTools_Parameters_AdjustMinSize_get, _IMeshTools.IMeshTools_Parameters_AdjustMinSize_set)

    def __init__(self, *args):
        """
        * Default constructor
        	:rtype: None
        """
        _IMeshTools.IMeshTools_Parameters_swiginit(self, _IMeshTools.new_IMeshTools_Parameters(*args))

    def RelMinSize(*args) -> "Standard_Real":
        """
        * Returns factor used to compute default value of MinSize (minimum mesh edge length) from deflection
        	:rtype: float
        """
        return _IMeshTools.IMeshTools_Parameters_RelMinSize(*args)

    RelMinSize = staticmethod(RelMinSize)

    __repr__ = _dumps_object

    __swig_destroy__ = _IMeshTools.delete_IMeshTools_Parameters
IMeshTools_Parameters_swigregister = _IMeshTools.IMeshTools_Parameters_swigregister
IMeshTools_Parameters_swigregister(IMeshTools_Parameters)

def IMeshTools_Parameters_RelMinSize(*args) -> "Standard_Real":
    """
    * Returns factor used to compute default value of MinSize (minimum mesh edge length) from deflection
    	:rtype: float
    """
    return _IMeshTools.IMeshTools_Parameters_RelMinSize(*args)

class IMeshTools_ShapeExplorer(OCC.Core.IMeshData.IMeshData_Shape):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def Accept(self, *args) -> "void":
        """
        * Starts exploring of a shape.
        	:param theVisitor:
        	:type theVisitor: IMeshTools_ShapeVisitor
        	:rtype: void
        """
        return _IMeshTools.IMeshTools_ShapeExplorer_Accept(self, *args)


    def __init__(self, *args):
        """
        * Constructor.
        	:param theShape:
        	:type theShape: TopoDS_Shape
        	:rtype: None
        """
        _IMeshTools.IMeshTools_ShapeExplorer_swiginit(self, _IMeshTools.new_IMeshTools_ShapeExplorer(*args))


    @staticmethod
    def DownCast(t):
      return Handle_IMeshTools_ShapeExplorer_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _IMeshTools.delete_IMeshTools_ShapeExplorer
IMeshTools_ShapeExplorer.Accept = new_instancemethod(_IMeshTools.IMeshTools_ShapeExplorer_Accept, None, IMeshTools_ShapeExplorer)
IMeshTools_ShapeExplorer_swigregister = _IMeshTools.IMeshTools_ShapeExplorer_swigregister
IMeshTools_ShapeExplorer_swigregister(IMeshTools_ShapeExplorer)

class IMeshTools_ShapeVisitor(OCC.Core.Standard.Standard_Transient):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined")
    __repr__ = _swig_repr

    def Visit(self, *args) -> "void":
        """
        * Handles TopoDS_Face object.
        	:param theFace:
        	:type theFace: TopoDS_Face
        	:rtype: void
        * Handles TopoDS_Edge object.
        	:param theEdge:
        	:type theEdge: TopoDS_Edge
        	:rtype: void
        """
        return _IMeshTools.IMeshTools_ShapeVisitor_Visit(self, *args)



    @staticmethod
    def DownCast(t):
      return Handle_IMeshTools_ShapeVisitor_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _IMeshTools.delete_IMeshTools_ShapeVisitor
IMeshTools_ShapeVisitor.Visit = new_instancemethod(_IMeshTools.IMeshTools_ShapeVisitor_Visit, None, IMeshTools_ShapeVisitor)
IMeshTools_ShapeVisitor_swigregister = _IMeshTools.IMeshTools_ShapeVisitor_swigregister
IMeshTools_ShapeVisitor_swigregister(IMeshTools_ShapeVisitor)


