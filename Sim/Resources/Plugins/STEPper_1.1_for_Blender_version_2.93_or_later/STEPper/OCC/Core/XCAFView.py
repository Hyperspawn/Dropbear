# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.12
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

"""
XCAFView module, see official documentation at
https://www.opencascade.com/doc/occt-7.4.0/refman/html/package_xcafview.html
"""


from sys import version_info as _swig_python_version_info
if _swig_python_version_info >= (3, 0, 0):
    new_instancemethod = lambda func, inst, cls: _XCAFView.SWIG_PyInstanceMethod_New(func)
else:
    from new import instancemethod as new_instancemethod
if _swig_python_version_info >= (2, 7, 0):
    def swig_import_helper():
        import importlib
        pkg = __name__.rpartition('.')[0]
        mname = '.'.join((pkg, '_XCAFView')).lstrip('.')
        try:
            return importlib.import_module(mname)
        except ImportError:
            return importlib.import_module('_XCAFView')
    _XCAFView = swig_import_helper()
    del swig_import_helper
elif _swig_python_version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_XCAFView', [dirname(__file__)])
        except ImportError:
            import _XCAFView
            return _XCAFView
        try:
            _mod = imp.load_module('_XCAFView', fp, pathname, description)
        finally:
            if fp is not None:
                fp.close()
        return _mod
    _XCAFView = swig_import_helper()
    del swig_import_helper
else:
    import _XCAFView
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
    __swig_destroy__ = _XCAFView.delete_SwigPyIterator
    def __iter__(self):
        return self
SwigPyIterator.value = new_instancemethod(_XCAFView.SwigPyIterator_value, None, SwigPyIterator)
SwigPyIterator.incr = new_instancemethod(_XCAFView.SwigPyIterator_incr, None, SwigPyIterator)
SwigPyIterator.decr = new_instancemethod(_XCAFView.SwigPyIterator_decr, None, SwigPyIterator)
SwigPyIterator.distance = new_instancemethod(_XCAFView.SwigPyIterator_distance, None, SwigPyIterator)
SwigPyIterator.equal = new_instancemethod(_XCAFView.SwigPyIterator_equal, None, SwigPyIterator)
SwigPyIterator.copy = new_instancemethod(_XCAFView.SwigPyIterator_copy, None, SwigPyIterator)
SwigPyIterator.next = new_instancemethod(_XCAFView.SwigPyIterator_next, None, SwigPyIterator)
SwigPyIterator.__next__ = new_instancemethod(_XCAFView.SwigPyIterator___next__, None, SwigPyIterator)
SwigPyIterator.previous = new_instancemethod(_XCAFView.SwigPyIterator_previous, None, SwigPyIterator)
SwigPyIterator.advance = new_instancemethod(_XCAFView.SwigPyIterator_advance, None, SwigPyIterator)
SwigPyIterator.__eq__ = new_instancemethod(_XCAFView.SwigPyIterator___eq__, None, SwigPyIterator)
SwigPyIterator.__ne__ = new_instancemethod(_XCAFView.SwigPyIterator___ne__, None, SwigPyIterator)
SwigPyIterator.__iadd__ = new_instancemethod(_XCAFView.SwigPyIterator___iadd__, None, SwigPyIterator)
SwigPyIterator.__isub__ = new_instancemethod(_XCAFView.SwigPyIterator___isub__, None, SwigPyIterator)
SwigPyIterator.__add__ = new_instancemethod(_XCAFView.SwigPyIterator___add__, None, SwigPyIterator)
SwigPyIterator.__sub__ = new_instancemethod(_XCAFView.SwigPyIterator___sub__, None, SwigPyIterator)
SwigPyIterator_swigregister = _XCAFView.SwigPyIterator_swigregister
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
    return _XCAFView.process_exception(error, method_name, class_name)
process_exception = _XCAFView.process_exception

from six import with_metaclass
import warnings
from OCC.Wrapper.wrapper_utils import Proxy, deprecated

import OCC.Core.Standard
import OCC.Core.NCollection
import OCC.Core.TCollection
import OCC.Core.gp
XCAFView_ProjectionType_NoCamera = _XCAFView.XCAFView_ProjectionType_NoCamera
XCAFView_ProjectionType_Parallel = _XCAFView.XCAFView_ProjectionType_Parallel
XCAFView_ProjectionType_Central = _XCAFView.XCAFView_ProjectionType_Central

def Handle_XCAFView_Object_Create() -> "opencascade::handle< XCAFView_Object >":
    return _XCAFView.Handle_XCAFView_Object_Create()
Handle_XCAFView_Object_Create = _XCAFView.Handle_XCAFView_Object_Create

def Handle_XCAFView_Object_DownCast(t: 'opencascade::handle< Standard_Transient > const &') -> "opencascade::handle< XCAFView_Object >":
    return _XCAFView.Handle_XCAFView_Object_DownCast(t)
Handle_XCAFView_Object_DownCast = _XCAFView.Handle_XCAFView_Object_DownCast

def Handle_XCAFView_Object_IsNull(t: 'opencascade::handle< XCAFView_Object > const &') -> "bool":
    return _XCAFView.Handle_XCAFView_Object_IsNull(t)
Handle_XCAFView_Object_IsNull = _XCAFView.Handle_XCAFView_Object_IsNull
class XCAFView_Object(OCC.Core.Standard.Standard_Transient):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def BackPlaneDistance(self, *args) -> "Standard_Real":
        """:rtype: float"""
        return _XCAFView.XCAFView_Object_BackPlaneDistance(self, *args)


    def ClippingExpression(self, *args) -> "opencascade::handle< TCollection_HAsciiString >":
        """:rtype: opencascade::handle<TCollection_HAsciiString>"""
        return _XCAFView.XCAFView_Object_ClippingExpression(self, *args)


    def CreateGDTPoints(self, *args) -> "void":
        """
        :param theLenght:
        	:type theLenght: int
        	:rtype: None
        """
        return _XCAFView.XCAFView_Object_CreateGDTPoints(self, *args)


    def FrontPlaneDistance(self, *args) -> "Standard_Real":
        """:rtype: float"""
        return _XCAFView.XCAFView_Object_FrontPlaneDistance(self, *args)


    def GDTPoint(self, *args) -> "gp_Pnt":
        """
        :param theIndex:
        	:type theIndex: int
        	:rtype: gp_Pnt
        """
        return _XCAFView.XCAFView_Object_GDTPoint(self, *args)


    def HasBackPlaneClipping(self, *args) -> "Standard_Boolean":
        """:rtype: bool"""
        return _XCAFView.XCAFView_Object_HasBackPlaneClipping(self, *args)


    def HasFrontPlaneClipping(self, *args) -> "Standard_Boolean":
        """:rtype: bool"""
        return _XCAFView.XCAFView_Object_HasFrontPlaneClipping(self, *args)


    def HasGDTPoints(self, *args) -> "Standard_Boolean":
        """:rtype: bool"""
        return _XCAFView.XCAFView_Object_HasGDTPoints(self, *args)


    def HasViewVolumeSidesClipping(self, *args) -> "Standard_Boolean":
        """:rtype: bool"""
        return _XCAFView.XCAFView_Object_HasViewVolumeSidesClipping(self, *args)


    def Name(self, *args) -> "opencascade::handle< TCollection_HAsciiString >":
        """:rtype: opencascade::handle<TCollection_HAsciiString>"""
        return _XCAFView.XCAFView_Object_Name(self, *args)


    def NbGDTPoints(self, *args) -> "Standard_Integer":
        """:rtype: int"""
        return _XCAFView.XCAFView_Object_NbGDTPoints(self, *args)


    def ProjectionPoint(self, *args) -> "gp_Pnt":
        """:rtype: gp_Pnt"""
        return _XCAFView.XCAFView_Object_ProjectionPoint(self, *args)


    def SetBackPlaneDistance(self, *args) -> "void":
        """
        :param theDistance:
        	:type theDistance: float
        	:rtype: None
        """
        return _XCAFView.XCAFView_Object_SetBackPlaneDistance(self, *args)


    def SetClippingExpression(self, *args) -> "void":
        """
        :param theExpression:
        	:type theExpression: TCollection_HAsciiString
        	:rtype: None
        """
        return _XCAFView.XCAFView_Object_SetClippingExpression(self, *args)


    def SetFrontPlaneDistance(self, *args) -> "void":
        """
        :param theDistance:
        	:type theDistance: float
        	:rtype: None
        """
        return _XCAFView.XCAFView_Object_SetFrontPlaneDistance(self, *args)


    def SetGDTPoint(self, *args) -> "void":
        """
        :param theIndex:
        	:type theIndex: int
        	:param thePoint:
        	:type thePoint: gp_Pnt
        	:rtype: None
        """
        return _XCAFView.XCAFView_Object_SetGDTPoint(self, *args)


    def SetName(self, *args) -> "void":
        """
        :param theName:
        	:type theName: TCollection_HAsciiString
        	:rtype: None
        """
        return _XCAFView.XCAFView_Object_SetName(self, *args)


    def SetProjectionPoint(self, *args) -> "void":
        """
        :param thePoint:
        	:type thePoint: gp_Pnt
        	:rtype: None
        """
        return _XCAFView.XCAFView_Object_SetProjectionPoint(self, *args)


    def SetType(self, *args) -> "void":
        """
        :param theType:
        	:type theType: XCAFView_ProjectionType
        	:rtype: None
        """
        return _XCAFView.XCAFView_Object_SetType(self, *args)


    def SetUpDirection(self, *args) -> "void":
        """
        :param theDirection:
        	:type theDirection: gp_Dir
        	:rtype: None
        """
        return _XCAFView.XCAFView_Object_SetUpDirection(self, *args)


    def SetViewDirection(self, *args) -> "void":
        """
        :param theDirection:
        	:type theDirection: gp_Dir
        	:rtype: None
        """
        return _XCAFView.XCAFView_Object_SetViewDirection(self, *args)


    def SetViewVolumeSidesClipping(self, *args) -> "void":
        """
        :param theViewVolumeSidesClipping:
        	:type theViewVolumeSidesClipping: bool
        	:rtype: None
        """
        return _XCAFView.XCAFView_Object_SetViewVolumeSidesClipping(self, *args)


    def SetWindowHorizontalSize(self, *args) -> "void":
        """
        :param theSize:
        	:type theSize: float
        	:rtype: None
        """
        return _XCAFView.XCAFView_Object_SetWindowHorizontalSize(self, *args)


    def SetWindowVerticalSize(self, *args) -> "void":
        """
        :param theSize:
        	:type theSize: float
        	:rtype: None
        """
        return _XCAFView.XCAFView_Object_SetWindowVerticalSize(self, *args)


    def SetZoomFactor(self, *args) -> "void":
        """
        :param theZoomFactor:
        	:type theZoomFactor: float
        	:rtype: None
        """
        return _XCAFView.XCAFView_Object_SetZoomFactor(self, *args)


    def Type(self, *args) -> "XCAFView_ProjectionType":
        """:rtype: XCAFView_ProjectionType"""
        return _XCAFView.XCAFView_Object_Type(self, *args)


    def UnsetBackPlaneClipping(self, *args) -> "void":
        """:rtype: None"""
        return _XCAFView.XCAFView_Object_UnsetBackPlaneClipping(self, *args)


    def UnsetFrontPlaneClipping(self, *args) -> "void":
        """:rtype: None"""
        return _XCAFView.XCAFView_Object_UnsetFrontPlaneClipping(self, *args)


    def UpDirection(self, *args) -> "gp_Dir":
        """:rtype: gp_Dir"""
        return _XCAFView.XCAFView_Object_UpDirection(self, *args)


    def ViewDirection(self, *args) -> "gp_Dir":
        """:rtype: gp_Dir"""
        return _XCAFView.XCAFView_Object_ViewDirection(self, *args)


    def WindowHorizontalSize(self, *args) -> "Standard_Real":
        """:rtype: float"""
        return _XCAFView.XCAFView_Object_WindowHorizontalSize(self, *args)


    def WindowVerticalSize(self, *args) -> "Standard_Real":
        """:rtype: float"""
        return _XCAFView.XCAFView_Object_WindowVerticalSize(self, *args)


    def __init__(self, *args):
        """
        :rtype: None
        :param theObj:
        	:type theObj: XCAFView_Object
        	:rtype: None
        """
        _XCAFView.XCAFView_Object_swiginit(self, _XCAFView.new_XCAFView_Object(*args))

    def ZoomFactor(self, *args) -> "Standard_Real":
        """:rtype: float"""
        return _XCAFView.XCAFView_Object_ZoomFactor(self, *args)



    @staticmethod
    def DownCast(t):
      return Handle_XCAFView_Object_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _XCAFView.delete_XCAFView_Object
XCAFView_Object.BackPlaneDistance = new_instancemethod(_XCAFView.XCAFView_Object_BackPlaneDistance, None, XCAFView_Object)
XCAFView_Object.ClippingExpression = new_instancemethod(_XCAFView.XCAFView_Object_ClippingExpression, None, XCAFView_Object)
XCAFView_Object.CreateGDTPoints = new_instancemethod(_XCAFView.XCAFView_Object_CreateGDTPoints, None, XCAFView_Object)
XCAFView_Object.FrontPlaneDistance = new_instancemethod(_XCAFView.XCAFView_Object_FrontPlaneDistance, None, XCAFView_Object)
XCAFView_Object.GDTPoint = new_instancemethod(_XCAFView.XCAFView_Object_GDTPoint, None, XCAFView_Object)
XCAFView_Object.HasBackPlaneClipping = new_instancemethod(_XCAFView.XCAFView_Object_HasBackPlaneClipping, None, XCAFView_Object)
XCAFView_Object.HasFrontPlaneClipping = new_instancemethod(_XCAFView.XCAFView_Object_HasFrontPlaneClipping, None, XCAFView_Object)
XCAFView_Object.HasGDTPoints = new_instancemethod(_XCAFView.XCAFView_Object_HasGDTPoints, None, XCAFView_Object)
XCAFView_Object.HasViewVolumeSidesClipping = new_instancemethod(_XCAFView.XCAFView_Object_HasViewVolumeSidesClipping, None, XCAFView_Object)
XCAFView_Object.Name = new_instancemethod(_XCAFView.XCAFView_Object_Name, None, XCAFView_Object)
XCAFView_Object.NbGDTPoints = new_instancemethod(_XCAFView.XCAFView_Object_NbGDTPoints, None, XCAFView_Object)
XCAFView_Object.ProjectionPoint = new_instancemethod(_XCAFView.XCAFView_Object_ProjectionPoint, None, XCAFView_Object)
XCAFView_Object.SetBackPlaneDistance = new_instancemethod(_XCAFView.XCAFView_Object_SetBackPlaneDistance, None, XCAFView_Object)
XCAFView_Object.SetClippingExpression = new_instancemethod(_XCAFView.XCAFView_Object_SetClippingExpression, None, XCAFView_Object)
XCAFView_Object.SetFrontPlaneDistance = new_instancemethod(_XCAFView.XCAFView_Object_SetFrontPlaneDistance, None, XCAFView_Object)
XCAFView_Object.SetGDTPoint = new_instancemethod(_XCAFView.XCAFView_Object_SetGDTPoint, None, XCAFView_Object)
XCAFView_Object.SetName = new_instancemethod(_XCAFView.XCAFView_Object_SetName, None, XCAFView_Object)
XCAFView_Object.SetProjectionPoint = new_instancemethod(_XCAFView.XCAFView_Object_SetProjectionPoint, None, XCAFView_Object)
XCAFView_Object.SetType = new_instancemethod(_XCAFView.XCAFView_Object_SetType, None, XCAFView_Object)
XCAFView_Object.SetUpDirection = new_instancemethod(_XCAFView.XCAFView_Object_SetUpDirection, None, XCAFView_Object)
XCAFView_Object.SetViewDirection = new_instancemethod(_XCAFView.XCAFView_Object_SetViewDirection, None, XCAFView_Object)
XCAFView_Object.SetViewVolumeSidesClipping = new_instancemethod(_XCAFView.XCAFView_Object_SetViewVolumeSidesClipping, None, XCAFView_Object)
XCAFView_Object.SetWindowHorizontalSize = new_instancemethod(_XCAFView.XCAFView_Object_SetWindowHorizontalSize, None, XCAFView_Object)
XCAFView_Object.SetWindowVerticalSize = new_instancemethod(_XCAFView.XCAFView_Object_SetWindowVerticalSize, None, XCAFView_Object)
XCAFView_Object.SetZoomFactor = new_instancemethod(_XCAFView.XCAFView_Object_SetZoomFactor, None, XCAFView_Object)
XCAFView_Object.Type = new_instancemethod(_XCAFView.XCAFView_Object_Type, None, XCAFView_Object)
XCAFView_Object.UnsetBackPlaneClipping = new_instancemethod(_XCAFView.XCAFView_Object_UnsetBackPlaneClipping, None, XCAFView_Object)
XCAFView_Object.UnsetFrontPlaneClipping = new_instancemethod(_XCAFView.XCAFView_Object_UnsetFrontPlaneClipping, None, XCAFView_Object)
XCAFView_Object.UpDirection = new_instancemethod(_XCAFView.XCAFView_Object_UpDirection, None, XCAFView_Object)
XCAFView_Object.ViewDirection = new_instancemethod(_XCAFView.XCAFView_Object_ViewDirection, None, XCAFView_Object)
XCAFView_Object.WindowHorizontalSize = new_instancemethod(_XCAFView.XCAFView_Object_WindowHorizontalSize, None, XCAFView_Object)
XCAFView_Object.WindowVerticalSize = new_instancemethod(_XCAFView.XCAFView_Object_WindowVerticalSize, None, XCAFView_Object)
XCAFView_Object.ZoomFactor = new_instancemethod(_XCAFView.XCAFView_Object_ZoomFactor, None, XCAFView_Object)
XCAFView_Object_swigregister = _XCAFView.XCAFView_Object_swigregister
XCAFView_Object_swigregister(XCAFView_Object)


