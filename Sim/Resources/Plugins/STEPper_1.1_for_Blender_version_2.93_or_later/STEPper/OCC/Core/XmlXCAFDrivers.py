# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.12
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

"""
XmlXCAFDrivers module, see official documentation at
https://www.opencascade.com/doc/occt-7.4.0/refman/html/package_xmlxcafdrivers.html
"""


from sys import version_info as _swig_python_version_info
if _swig_python_version_info >= (3, 0, 0):
    new_instancemethod = lambda func, inst, cls: _XmlXCAFDrivers.SWIG_PyInstanceMethod_New(func)
else:
    from new import instancemethod as new_instancemethod
if _swig_python_version_info >= (2, 7, 0):
    def swig_import_helper():
        import importlib
        pkg = __name__.rpartition('.')[0]
        mname = '.'.join((pkg, '_XmlXCAFDrivers')).lstrip('.')
        try:
            return importlib.import_module(mname)
        except ImportError:
            return importlib.import_module('_XmlXCAFDrivers')
    _XmlXCAFDrivers = swig_import_helper()
    del swig_import_helper
elif _swig_python_version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_XmlXCAFDrivers', [dirname(__file__)])
        except ImportError:
            import _XmlXCAFDrivers
            return _XmlXCAFDrivers
        try:
            _mod = imp.load_module('_XmlXCAFDrivers', fp, pathname, description)
        finally:
            if fp is not None:
                fp.close()
        return _mod
    _XmlXCAFDrivers = swig_import_helper()
    del swig_import_helper
else:
    import _XmlXCAFDrivers
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
    __swig_destroy__ = _XmlXCAFDrivers.delete_SwigPyIterator
    def __iter__(self):
        return self
SwigPyIterator.value = new_instancemethod(_XmlXCAFDrivers.SwigPyIterator_value, None, SwigPyIterator)
SwigPyIterator.incr = new_instancemethod(_XmlXCAFDrivers.SwigPyIterator_incr, None, SwigPyIterator)
SwigPyIterator.decr = new_instancemethod(_XmlXCAFDrivers.SwigPyIterator_decr, None, SwigPyIterator)
SwigPyIterator.distance = new_instancemethod(_XmlXCAFDrivers.SwigPyIterator_distance, None, SwigPyIterator)
SwigPyIterator.equal = new_instancemethod(_XmlXCAFDrivers.SwigPyIterator_equal, None, SwigPyIterator)
SwigPyIterator.copy = new_instancemethod(_XmlXCAFDrivers.SwigPyIterator_copy, None, SwigPyIterator)
SwigPyIterator.next = new_instancemethod(_XmlXCAFDrivers.SwigPyIterator_next, None, SwigPyIterator)
SwigPyIterator.__next__ = new_instancemethod(_XmlXCAFDrivers.SwigPyIterator___next__, None, SwigPyIterator)
SwigPyIterator.previous = new_instancemethod(_XmlXCAFDrivers.SwigPyIterator_previous, None, SwigPyIterator)
SwigPyIterator.advance = new_instancemethod(_XmlXCAFDrivers.SwigPyIterator_advance, None, SwigPyIterator)
SwigPyIterator.__eq__ = new_instancemethod(_XmlXCAFDrivers.SwigPyIterator___eq__, None, SwigPyIterator)
SwigPyIterator.__ne__ = new_instancemethod(_XmlXCAFDrivers.SwigPyIterator___ne__, None, SwigPyIterator)
SwigPyIterator.__iadd__ = new_instancemethod(_XmlXCAFDrivers.SwigPyIterator___iadd__, None, SwigPyIterator)
SwigPyIterator.__isub__ = new_instancemethod(_XmlXCAFDrivers.SwigPyIterator___isub__, None, SwigPyIterator)
SwigPyIterator.__add__ = new_instancemethod(_XmlXCAFDrivers.SwigPyIterator___add__, None, SwigPyIterator)
SwigPyIterator.__sub__ = new_instancemethod(_XmlXCAFDrivers.SwigPyIterator___sub__, None, SwigPyIterator)
SwigPyIterator_swigregister = _XmlXCAFDrivers.SwigPyIterator_swigregister
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
    return _XmlXCAFDrivers.process_exception(error, method_name, class_name)
process_exception = _XmlXCAFDrivers.process_exception

from six import with_metaclass
import warnings
from OCC.Wrapper.wrapper_utils import Proxy, deprecated

import OCC.Core.Standard
import OCC.Core.NCollection
import OCC.Core.TDocStd
import OCC.Core.TDF
import OCC.Core.TCollection
import OCC.Core.TColStd
import OCC.Core.CDF
import OCC.Core.CDM
import OCC.Core.Message
import OCC.Core.Resource
import OCC.Core.PCDM
import OCC.Core.Storage
import OCC.Core.XmlDrivers
import OCC.Core.XmlMDF
import OCC.Core.XmlObjMgt
import OCC.Core.LDOM
import OCC.Core.gp
import OCC.Core.XmlLDrivers

def Handle_XmlXCAFDrivers_DocumentRetrievalDriver_Create() -> "opencascade::handle< XmlXCAFDrivers_DocumentRetrievalDriver >":
    return _XmlXCAFDrivers.Handle_XmlXCAFDrivers_DocumentRetrievalDriver_Create()
Handle_XmlXCAFDrivers_DocumentRetrievalDriver_Create = _XmlXCAFDrivers.Handle_XmlXCAFDrivers_DocumentRetrievalDriver_Create

def Handle_XmlXCAFDrivers_DocumentRetrievalDriver_DownCast(t: 'opencascade::handle< Standard_Transient > const &') -> "opencascade::handle< XmlXCAFDrivers_DocumentRetrievalDriver >":
    return _XmlXCAFDrivers.Handle_XmlXCAFDrivers_DocumentRetrievalDriver_DownCast(t)
Handle_XmlXCAFDrivers_DocumentRetrievalDriver_DownCast = _XmlXCAFDrivers.Handle_XmlXCAFDrivers_DocumentRetrievalDriver_DownCast

def Handle_XmlXCAFDrivers_DocumentRetrievalDriver_IsNull(t: 'opencascade::handle< XmlXCAFDrivers_DocumentRetrievalDriver > const &') -> "bool":
    return _XmlXCAFDrivers.Handle_XmlXCAFDrivers_DocumentRetrievalDriver_IsNull(t)
Handle_XmlXCAFDrivers_DocumentRetrievalDriver_IsNull = _XmlXCAFDrivers.Handle_XmlXCAFDrivers_DocumentRetrievalDriver_IsNull

def Handle_XmlXCAFDrivers_DocumentStorageDriver_Create() -> "opencascade::handle< XmlXCAFDrivers_DocumentStorageDriver >":
    return _XmlXCAFDrivers.Handle_XmlXCAFDrivers_DocumentStorageDriver_Create()
Handle_XmlXCAFDrivers_DocumentStorageDriver_Create = _XmlXCAFDrivers.Handle_XmlXCAFDrivers_DocumentStorageDriver_Create

def Handle_XmlXCAFDrivers_DocumentStorageDriver_DownCast(t: 'opencascade::handle< Standard_Transient > const &') -> "opencascade::handle< XmlXCAFDrivers_DocumentStorageDriver >":
    return _XmlXCAFDrivers.Handle_XmlXCAFDrivers_DocumentStorageDriver_DownCast(t)
Handle_XmlXCAFDrivers_DocumentStorageDriver_DownCast = _XmlXCAFDrivers.Handle_XmlXCAFDrivers_DocumentStorageDriver_DownCast

def Handle_XmlXCAFDrivers_DocumentStorageDriver_IsNull(t: 'opencascade::handle< XmlXCAFDrivers_DocumentStorageDriver > const &') -> "bool":
    return _XmlXCAFDrivers.Handle_XmlXCAFDrivers_DocumentStorageDriver_IsNull(t)
Handle_XmlXCAFDrivers_DocumentStorageDriver_IsNull = _XmlXCAFDrivers.Handle_XmlXCAFDrivers_DocumentStorageDriver_IsNull
class xmlxcafdrivers(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def DefineFormat(*args) -> "void":
        """
        * Defines format 'XmlXCAF' and registers its read and write drivers in the specified application
        	:param theApp:
        	:type theApp: TDocStd_Application
        	:rtype: void
        """
        return _XmlXCAFDrivers.xmlxcafdrivers_DefineFormat(*args)

    DefineFormat = staticmethod(DefineFormat)

    def Factory(*args) -> "opencascade::handle< Standard_Transient > const &":
        """
        * Depending from the ID, returns a list of storage or retrieval attribute drivers. Used for plugin. //! Standard data model drivers =========================== 47b0b826-d931-11d1-b5da-00a0c9064368 Transient-Persistent 47b0b827-d931-11d1-b5da-00a0c9064368 Persistent-Transient //! XCAF data model drivers ================================= ed8793f8-3142-11d4-b9b5-0060b0ee281b Transient-Persistent ed8793f9-3142-11d4-b9b5-0060b0ee281b Persistent-Transient ed8793fa-3142-11d4-b9b5-0060b0ee281b XCAFSchema
        	:param aGUID:
        	:type aGUID: Standard_GUID
        	:rtype: opencascade::handle<Standard_Transient>
        """
        return _XmlXCAFDrivers.xmlxcafdrivers_Factory(*args)

    Factory = staticmethod(Factory)

    __repr__ = _dumps_object


    def __init__(self):
        _XmlXCAFDrivers.xmlxcafdrivers_swiginit(self, _XmlXCAFDrivers.new_xmlxcafdrivers())
    __swig_destroy__ = _XmlXCAFDrivers.delete_xmlxcafdrivers
xmlxcafdrivers_swigregister = _XmlXCAFDrivers.xmlxcafdrivers_swigregister
xmlxcafdrivers_swigregister(xmlxcafdrivers)

def xmlxcafdrivers_DefineFormat(*args) -> "void":
    """
    * Defines format 'XmlXCAF' and registers its read and write drivers in the specified application
    	:param theApp:
    	:type theApp: TDocStd_Application
    	:rtype: void
    """
    return _XmlXCAFDrivers.xmlxcafdrivers_DefineFormat(*args)

def xmlxcafdrivers_Factory(*args) -> "opencascade::handle< Standard_Transient > const &":
    """
    * Depending from the ID, returns a list of storage or retrieval attribute drivers. Used for plugin. //! Standard data model drivers =========================== 47b0b826-d931-11d1-b5da-00a0c9064368 Transient-Persistent 47b0b827-d931-11d1-b5da-00a0c9064368 Persistent-Transient //! XCAF data model drivers ================================= ed8793f8-3142-11d4-b9b5-0060b0ee281b Transient-Persistent ed8793f9-3142-11d4-b9b5-0060b0ee281b Persistent-Transient ed8793fa-3142-11d4-b9b5-0060b0ee281b XCAFSchema
    	:param aGUID:
    	:type aGUID: Standard_GUID
    	:rtype: opencascade::handle<Standard_Transient>
    """
    return _XmlXCAFDrivers.xmlxcafdrivers_Factory(*args)

class XmlXCAFDrivers_DocumentRetrievalDriver(OCC.Core.XmlDrivers.XmlDrivers_DocumentRetrievalDriver):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def __init__(self, *args):
        """:rtype: None"""
        _XmlXCAFDrivers.XmlXCAFDrivers_DocumentRetrievalDriver_swiginit(self, _XmlXCAFDrivers.new_XmlXCAFDrivers_DocumentRetrievalDriver(*args))


    @staticmethod
    def DownCast(t):
      return Handle_XmlXCAFDrivers_DocumentRetrievalDriver_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _XmlXCAFDrivers.delete_XmlXCAFDrivers_DocumentRetrievalDriver
XmlXCAFDrivers_DocumentRetrievalDriver_swigregister = _XmlXCAFDrivers.XmlXCAFDrivers_DocumentRetrievalDriver_swigregister
XmlXCAFDrivers_DocumentRetrievalDriver_swigregister(XmlXCAFDrivers_DocumentRetrievalDriver)

class XmlXCAFDrivers_DocumentStorageDriver(OCC.Core.XmlDrivers.XmlDrivers_DocumentStorageDriver):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def __init__(self, *args):
        """
        :param theCopyright:
        	:type theCopyright: TCollection_ExtendedString
        	:rtype: None
        """
        _XmlXCAFDrivers.XmlXCAFDrivers_DocumentStorageDriver_swiginit(self, _XmlXCAFDrivers.new_XmlXCAFDrivers_DocumentStorageDriver(*args))


    @staticmethod
    def DownCast(t):
      return Handle_XmlXCAFDrivers_DocumentStorageDriver_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _XmlXCAFDrivers.delete_XmlXCAFDrivers_DocumentStorageDriver
XmlXCAFDrivers_DocumentStorageDriver_swigregister = _XmlXCAFDrivers.XmlXCAFDrivers_DocumentStorageDriver_swigregister
XmlXCAFDrivers_DocumentStorageDriver_swigregister(XmlXCAFDrivers_DocumentStorageDriver)



