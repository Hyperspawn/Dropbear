# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.12
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

"""
XmlMDF module, see official documentation at
https://www.opencascade.com/doc/occt-7.4.0/refman/html/package_xmlmdf.html
"""


from sys import version_info as _swig_python_version_info
if _swig_python_version_info >= (3, 0, 0):
    new_instancemethod = lambda func, inst, cls: _XmlMDF.SWIG_PyInstanceMethod_New(func)
else:
    from new import instancemethod as new_instancemethod
if _swig_python_version_info >= (2, 7, 0):
    def swig_import_helper():
        import importlib
        pkg = __name__.rpartition('.')[0]
        mname = '.'.join((pkg, '_XmlMDF')).lstrip('.')
        try:
            return importlib.import_module(mname)
        except ImportError:
            return importlib.import_module('_XmlMDF')
    _XmlMDF = swig_import_helper()
    del swig_import_helper
elif _swig_python_version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_XmlMDF', [dirname(__file__)])
        except ImportError:
            import _XmlMDF
            return _XmlMDF
        try:
            _mod = imp.load_module('_XmlMDF', fp, pathname, description)
        finally:
            if fp is not None:
                fp.close()
        return _mod
    _XmlMDF = swig_import_helper()
    del swig_import_helper
else:
    import _XmlMDF
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
    __swig_destroy__ = _XmlMDF.delete_SwigPyIterator
    def __iter__(self):
        return self
SwigPyIterator.value = new_instancemethod(_XmlMDF.SwigPyIterator_value, None, SwigPyIterator)
SwigPyIterator.incr = new_instancemethod(_XmlMDF.SwigPyIterator_incr, None, SwigPyIterator)
SwigPyIterator.decr = new_instancemethod(_XmlMDF.SwigPyIterator_decr, None, SwigPyIterator)
SwigPyIterator.distance = new_instancemethod(_XmlMDF.SwigPyIterator_distance, None, SwigPyIterator)
SwigPyIterator.equal = new_instancemethod(_XmlMDF.SwigPyIterator_equal, None, SwigPyIterator)
SwigPyIterator.copy = new_instancemethod(_XmlMDF.SwigPyIterator_copy, None, SwigPyIterator)
SwigPyIterator.next = new_instancemethod(_XmlMDF.SwigPyIterator_next, None, SwigPyIterator)
SwigPyIterator.__next__ = new_instancemethod(_XmlMDF.SwigPyIterator___next__, None, SwigPyIterator)
SwigPyIterator.previous = new_instancemethod(_XmlMDF.SwigPyIterator_previous, None, SwigPyIterator)
SwigPyIterator.advance = new_instancemethod(_XmlMDF.SwigPyIterator_advance, None, SwigPyIterator)
SwigPyIterator.__eq__ = new_instancemethod(_XmlMDF.SwigPyIterator___eq__, None, SwigPyIterator)
SwigPyIterator.__ne__ = new_instancemethod(_XmlMDF.SwigPyIterator___ne__, None, SwigPyIterator)
SwigPyIterator.__iadd__ = new_instancemethod(_XmlMDF.SwigPyIterator___iadd__, None, SwigPyIterator)
SwigPyIterator.__isub__ = new_instancemethod(_XmlMDF.SwigPyIterator___isub__, None, SwigPyIterator)
SwigPyIterator.__add__ = new_instancemethod(_XmlMDF.SwigPyIterator___add__, None, SwigPyIterator)
SwigPyIterator.__sub__ = new_instancemethod(_XmlMDF.SwigPyIterator___sub__, None, SwigPyIterator)
SwigPyIterator_swigregister = _XmlMDF.SwigPyIterator_swigregister
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
    return _XmlMDF.process_exception(error, method_name, class_name)
process_exception = _XmlMDF.process_exception

from six import with_metaclass
import warnings
from OCC.Wrapper.wrapper_utils import Proxy, deprecated

import OCC.Core.Standard
import OCC.Core.NCollection
import OCC.Core.Message
import OCC.Core.TDF
import OCC.Core.TCollection
import OCC.Core.TColStd
import OCC.Core.XmlObjMgt
import OCC.Core.LDOM
import OCC.Core.gp
import OCC.Core.Storage

def Handle_XmlMDF_ADriver_Create() -> "opencascade::handle< XmlMDF_ADriver >":
    return _XmlMDF.Handle_XmlMDF_ADriver_Create()
Handle_XmlMDF_ADriver_Create = _XmlMDF.Handle_XmlMDF_ADriver_Create

def Handle_XmlMDF_ADriver_DownCast(t: 'opencascade::handle< Standard_Transient > const &') -> "opencascade::handle< XmlMDF_ADriver >":
    return _XmlMDF.Handle_XmlMDF_ADriver_DownCast(t)
Handle_XmlMDF_ADriver_DownCast = _XmlMDF.Handle_XmlMDF_ADriver_DownCast

def Handle_XmlMDF_ADriver_IsNull(t: 'opencascade::handle< XmlMDF_ADriver > const &') -> "bool":
    return _XmlMDF.Handle_XmlMDF_ADriver_IsNull(t)
Handle_XmlMDF_ADriver_IsNull = _XmlMDF.Handle_XmlMDF_ADriver_IsNull

def Handle_XmlMDF_ADriverTable_Create() -> "opencascade::handle< XmlMDF_ADriverTable >":
    return _XmlMDF.Handle_XmlMDF_ADriverTable_Create()
Handle_XmlMDF_ADriverTable_Create = _XmlMDF.Handle_XmlMDF_ADriverTable_Create

def Handle_XmlMDF_ADriverTable_DownCast(t: 'opencascade::handle< Standard_Transient > const &') -> "opencascade::handle< XmlMDF_ADriverTable >":
    return _XmlMDF.Handle_XmlMDF_ADriverTable_DownCast(t)
Handle_XmlMDF_ADriverTable_DownCast = _XmlMDF.Handle_XmlMDF_ADriverTable_DownCast

def Handle_XmlMDF_ADriverTable_IsNull(t: 'opencascade::handle< XmlMDF_ADriverTable > const &') -> "bool":
    return _XmlMDF.Handle_XmlMDF_ADriverTable_IsNull(t)
Handle_XmlMDF_ADriverTable_IsNull = _XmlMDF.Handle_XmlMDF_ADriverTable_IsNull

def Handle_XmlMDF_ReferenceDriver_Create() -> "opencascade::handle< XmlMDF_ReferenceDriver >":
    return _XmlMDF.Handle_XmlMDF_ReferenceDriver_Create()
Handle_XmlMDF_ReferenceDriver_Create = _XmlMDF.Handle_XmlMDF_ReferenceDriver_Create

def Handle_XmlMDF_ReferenceDriver_DownCast(t: 'opencascade::handle< Standard_Transient > const &') -> "opencascade::handle< XmlMDF_ReferenceDriver >":
    return _XmlMDF.Handle_XmlMDF_ReferenceDriver_DownCast(t)
Handle_XmlMDF_ReferenceDriver_DownCast = _XmlMDF.Handle_XmlMDF_ReferenceDriver_DownCast

def Handle_XmlMDF_ReferenceDriver_IsNull(t: 'opencascade::handle< XmlMDF_ReferenceDriver > const &') -> "bool":
    return _XmlMDF.Handle_XmlMDF_ReferenceDriver_IsNull(t)
Handle_XmlMDF_ReferenceDriver_IsNull = _XmlMDF.Handle_XmlMDF_ReferenceDriver_IsNull

def Handle_XmlMDF_TagSourceDriver_Create() -> "opencascade::handle< XmlMDF_TagSourceDriver >":
    return _XmlMDF.Handle_XmlMDF_TagSourceDriver_Create()
Handle_XmlMDF_TagSourceDriver_Create = _XmlMDF.Handle_XmlMDF_TagSourceDriver_Create

def Handle_XmlMDF_TagSourceDriver_DownCast(t: 'opencascade::handle< Standard_Transient > const &') -> "opencascade::handle< XmlMDF_TagSourceDriver >":
    return _XmlMDF.Handle_XmlMDF_TagSourceDriver_DownCast(t)
Handle_XmlMDF_TagSourceDriver_DownCast = _XmlMDF.Handle_XmlMDF_TagSourceDriver_DownCast

def Handle_XmlMDF_TagSourceDriver_IsNull(t: 'opencascade::handle< XmlMDF_TagSourceDriver > const &') -> "bool":
    return _XmlMDF.Handle_XmlMDF_TagSourceDriver_IsNull(t)
Handle_XmlMDF_TagSourceDriver_IsNull = _XmlMDF.Handle_XmlMDF_TagSourceDriver_IsNull
class XmlMDF_MapOfDriver(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def __init__(self, *args):
        _XmlMDF.XmlMDF_MapOfDriver_swiginit(self, _XmlMDF.new_XmlMDF_MapOfDriver(*args))
    __swig_destroy__ = _XmlMDF.delete_XmlMDF_MapOfDriver
XmlMDF_MapOfDriver.begin = new_instancemethod(_XmlMDF.XmlMDF_MapOfDriver_begin, None, XmlMDF_MapOfDriver)
XmlMDF_MapOfDriver.end = new_instancemethod(_XmlMDF.XmlMDF_MapOfDriver_end, None, XmlMDF_MapOfDriver)
XmlMDF_MapOfDriver.cbegin = new_instancemethod(_XmlMDF.XmlMDF_MapOfDriver_cbegin, None, XmlMDF_MapOfDriver)
XmlMDF_MapOfDriver.cend = new_instancemethod(_XmlMDF.XmlMDF_MapOfDriver_cend, None, XmlMDF_MapOfDriver)
XmlMDF_MapOfDriver.Exchange = new_instancemethod(_XmlMDF.XmlMDF_MapOfDriver_Exchange, None, XmlMDF_MapOfDriver)
XmlMDF_MapOfDriver.Assign = new_instancemethod(_XmlMDF.XmlMDF_MapOfDriver_Assign, None, XmlMDF_MapOfDriver)
XmlMDF_MapOfDriver.Set = new_instancemethod(_XmlMDF.XmlMDF_MapOfDriver_Set, None, XmlMDF_MapOfDriver)
XmlMDF_MapOfDriver.ReSize = new_instancemethod(_XmlMDF.XmlMDF_MapOfDriver_ReSize, None, XmlMDF_MapOfDriver)
XmlMDF_MapOfDriver.Bind = new_instancemethod(_XmlMDF.XmlMDF_MapOfDriver_Bind, None, XmlMDF_MapOfDriver)
XmlMDF_MapOfDriver.Bound = new_instancemethod(_XmlMDF.XmlMDF_MapOfDriver_Bound, None, XmlMDF_MapOfDriver)
XmlMDF_MapOfDriver.IsBound = new_instancemethod(_XmlMDF.XmlMDF_MapOfDriver_IsBound, None, XmlMDF_MapOfDriver)
XmlMDF_MapOfDriver.UnBind = new_instancemethod(_XmlMDF.XmlMDF_MapOfDriver_UnBind, None, XmlMDF_MapOfDriver)
XmlMDF_MapOfDriver.Seek = new_instancemethod(_XmlMDF.XmlMDF_MapOfDriver_Seek, None, XmlMDF_MapOfDriver)
XmlMDF_MapOfDriver.Find = new_instancemethod(_XmlMDF.XmlMDF_MapOfDriver_Find, None, XmlMDF_MapOfDriver)
XmlMDF_MapOfDriver.ChangeSeek = new_instancemethod(_XmlMDF.XmlMDF_MapOfDriver_ChangeSeek, None, XmlMDF_MapOfDriver)
XmlMDF_MapOfDriver.ChangeFind = new_instancemethod(_XmlMDF.XmlMDF_MapOfDriver_ChangeFind, None, XmlMDF_MapOfDriver)
XmlMDF_MapOfDriver.__call__ = new_instancemethod(_XmlMDF.XmlMDF_MapOfDriver___call__, None, XmlMDF_MapOfDriver)
XmlMDF_MapOfDriver.Clear = new_instancemethod(_XmlMDF.XmlMDF_MapOfDriver_Clear, None, XmlMDF_MapOfDriver)
XmlMDF_MapOfDriver.Size = new_instancemethod(_XmlMDF.XmlMDF_MapOfDriver_Size, None, XmlMDF_MapOfDriver)
XmlMDF_MapOfDriver_swigregister = _XmlMDF.XmlMDF_MapOfDriver_swigregister
XmlMDF_MapOfDriver_swigregister(XmlMDF_MapOfDriver)

class XmlMDF_TypeADriverMap(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def __init__(self, *args):
        _XmlMDF.XmlMDF_TypeADriverMap_swiginit(self, _XmlMDF.new_XmlMDF_TypeADriverMap(*args))
    __swig_destroy__ = _XmlMDF.delete_XmlMDF_TypeADriverMap
XmlMDF_TypeADriverMap.begin = new_instancemethod(_XmlMDF.XmlMDF_TypeADriverMap_begin, None, XmlMDF_TypeADriverMap)
XmlMDF_TypeADriverMap.end = new_instancemethod(_XmlMDF.XmlMDF_TypeADriverMap_end, None, XmlMDF_TypeADriverMap)
XmlMDF_TypeADriverMap.cbegin = new_instancemethod(_XmlMDF.XmlMDF_TypeADriverMap_cbegin, None, XmlMDF_TypeADriverMap)
XmlMDF_TypeADriverMap.cend = new_instancemethod(_XmlMDF.XmlMDF_TypeADriverMap_cend, None, XmlMDF_TypeADriverMap)
XmlMDF_TypeADriverMap.Exchange = new_instancemethod(_XmlMDF.XmlMDF_TypeADriverMap_Exchange, None, XmlMDF_TypeADriverMap)
XmlMDF_TypeADriverMap.Assign = new_instancemethod(_XmlMDF.XmlMDF_TypeADriverMap_Assign, None, XmlMDF_TypeADriverMap)
XmlMDF_TypeADriverMap.Set = new_instancemethod(_XmlMDF.XmlMDF_TypeADriverMap_Set, None, XmlMDF_TypeADriverMap)
XmlMDF_TypeADriverMap.ReSize = new_instancemethod(_XmlMDF.XmlMDF_TypeADriverMap_ReSize, None, XmlMDF_TypeADriverMap)
XmlMDF_TypeADriverMap.Bind = new_instancemethod(_XmlMDF.XmlMDF_TypeADriverMap_Bind, None, XmlMDF_TypeADriverMap)
XmlMDF_TypeADriverMap.Bound = new_instancemethod(_XmlMDF.XmlMDF_TypeADriverMap_Bound, None, XmlMDF_TypeADriverMap)
XmlMDF_TypeADriverMap.IsBound = new_instancemethod(_XmlMDF.XmlMDF_TypeADriverMap_IsBound, None, XmlMDF_TypeADriverMap)
XmlMDF_TypeADriverMap.UnBind = new_instancemethod(_XmlMDF.XmlMDF_TypeADriverMap_UnBind, None, XmlMDF_TypeADriverMap)
XmlMDF_TypeADriverMap.Seek = new_instancemethod(_XmlMDF.XmlMDF_TypeADriverMap_Seek, None, XmlMDF_TypeADriverMap)
XmlMDF_TypeADriverMap.Find = new_instancemethod(_XmlMDF.XmlMDF_TypeADriverMap_Find, None, XmlMDF_TypeADriverMap)
XmlMDF_TypeADriverMap.ChangeSeek = new_instancemethod(_XmlMDF.XmlMDF_TypeADriverMap_ChangeSeek, None, XmlMDF_TypeADriverMap)
XmlMDF_TypeADriverMap.ChangeFind = new_instancemethod(_XmlMDF.XmlMDF_TypeADriverMap_ChangeFind, None, XmlMDF_TypeADriverMap)
XmlMDF_TypeADriverMap.__call__ = new_instancemethod(_XmlMDF.XmlMDF_TypeADriverMap___call__, None, XmlMDF_TypeADriverMap)
XmlMDF_TypeADriverMap.Clear = new_instancemethod(_XmlMDF.XmlMDF_TypeADriverMap_Clear, None, XmlMDF_TypeADriverMap)
XmlMDF_TypeADriverMap.Size = new_instancemethod(_XmlMDF.XmlMDF_TypeADriverMap_Size, None, XmlMDF_TypeADriverMap)
XmlMDF_TypeADriverMap_swigregister = _XmlMDF.XmlMDF_TypeADriverMap_swigregister
XmlMDF_TypeADriverMap_swigregister(XmlMDF_TypeADriverMap)

class xmlmdf(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def AddDrivers(*args) -> "void":
        """
        * Adds the attribute storage drivers to <aDriverSeq>.
        	:param aDriverTable:
        	:type aDriverTable: XmlMDF_ADriverTable
        	:param theMessageDriver:
        	:type theMessageDriver: Message_Messenger
        	:rtype: void
        """
        return _XmlMDF.xmlmdf_AddDrivers(*args)

    AddDrivers = staticmethod(AddDrivers)

    def FromTo(*args) -> "Standard_Boolean":
        """
        * Translates a transient <aSource> into a persistent <aTarget>.
        	:param aSource:
        	:type aSource: TDF_Data
        	:param aTarget:
        	:type aTarget: XmlObjMgt_Element
        	:param aReloc:
        	:type aReloc: XmlObjMgt_SRelocationTable
        	:param aDrivers:
        	:type aDrivers: XmlMDF_ADriverTable
        	:rtype: void
        * Translates a persistent <aSource> into a transient <aTarget>. Returns True if completed successfully (False on error)
        	:param aSource:
        	:type aSource: XmlObjMgt_Element
        	:param aTarget:
        	:type aTarget: TDF_Data
        	:param aReloc:
        	:type aReloc: XmlObjMgt_RRelocationTable
        	:param aDrivers:
        	:type aDrivers: XmlMDF_ADriverTable
        	:rtype: bool
        """
        return _XmlMDF.xmlmdf_FromTo(*args)

    FromTo = staticmethod(FromTo)

    __repr__ = _dumps_object


    def __init__(self):
        _XmlMDF.xmlmdf_swiginit(self, _XmlMDF.new_xmlmdf())
    __swig_destroy__ = _XmlMDF.delete_xmlmdf
xmlmdf_swigregister = _XmlMDF.xmlmdf_swigregister
xmlmdf_swigregister(xmlmdf)

def xmlmdf_AddDrivers(*args) -> "void":
    """
    * Adds the attribute storage drivers to <aDriverSeq>.
    	:param aDriverTable:
    	:type aDriverTable: XmlMDF_ADriverTable
    	:param theMessageDriver:
    	:type theMessageDriver: Message_Messenger
    	:rtype: void
    """
    return _XmlMDF.xmlmdf_AddDrivers(*args)

def xmlmdf_FromTo(*args) -> "Standard_Boolean":
    """
    * Translates a transient <aSource> into a persistent <aTarget>.
    	:param aSource:
    	:type aSource: TDF_Data
    	:param aTarget:
    	:type aTarget: XmlObjMgt_Element
    	:param aReloc:
    	:type aReloc: XmlObjMgt_SRelocationTable
    	:param aDrivers:
    	:type aDrivers: XmlMDF_ADriverTable
    	:rtype: void
    * Translates a persistent <aSource> into a transient <aTarget>. Returns True if completed successfully (False on error)
    	:param aSource:
    	:type aSource: XmlObjMgt_Element
    	:param aTarget:
    	:type aTarget: TDF_Data
    	:param aReloc:
    	:type aReloc: XmlObjMgt_RRelocationTable
    	:param aDrivers:
    	:type aDrivers: XmlMDF_ADriverTable
    	:rtype: bool
    """
    return _XmlMDF.xmlmdf_FromTo(*args)

class XmlMDF_ADriver(OCC.Core.Standard.Standard_Transient):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined")
    __repr__ = _swig_repr

    def NewEmpty(self, *args) -> "opencascade::handle< TDF_Attribute >":
        """
        * Creates a new attribute from TDF.
        	:rtype: opencascade::handle<TDF_Attribute>
        """
        return _XmlMDF.XmlMDF_ADriver_NewEmpty(self, *args)


    def Paste(self, *args) -> "void":
        """
        * Translate the contents of <aSource> and put it into <aTarget>, using the relocation table <aRelocTable> to keep the sharings.
        	:param aSource:
        	:type aSource: XmlObjMgt_Persistent
        	:param aTarget:
        	:type aTarget: TDF_Attribute
        	:param aRelocTable:
        	:type aRelocTable: XmlObjMgt_RRelocationTable
        	:rtype: bool
        * Translate the contents of <aSource> and put it into <aTarget>, using the relocation table <aRelocTable> to keep the sharings.
        	:param aSource:
        	:type aSource: TDF_Attribute
        	:param aTarget:
        	:type aTarget: XmlObjMgt_Persistent
        	:param aRelocTable:
        	:type aRelocTable: XmlObjMgt_SRelocationTable
        	:rtype: void
        """
        return _XmlMDF.XmlMDF_ADriver_Paste(self, *args)


    def SourceType(self, *args) -> "opencascade::handle< Standard_Type >":
        """
        * Returns the type of source object, inheriting from Attribute from TDF.
        	:rtype: opencascade::handle<Standard_Type>
        """
        return _XmlMDF.XmlMDF_ADriver_SourceType(self, *args)


    def TypeName(self, *args) -> "TCollection_AsciiString const &":
        """
        * Returns the full XML tag name (including NS prefix)
        	:rtype: TCollection_AsciiString
        """
        return _XmlMDF.XmlMDF_ADriver_TypeName(self, *args)


    def VersionNumber(self, *args) -> "Standard_Integer":
        """
        * Returns the version number from which the driver is available.
        	:rtype: int
        """
        return _XmlMDF.XmlMDF_ADriver_VersionNumber(self, *args)



    @staticmethod
    def DownCast(t):
      return Handle_XmlMDF_ADriver_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _XmlMDF.delete_XmlMDF_ADriver
XmlMDF_ADriver.NewEmpty = new_instancemethod(_XmlMDF.XmlMDF_ADriver_NewEmpty, None, XmlMDF_ADriver)
XmlMDF_ADriver.Paste = new_instancemethod(_XmlMDF.XmlMDF_ADriver_Paste, None, XmlMDF_ADriver)
XmlMDF_ADriver.SourceType = new_instancemethod(_XmlMDF.XmlMDF_ADriver_SourceType, None, XmlMDF_ADriver)
XmlMDF_ADriver.TypeName = new_instancemethod(_XmlMDF.XmlMDF_ADriver_TypeName, None, XmlMDF_ADriver)
XmlMDF_ADriver.VersionNumber = new_instancemethod(_XmlMDF.XmlMDF_ADriver_VersionNumber, None, XmlMDF_ADriver)
XmlMDF_ADriver_swigregister = _XmlMDF.XmlMDF_ADriver_swigregister
XmlMDF_ADriver_swigregister(XmlMDF_ADriver)

class XmlMDF_ADriverTable(OCC.Core.Standard.Standard_Transient):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def AddDriver(self, *args) -> "void":
        """
        * Sets a translation driver: <aDriver>.
        	:param anHDriver:
        	:type anHDriver: XmlMDF_ADriver
        	:rtype: None
        """
        return _XmlMDF.XmlMDF_ADriverTable_AddDriver(self, *args)


    def GetDriver(self, *args) -> "Standard_Boolean":
        """
        * Gets a driver <aDriver> according to <aType> //! Returns True if a driver is found; false otherwise.
        	:param aType:
        	:type aType: Standard_Type
        	:param anHDriver:
        	:type anHDriver: XmlMDF_ADriver
        	:rtype: bool
        """
        return _XmlMDF.XmlMDF_ADriverTable_GetDriver(self, *args)


    def GetDrivers(self, *args) -> "XmlMDF_TypeADriverMap const &":
        """
        * Gets a map of drivers.
        	:rtype: XmlMDF_TypeADriverMap
        """
        return _XmlMDF.XmlMDF_ADriverTable_GetDrivers(self, *args)


    def __init__(self, *args):
        """
        * Creates a mutable ADriverTable from XmlMDF.
        	:rtype: None
        """
        _XmlMDF.XmlMDF_ADriverTable_swiginit(self, _XmlMDF.new_XmlMDF_ADriverTable(*args))


    @staticmethod
    def DownCast(t):
      return Handle_XmlMDF_ADriverTable_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _XmlMDF.delete_XmlMDF_ADriverTable
XmlMDF_ADriverTable.AddDriver = new_instancemethod(_XmlMDF.XmlMDF_ADriverTable_AddDriver, None, XmlMDF_ADriverTable)
XmlMDF_ADriverTable.GetDriver = new_instancemethod(_XmlMDF.XmlMDF_ADriverTable_GetDriver, None, XmlMDF_ADriverTable)
XmlMDF_ADriverTable.GetDrivers = new_instancemethod(_XmlMDF.XmlMDF_ADriverTable_GetDrivers, None, XmlMDF_ADriverTable)
XmlMDF_ADriverTable_swigregister = _XmlMDF.XmlMDF_ADriverTable_swigregister
XmlMDF_ADriverTable_swigregister(XmlMDF_ADriverTable)

class XmlMDF_ReferenceDriver(XmlMDF_ADriver):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def Paste(self, *args) -> "void":
        """
        :param Source:
        	:type Source: XmlObjMgt_Persistent
        	:param Target:
        	:type Target: TDF_Attribute
        	:param RelocTable:
        	:type RelocTable: XmlObjMgt_RRelocationTable
        	:rtype: bool
        :param Source:
        	:type Source: TDF_Attribute
        	:param Target:
        	:type Target: XmlObjMgt_Persistent
        	:param RelocTable:
        	:type RelocTable: XmlObjMgt_SRelocationTable
        	:rtype: None
        """
        return _XmlMDF.XmlMDF_ReferenceDriver_Paste(self, *args)


    def __init__(self, *args):
        """
        :param theMessageDriver:
        	:type theMessageDriver: Message_Messenger
        	:rtype: None
        """
        _XmlMDF.XmlMDF_ReferenceDriver_swiginit(self, _XmlMDF.new_XmlMDF_ReferenceDriver(*args))


    @staticmethod
    def DownCast(t):
      return Handle_XmlMDF_ReferenceDriver_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _XmlMDF.delete_XmlMDF_ReferenceDriver
XmlMDF_ReferenceDriver.Paste = new_instancemethod(_XmlMDF.XmlMDF_ReferenceDriver_Paste, None, XmlMDF_ReferenceDriver)
XmlMDF_ReferenceDriver_swigregister = _XmlMDF.XmlMDF_ReferenceDriver_swigregister
XmlMDF_ReferenceDriver_swigregister(XmlMDF_ReferenceDriver)

class XmlMDF_TagSourceDriver(XmlMDF_ADriver):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def Paste(self, *args) -> "void":
        """
        :param Source:
        	:type Source: XmlObjMgt_Persistent
        	:param Target:
        	:type Target: TDF_Attribute
        	:param RelocTable:
        	:type RelocTable: XmlObjMgt_RRelocationTable
        	:rtype: bool
        :param Source:
        	:type Source: TDF_Attribute
        	:param Target:
        	:type Target: XmlObjMgt_Persistent
        	:param RelocTable:
        	:type RelocTable: XmlObjMgt_SRelocationTable
        	:rtype: None
        """
        return _XmlMDF.XmlMDF_TagSourceDriver_Paste(self, *args)


    def __init__(self, *args):
        """
        :param theMessageDriver:
        	:type theMessageDriver: Message_Messenger
        	:rtype: None
        """
        _XmlMDF.XmlMDF_TagSourceDriver_swiginit(self, _XmlMDF.new_XmlMDF_TagSourceDriver(*args))


    @staticmethod
    def DownCast(t):
      return Handle_XmlMDF_TagSourceDriver_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _XmlMDF.delete_XmlMDF_TagSourceDriver
XmlMDF_TagSourceDriver.Paste = new_instancemethod(_XmlMDF.XmlMDF_TagSourceDriver_Paste, None, XmlMDF_TagSourceDriver)
XmlMDF_TagSourceDriver_swigregister = _XmlMDF.XmlMDF_TagSourceDriver_swigregister
XmlMDF_TagSourceDriver_swigregister(XmlMDF_TagSourceDriver)


