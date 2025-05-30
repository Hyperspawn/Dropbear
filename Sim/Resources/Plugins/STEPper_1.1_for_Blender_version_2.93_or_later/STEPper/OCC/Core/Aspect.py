# This file was automatically generated by SWIG (http://www.swig.org).
# Version 4.0.1
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

"""
Aspect module, see official documentation at
https://www.opencascade.com/doc/occt-7.4.0/refman/html/package_aspect.html
"""

from sys import version_info as _swig_python_version_info
if _swig_python_version_info < (2, 7, 0):
    raise RuntimeError("Python 2.7 or later required")

# Import the low-level C/C++ module
if __package__ or "." in __name__:
    from . import _Aspect
else:
    import _Aspect

try:
    import builtins as __builtin__
except ImportError:
    import __builtin__

_swig_new_instance_method = _Aspect.SWIG_PyInstanceMethod_New
_swig_new_static_method = _Aspect.SWIG_PyStaticMethod_New

def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except __builtin__.Exception:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)


def _swig_setattr_nondynamic_instance_variable(set):
    def set_instance_attr(self, name, value):
        if name == "thisown":
            self.this.own(value)
        elif name == "this":
            set(self, name, value)
        elif hasattr(self, name) and isinstance(getattr(type(self), name), property):
            set(self, name, value)
        else:
            raise AttributeError("You cannot add instance attributes to %s" % self)
    return set_instance_attr


def _swig_setattr_nondynamic_class_variable(set):
    def set_class_attr(cls, name, value):
        if hasattr(cls, name) and not isinstance(getattr(cls, name), property):
            set(cls, name, value)
        else:
            raise AttributeError("You cannot add class attributes to %s" % cls)
    return set_class_attr


def _swig_add_metaclass(metaclass):
    """Class decorator for adding a metaclass to a SWIG wrapped class - a slimmed down version of six.add_metaclass"""
    def wrapper(cls):
        return metaclass(cls.__name__, cls.__bases__, cls.__dict__.copy())
    return wrapper


class _SwigNonDynamicMeta(type):
    """Meta class to enforce nondynamic attributes (no new attributes) for a class"""
    __setattr__ = _swig_setattr_nondynamic_class_variable(type.__setattr__)


class SwigPyIterator(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined - class is abstract")
    __repr__ = _swig_repr
    __swig_destroy__ = _Aspect.delete_SwigPyIterator
    value = _swig_new_instance_method(_Aspect.SwigPyIterator_value)
    incr = _swig_new_instance_method(_Aspect.SwigPyIterator_incr)
    decr = _swig_new_instance_method(_Aspect.SwigPyIterator_decr)
    distance = _swig_new_instance_method(_Aspect.SwigPyIterator_distance)
    equal = _swig_new_instance_method(_Aspect.SwigPyIterator_equal)
    copy = _swig_new_instance_method(_Aspect.SwigPyIterator_copy)
    next = _swig_new_instance_method(_Aspect.SwigPyIterator_next)
    __next__ = _swig_new_instance_method(_Aspect.SwigPyIterator___next__)
    previous = _swig_new_instance_method(_Aspect.SwigPyIterator_previous)
    advance = _swig_new_instance_method(_Aspect.SwigPyIterator_advance)
    __eq__ = _swig_new_instance_method(_Aspect.SwigPyIterator___eq__)
    __ne__ = _swig_new_instance_method(_Aspect.SwigPyIterator___ne__)
    __iadd__ = _swig_new_instance_method(_Aspect.SwigPyIterator___iadd__)
    __isub__ = _swig_new_instance_method(_Aspect.SwigPyIterator___isub__)
    __add__ = _swig_new_instance_method(_Aspect.SwigPyIterator___add__)
    __sub__ = _swig_new_instance_method(_Aspect.SwigPyIterator___sub__)
    def __iter__(self):
        return self

# Register SwigPyIterator in _Aspect:
_Aspect.SwigPyIterator_swigregister(SwigPyIterator)


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


from six import with_metaclass
import warnings
from OCC.Wrapper.wrapper_utils import Proxy, deprecated

import OCC.Core.Standard
import OCC.Core.NCollection
import OCC.Core.Quantity
import OCC.Core.TCollection
Aspect_VKey_UNKNOWN = _Aspect.Aspect_VKey_UNKNOWN
Aspect_VKey_A = _Aspect.Aspect_VKey_A
Aspect_VKey_B = _Aspect.Aspect_VKey_B
Aspect_VKey_C = _Aspect.Aspect_VKey_C
Aspect_VKey_D = _Aspect.Aspect_VKey_D
Aspect_VKey_E = _Aspect.Aspect_VKey_E
Aspect_VKey_F = _Aspect.Aspect_VKey_F
Aspect_VKey_G = _Aspect.Aspect_VKey_G
Aspect_VKey_H = _Aspect.Aspect_VKey_H
Aspect_VKey_I = _Aspect.Aspect_VKey_I
Aspect_VKey_J = _Aspect.Aspect_VKey_J
Aspect_VKey_K = _Aspect.Aspect_VKey_K
Aspect_VKey_L = _Aspect.Aspect_VKey_L
Aspect_VKey_M = _Aspect.Aspect_VKey_M
Aspect_VKey_N = _Aspect.Aspect_VKey_N
Aspect_VKey_O = _Aspect.Aspect_VKey_O
Aspect_VKey_P = _Aspect.Aspect_VKey_P
Aspect_VKey_Q = _Aspect.Aspect_VKey_Q
Aspect_VKey_R = _Aspect.Aspect_VKey_R
Aspect_VKey_S = _Aspect.Aspect_VKey_S
Aspect_VKey_T = _Aspect.Aspect_VKey_T
Aspect_VKey_U = _Aspect.Aspect_VKey_U
Aspect_VKey_V = _Aspect.Aspect_VKey_V
Aspect_VKey_W = _Aspect.Aspect_VKey_W
Aspect_VKey_X = _Aspect.Aspect_VKey_X
Aspect_VKey_Y = _Aspect.Aspect_VKey_Y
Aspect_VKey_Z = _Aspect.Aspect_VKey_Z
Aspect_VKey_0 = _Aspect.Aspect_VKey_0
Aspect_VKey_1 = _Aspect.Aspect_VKey_1
Aspect_VKey_2 = _Aspect.Aspect_VKey_2
Aspect_VKey_3 = _Aspect.Aspect_VKey_3
Aspect_VKey_4 = _Aspect.Aspect_VKey_4
Aspect_VKey_5 = _Aspect.Aspect_VKey_5
Aspect_VKey_6 = _Aspect.Aspect_VKey_6
Aspect_VKey_7 = _Aspect.Aspect_VKey_7
Aspect_VKey_8 = _Aspect.Aspect_VKey_8
Aspect_VKey_9 = _Aspect.Aspect_VKey_9
Aspect_VKey_F1 = _Aspect.Aspect_VKey_F1
Aspect_VKey_F2 = _Aspect.Aspect_VKey_F2
Aspect_VKey_F3 = _Aspect.Aspect_VKey_F3
Aspect_VKey_F4 = _Aspect.Aspect_VKey_F4
Aspect_VKey_F5 = _Aspect.Aspect_VKey_F5
Aspect_VKey_F6 = _Aspect.Aspect_VKey_F6
Aspect_VKey_F7 = _Aspect.Aspect_VKey_F7
Aspect_VKey_F8 = _Aspect.Aspect_VKey_F8
Aspect_VKey_F9 = _Aspect.Aspect_VKey_F9
Aspect_VKey_F10 = _Aspect.Aspect_VKey_F10
Aspect_VKey_F11 = _Aspect.Aspect_VKey_F11
Aspect_VKey_F12 = _Aspect.Aspect_VKey_F12
Aspect_VKey_Up = _Aspect.Aspect_VKey_Up
Aspect_VKey_Down = _Aspect.Aspect_VKey_Down
Aspect_VKey_Left = _Aspect.Aspect_VKey_Left
Aspect_VKey_Right = _Aspect.Aspect_VKey_Right
Aspect_VKey_Plus = _Aspect.Aspect_VKey_Plus
Aspect_VKey_Minus = _Aspect.Aspect_VKey_Minus
Aspect_VKey_Equal = _Aspect.Aspect_VKey_Equal
Aspect_VKey_PageUp = _Aspect.Aspect_VKey_PageUp
Aspect_VKey_PageDown = _Aspect.Aspect_VKey_PageDown
Aspect_VKey_Home = _Aspect.Aspect_VKey_Home
Aspect_VKey_End = _Aspect.Aspect_VKey_End
Aspect_VKey_Escape = _Aspect.Aspect_VKey_Escape
Aspect_VKey_Back = _Aspect.Aspect_VKey_Back
Aspect_VKey_Enter = _Aspect.Aspect_VKey_Enter
Aspect_VKey_Backspace = _Aspect.Aspect_VKey_Backspace
Aspect_VKey_Space = _Aspect.Aspect_VKey_Space
Aspect_VKey_Delete = _Aspect.Aspect_VKey_Delete
Aspect_VKey_Tilde = _Aspect.Aspect_VKey_Tilde
Aspect_VKey_Tab = _Aspect.Aspect_VKey_Tab
Aspect_VKey_Comma = _Aspect.Aspect_VKey_Comma
Aspect_VKey_Period = _Aspect.Aspect_VKey_Period
Aspect_VKey_Semicolon = _Aspect.Aspect_VKey_Semicolon
Aspect_VKey_Slash = _Aspect.Aspect_VKey_Slash
Aspect_VKey_BracketLeft = _Aspect.Aspect_VKey_BracketLeft
Aspect_VKey_Backslash = _Aspect.Aspect_VKey_Backslash
Aspect_VKey_BracketRight = _Aspect.Aspect_VKey_BracketRight
Aspect_VKey_Apostrophe = _Aspect.Aspect_VKey_Apostrophe
Aspect_VKey_Numlock = _Aspect.Aspect_VKey_Numlock
Aspect_VKey_Scroll = _Aspect.Aspect_VKey_Scroll
Aspect_VKey_Numpad0 = _Aspect.Aspect_VKey_Numpad0
Aspect_VKey_Numpad1 = _Aspect.Aspect_VKey_Numpad1
Aspect_VKey_Numpad2 = _Aspect.Aspect_VKey_Numpad2
Aspect_VKey_Numpad3 = _Aspect.Aspect_VKey_Numpad3
Aspect_VKey_Numpad4 = _Aspect.Aspect_VKey_Numpad4
Aspect_VKey_Numpad5 = _Aspect.Aspect_VKey_Numpad5
Aspect_VKey_Numpad6 = _Aspect.Aspect_VKey_Numpad6
Aspect_VKey_Numpad7 = _Aspect.Aspect_VKey_Numpad7
Aspect_VKey_Numpad8 = _Aspect.Aspect_VKey_Numpad8
Aspect_VKey_Numpad9 = _Aspect.Aspect_VKey_Numpad9
Aspect_VKey_NumpadMultiply = _Aspect.Aspect_VKey_NumpadMultiply
Aspect_VKey_NumpadAdd = _Aspect.Aspect_VKey_NumpadAdd
Aspect_VKey_NumpadSubtract = _Aspect.Aspect_VKey_NumpadSubtract
Aspect_VKey_NumpadDivide = _Aspect.Aspect_VKey_NumpadDivide
Aspect_VKey_MediaNextTrack = _Aspect.Aspect_VKey_MediaNextTrack
Aspect_VKey_MediaPreviousTrack = _Aspect.Aspect_VKey_MediaPreviousTrack
Aspect_VKey_MediaStop = _Aspect.Aspect_VKey_MediaStop
Aspect_VKey_MediaPlayPause = _Aspect.Aspect_VKey_MediaPlayPause
Aspect_VKey_VolumeMute = _Aspect.Aspect_VKey_VolumeMute
Aspect_VKey_VolumeDown = _Aspect.Aspect_VKey_VolumeDown
Aspect_VKey_VolumeUp = _Aspect.Aspect_VKey_VolumeUp
Aspect_VKey_BrowserBack = _Aspect.Aspect_VKey_BrowserBack
Aspect_VKey_BrowserForward = _Aspect.Aspect_VKey_BrowserForward
Aspect_VKey_BrowserRefresh = _Aspect.Aspect_VKey_BrowserRefresh
Aspect_VKey_BrowserStop = _Aspect.Aspect_VKey_BrowserStop
Aspect_VKey_BrowserSearch = _Aspect.Aspect_VKey_BrowserSearch
Aspect_VKey_BrowserFavorites = _Aspect.Aspect_VKey_BrowserFavorites
Aspect_VKey_BrowserHome = _Aspect.Aspect_VKey_BrowserHome
Aspect_VKey_Shift = _Aspect.Aspect_VKey_Shift
Aspect_VKey_Control = _Aspect.Aspect_VKey_Control
Aspect_VKey_Alt = _Aspect.Aspect_VKey_Alt
Aspect_VKey_Menu = _Aspect.Aspect_VKey_Menu
Aspect_VKey_Meta = _Aspect.Aspect_VKey_Meta
Aspect_VKey_NavInteract = _Aspect.Aspect_VKey_NavInteract
Aspect_VKey_NavForward = _Aspect.Aspect_VKey_NavForward
Aspect_VKey_NavBackward = _Aspect.Aspect_VKey_NavBackward
Aspect_VKey_NavSlideLeft = _Aspect.Aspect_VKey_NavSlideLeft
Aspect_VKey_NavSlideRight = _Aspect.Aspect_VKey_NavSlideRight
Aspect_VKey_NavSlideUp = _Aspect.Aspect_VKey_NavSlideUp
Aspect_VKey_NavSlideDown = _Aspect.Aspect_VKey_NavSlideDown
Aspect_VKey_NavRollCCW = _Aspect.Aspect_VKey_NavRollCCW
Aspect_VKey_NavRollCW = _Aspect.Aspect_VKey_NavRollCW
Aspect_VKey_NavLookLeft = _Aspect.Aspect_VKey_NavLookLeft
Aspect_VKey_NavLookRight = _Aspect.Aspect_VKey_NavLookRight
Aspect_VKey_NavLookUp = _Aspect.Aspect_VKey_NavLookUp
Aspect_VKey_NavLookDown = _Aspect.Aspect_VKey_NavLookDown
Aspect_VKey_NavCrouch = _Aspect.Aspect_VKey_NavCrouch
Aspect_VKey_NavJump = _Aspect.Aspect_VKey_NavJump
Aspect_VKey_NavThrustForward = _Aspect.Aspect_VKey_NavThrustForward
Aspect_VKey_NavThrustBackward = _Aspect.Aspect_VKey_NavThrustBackward
Aspect_VKey_NavThrustStop = _Aspect.Aspect_VKey_NavThrustStop
Aspect_VKey_NavSpeedIncrease = _Aspect.Aspect_VKey_NavSpeedIncrease
Aspect_VKey_NavSpeedDecrease = _Aspect.Aspect_VKey_NavSpeedDecrease
Aspect_VKey_Lower = _Aspect.Aspect_VKey_Lower
Aspect_VKey_ModifiersLower = _Aspect.Aspect_VKey_ModifiersLower
Aspect_VKey_ModifiersUpper = _Aspect.Aspect_VKey_ModifiersUpper
Aspect_VKey_NavigationKeysLower = _Aspect.Aspect_VKey_NavigationKeysLower
Aspect_VKey_NavigationKeysUpper = _Aspect.Aspect_VKey_NavigationKeysUpper
Aspect_VKey_Upper = _Aspect.Aspect_VKey_Upper
Aspect_VKey_NB = _Aspect.Aspect_VKey_NB
Aspect_VKey_MAX = _Aspect.Aspect_VKey_MAX
Aspect_TOD_RELATIVE = _Aspect.Aspect_TOD_RELATIVE
Aspect_TOD_ABSOLUTE = _Aspect.Aspect_TOD_ABSOLUTE
Aspect_TOL_EMPTY = _Aspect.Aspect_TOL_EMPTY
Aspect_TOL_SOLID = _Aspect.Aspect_TOL_SOLID
Aspect_TOL_DASH = _Aspect.Aspect_TOL_DASH
Aspect_TOL_DOT = _Aspect.Aspect_TOL_DOT
Aspect_TOL_DOTDASH = _Aspect.Aspect_TOL_DOTDASH
Aspect_TOL_USERDEFINED = _Aspect.Aspect_TOL_USERDEFINED
Aspect_GFM_NONE = _Aspect.Aspect_GFM_NONE
Aspect_GFM_HOR = _Aspect.Aspect_GFM_HOR
Aspect_GFM_VER = _Aspect.Aspect_GFM_VER
Aspect_GFM_DIAG1 = _Aspect.Aspect_GFM_DIAG1
Aspect_GFM_DIAG2 = _Aspect.Aspect_GFM_DIAG2
Aspect_GFM_CORNER1 = _Aspect.Aspect_GFM_CORNER1
Aspect_GFM_CORNER2 = _Aspect.Aspect_GFM_CORNER2
Aspect_GFM_CORNER3 = _Aspect.Aspect_GFM_CORNER3
Aspect_GFM_CORNER4 = _Aspect.Aspect_GFM_CORNER4
Aspect_TOHM_COLOR = _Aspect.Aspect_TOHM_COLOR
Aspect_TOHM_BOUNDBOX = _Aspect.Aspect_TOHM_BOUNDBOX
Aspect_TOR_UNKNOWN = _Aspect.Aspect_TOR_UNKNOWN
Aspect_TOR_NO_BORDER = _Aspect.Aspect_TOR_NO_BORDER
Aspect_TOR_TOP_BORDER = _Aspect.Aspect_TOR_TOP_BORDER
Aspect_TOR_RIGHT_BORDER = _Aspect.Aspect_TOR_RIGHT_BORDER
Aspect_TOR_BOTTOM_BORDER = _Aspect.Aspect_TOR_BOTTOM_BORDER
Aspect_TOR_LEFT_BORDER = _Aspect.Aspect_TOR_LEFT_BORDER
Aspect_TOR_TOP_AND_RIGHT_BORDER = _Aspect.Aspect_TOR_TOP_AND_RIGHT_BORDER
Aspect_TOR_RIGHT_AND_BOTTOM_BORDER = _Aspect.Aspect_TOR_RIGHT_AND_BOTTOM_BORDER
Aspect_TOR_BOTTOM_AND_LEFT_BORDER = _Aspect.Aspect_TOR_BOTTOM_AND_LEFT_BORDER
Aspect_TOR_LEFT_AND_TOP_BORDER = _Aspect.Aspect_TOR_LEFT_AND_TOP_BORDER
Aspect_GT_Rectangular = _Aspect.Aspect_GT_Rectangular
Aspect_GT_Circular = _Aspect.Aspect_GT_Circular
Aspect_TOCSD_AUTO = _Aspect.Aspect_TOCSD_AUTO
Aspect_TOCSD_USER = _Aspect.Aspect_TOCSD_USER
Aspect_TOST_NORMAL = _Aspect.Aspect_TOST_NORMAL
Aspect_TOST_ANNOTATION = _Aspect.Aspect_TOST_ANNOTATION
Aspect_TOM_EMPTY = _Aspect.Aspect_TOM_EMPTY
Aspect_TOM_POINT = _Aspect.Aspect_TOM_POINT
Aspect_TOM_PLUS = _Aspect.Aspect_TOM_PLUS
Aspect_TOM_STAR = _Aspect.Aspect_TOM_STAR
Aspect_TOM_X = _Aspect.Aspect_TOM_X
Aspect_TOM_O = _Aspect.Aspect_TOM_O
Aspect_TOM_O_POINT = _Aspect.Aspect_TOM_O_POINT
Aspect_TOM_O_PLUS = _Aspect.Aspect_TOM_O_PLUS
Aspect_TOM_O_STAR = _Aspect.Aspect_TOM_O_STAR
Aspect_TOM_O_X = _Aspect.Aspect_TOM_O_X
Aspect_TOM_RING1 = _Aspect.Aspect_TOM_RING1
Aspect_TOM_RING2 = _Aspect.Aspect_TOM_RING2
Aspect_TOM_RING3 = _Aspect.Aspect_TOM_RING3
Aspect_TOM_BALL = _Aspect.Aspect_TOM_BALL
Aspect_TOM_USERDEFINED = _Aspect.Aspect_TOM_USERDEFINED
Aspect_TOCSO_NONE = _Aspect.Aspect_TOCSO_NONE
Aspect_TOCSO_LEFT = _Aspect.Aspect_TOCSO_LEFT
Aspect_TOCSO_RIGHT = _Aspect.Aspect_TOCSO_RIGHT
Aspect_TOCSO_CENTER = _Aspect.Aspect_TOCSO_CENTER
Aspect_TOFM_BOTH_SIDE = _Aspect.Aspect_TOFM_BOTH_SIDE
Aspect_TOFM_BACK_SIDE = _Aspect.Aspect_TOFM_BACK_SIDE
Aspect_TOFM_FRONT_SIDE = _Aspect.Aspect_TOFM_FRONT_SIDE
Aspect_FM_NONE = _Aspect.Aspect_FM_NONE
Aspect_FM_CENTERED = _Aspect.Aspect_FM_CENTERED
Aspect_FM_TILED = _Aspect.Aspect_FM_TILED
Aspect_FM_STRETCH = _Aspect.Aspect_FM_STRETCH
Aspect_HS_SOLID = _Aspect.Aspect_HS_SOLID
Aspect_HS_HORIZONTAL = _Aspect.Aspect_HS_HORIZONTAL
Aspect_HS_HORIZONTAL_WIDE = _Aspect.Aspect_HS_HORIZONTAL_WIDE
Aspect_HS_VERTICAL = _Aspect.Aspect_HS_VERTICAL
Aspect_HS_VERTICAL_WIDE = _Aspect.Aspect_HS_VERTICAL_WIDE
Aspect_HS_DIAGONAL_45 = _Aspect.Aspect_HS_DIAGONAL_45
Aspect_HS_DIAGONAL_45_WIDE = _Aspect.Aspect_HS_DIAGONAL_45_WIDE
Aspect_HS_DIAGONAL_135 = _Aspect.Aspect_HS_DIAGONAL_135
Aspect_HS_DIAGONAL_135_WIDE = _Aspect.Aspect_HS_DIAGONAL_135_WIDE
Aspect_HS_GRID = _Aspect.Aspect_HS_GRID
Aspect_HS_GRID_WIDE = _Aspect.Aspect_HS_GRID_WIDE
Aspect_HS_GRID_DIAGONAL = _Aspect.Aspect_HS_GRID_DIAGONAL
Aspect_HS_GRID_DIAGONAL_WIDE = _Aspect.Aspect_HS_GRID_DIAGONAL_WIDE
Aspect_HS_NB = _Aspect.Aspect_HS_NB
Aspect_POM_Off = _Aspect.Aspect_POM_Off
Aspect_POM_Fill = _Aspect.Aspect_POM_Fill
Aspect_POM_Line = _Aspect.Aspect_POM_Line
Aspect_POM_Point = _Aspect.Aspect_POM_Point
Aspect_POM_All = _Aspect.Aspect_POM_All
Aspect_POM_None = _Aspect.Aspect_POM_None
Aspect_POM_Mask = _Aspect.Aspect_POM_Mask
Aspect_TOCSP_NONE = _Aspect.Aspect_TOCSP_NONE
Aspect_TOCSP_LEFT = _Aspect.Aspect_TOCSP_LEFT
Aspect_TOCSP_RIGHT = _Aspect.Aspect_TOCSP_RIGHT
Aspect_TOCSP_CENTER = _Aspect.Aspect_TOCSP_CENTER
Aspect_VKeyFlags_NONE = _Aspect.Aspect_VKeyFlags_NONE
Aspect_VKeyFlags_SHIFT = _Aspect.Aspect_VKeyFlags_SHIFT
Aspect_VKeyFlags_CTRL = _Aspect.Aspect_VKeyFlags_CTRL
Aspect_VKeyFlags_ALT = _Aspect.Aspect_VKeyFlags_ALT
Aspect_VKeyFlags_MENU = _Aspect.Aspect_VKeyFlags_MENU
Aspect_VKeyFlags_META = _Aspect.Aspect_VKeyFlags_META
Aspect_VKeyFlags_ALL = _Aspect.Aspect_VKeyFlags_ALL
Aspect_VKeyMouse_NONE = _Aspect.Aspect_VKeyMouse_NONE
Aspect_VKeyMouse_LeftButton = _Aspect.Aspect_VKeyMouse_LeftButton
Aspect_VKeyMouse_MiddleButton = _Aspect.Aspect_VKeyMouse_MiddleButton
Aspect_VKeyMouse_RightButton = _Aspect.Aspect_VKeyMouse_RightButton
Aspect_VKeyMouse_MainButtons = _Aspect.Aspect_VKeyMouse_MainButtons
Aspect_XA_DELETE_WINDOW = _Aspect.Aspect_XA_DELETE_WINDOW
Aspect_TOTP_CENTER = _Aspect.Aspect_TOTP_CENTER
Aspect_TOTP_TOP = _Aspect.Aspect_TOTP_TOP
Aspect_TOTP_BOTTOM = _Aspect.Aspect_TOTP_BOTTOM
Aspect_TOTP_LEFT = _Aspect.Aspect_TOTP_LEFT
Aspect_TOTP_RIGHT = _Aspect.Aspect_TOTP_RIGHT
Aspect_TOTP_LEFT_LOWER = _Aspect.Aspect_TOTP_LEFT_LOWER
Aspect_TOTP_LEFT_UPPER = _Aspect.Aspect_TOTP_LEFT_UPPER
Aspect_TOTP_RIGHT_LOWER = _Aspect.Aspect_TOTP_RIGHT_LOWER
Aspect_TOTP_RIGHT_UPPER = _Aspect.Aspect_TOTP_RIGHT_UPPER
Aspect_GDM_Lines = _Aspect.Aspect_GDM_Lines
Aspect_GDM_Points = _Aspect.Aspect_GDM_Points
Aspect_GDM_None = _Aspect.Aspect_GDM_None
Aspect_WOL_THIN = _Aspect.Aspect_WOL_THIN
Aspect_WOL_MEDIUM = _Aspect.Aspect_WOL_MEDIUM
Aspect_WOL_THICK = _Aspect.Aspect_WOL_THICK
Aspect_WOL_VERYTHICK = _Aspect.Aspect_WOL_VERYTHICK
Aspect_WOL_USERDEFINED = _Aspect.Aspect_WOL_USERDEFINED
Aspect_TODT_NORMAL = _Aspect.Aspect_TODT_NORMAL
Aspect_TODT_SUBTITLE = _Aspect.Aspect_TODT_SUBTITLE
Aspect_TODT_DEKALE = _Aspect.Aspect_TODT_DEKALE
Aspect_TODT_BLEND = _Aspect.Aspect_TODT_BLEND
Aspect_TODT_DIMENSION = _Aspect.Aspect_TODT_DIMENSION
Aspect_TODT_SHADOW = _Aspect.Aspect_TODT_SHADOW
Aspect_IS_EMPTY = _Aspect.Aspect_IS_EMPTY
Aspect_IS_SOLID = _Aspect.Aspect_IS_SOLID
Aspect_IS_HATCH = _Aspect.Aspect_IS_HATCH
Aspect_IS_HIDDENLINE = _Aspect.Aspect_IS_HIDDENLINE
Aspect_IS_POINT = _Aspect.Aspect_IS_POINT
Aspect_IS_HOLLOW = _Aspect.Aspect_IS_HOLLOW
Handle_Aspect_DisplayConnection_Create = _Aspect.Handle_Aspect_DisplayConnection_Create
Handle_Aspect_DisplayConnection_DownCast = _Aspect.Handle_Aspect_DisplayConnection_DownCast
Handle_Aspect_DisplayConnection_IsNull = _Aspect.Handle_Aspect_DisplayConnection_IsNull
Handle_Aspect_Grid_Create = _Aspect.Handle_Aspect_Grid_Create
Handle_Aspect_Grid_DownCast = _Aspect.Handle_Aspect_Grid_DownCast
Handle_Aspect_Grid_IsNull = _Aspect.Handle_Aspect_Grid_IsNull
Handle_Aspect_Window_Create = _Aspect.Handle_Aspect_Window_Create
Handle_Aspect_Window_DownCast = _Aspect.Handle_Aspect_Window_DownCast
Handle_Aspect_Window_IsNull = _Aspect.Handle_Aspect_Window_IsNull
class Aspect_SequenceOfColor(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    begin = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_begin)
    end = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_end)
    cbegin = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_cbegin)
    cend = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_cend)

    def __init__(self, *args):
        _Aspect.Aspect_SequenceOfColor_swiginit(self, _Aspect.new_Aspect_SequenceOfColor(*args))
    Size = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_Size)
    Length = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_Length)
    Lower = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_Lower)
    Upper = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_Upper)
    IsEmpty = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_IsEmpty)
    Reverse = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_Reverse)
    Exchange = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_Exchange)
    delNode = _swig_new_static_method(_Aspect.Aspect_SequenceOfColor_delNode)
    Clear = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_Clear)
    Assign = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_Assign)
    Set = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_Set)
    Remove = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_Remove)
    Append = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_Append)
    Prepend = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_Prepend)
    InsertBefore = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_InsertBefore)
    InsertAfter = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_InsertAfter)
    Split = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_Split)
    First = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_First)
    ChangeFirst = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_ChangeFirst)
    Last = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_Last)
    ChangeLast = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_ChangeLast)
    Value = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_Value)
    ChangeValue = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_ChangeValue)
    __call__ = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor___call__)
    SetValue = _swig_new_instance_method(_Aspect.Aspect_SequenceOfColor_SetValue)
    __swig_destroy__ = _Aspect.delete_Aspect_SequenceOfColor

# Register Aspect_SequenceOfColor in _Aspect:
_Aspect.Aspect_SequenceOfColor_swigregister(Aspect_SequenceOfColor)
Aspect_SequenceOfColor_delNode = _Aspect.Aspect_SequenceOfColor_delNode

class Aspect_TouchMap(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    begin = _swig_new_instance_method(_Aspect.Aspect_TouchMap_begin)
    end = _swig_new_instance_method(_Aspect.Aspect_TouchMap_end)
    cbegin = _swig_new_instance_method(_Aspect.Aspect_TouchMap_cbegin)
    cend = _swig_new_instance_method(_Aspect.Aspect_TouchMap_cend)

    def __init__(self, *args):
        _Aspect.Aspect_TouchMap_swiginit(self, _Aspect.new_Aspect_TouchMap(*args))
    Exchange = _swig_new_instance_method(_Aspect.Aspect_TouchMap_Exchange)
    Assign = _swig_new_instance_method(_Aspect.Aspect_TouchMap_Assign)
    Set = _swig_new_instance_method(_Aspect.Aspect_TouchMap_Set)
    ReSize = _swig_new_instance_method(_Aspect.Aspect_TouchMap_ReSize)
    Add = _swig_new_instance_method(_Aspect.Aspect_TouchMap_Add)
    Contains = _swig_new_instance_method(_Aspect.Aspect_TouchMap_Contains)
    Substitute = _swig_new_instance_method(_Aspect.Aspect_TouchMap_Substitute)
    Swap = _swig_new_instance_method(_Aspect.Aspect_TouchMap_Swap)
    RemoveLast = _swig_new_instance_method(_Aspect.Aspect_TouchMap_RemoveLast)
    RemoveFromIndex = _swig_new_instance_method(_Aspect.Aspect_TouchMap_RemoveFromIndex)
    RemoveKey = _swig_new_instance_method(_Aspect.Aspect_TouchMap_RemoveKey)
    FindKey = _swig_new_instance_method(_Aspect.Aspect_TouchMap_FindKey)
    FindFromIndex = _swig_new_instance_method(_Aspect.Aspect_TouchMap_FindFromIndex)
    ChangeFromIndex = _swig_new_instance_method(_Aspect.Aspect_TouchMap_ChangeFromIndex)
    __call__ = _swig_new_instance_method(_Aspect.Aspect_TouchMap___call__)
    FindIndex = _swig_new_instance_method(_Aspect.Aspect_TouchMap_FindIndex)
    ChangeFromKey = _swig_new_instance_method(_Aspect.Aspect_TouchMap_ChangeFromKey)
    Seek = _swig_new_instance_method(_Aspect.Aspect_TouchMap_Seek)
    ChangeSeek = _swig_new_instance_method(_Aspect.Aspect_TouchMap_ChangeSeek)
    FindFromKey = _swig_new_instance_method(_Aspect.Aspect_TouchMap_FindFromKey)
    Clear = _swig_new_instance_method(_Aspect.Aspect_TouchMap_Clear)
    __swig_destroy__ = _Aspect.delete_Aspect_TouchMap
    Size = _swig_new_instance_method(_Aspect.Aspect_TouchMap_Size)

# Register Aspect_TouchMap in _Aspect:
_Aspect.Aspect_TouchMap_swigregister(Aspect_TouchMap)

class Aspect_Background(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr

    def __init__(self, *args):
        r"""
        * Creates a window background. Default color : NOC_MATRAGRAY.
        	:rtype: None* Creates a window background with the colour <AColor>.
        	:param AColor:
        	:type AColor: Quantity_Color
        	:rtype: None
        """
        _Aspect.Aspect_Background_swiginit(self, _Aspect.new_Aspect_Background(*args))
    Color = _swig_new_instance_method(_Aspect.Aspect_Background_Color)
    SetColor = _swig_new_instance_method(_Aspect.Aspect_Background_SetColor)

    __repr__ = _dumps_object

    __swig_destroy__ = _Aspect.delete_Aspect_Background

# Register Aspect_Background in _Aspect:
_Aspect.Aspect_Background_swigregister(Aspect_Background)

class Aspect_DisplayConnection(OCC.Core.Standard.Standard_Transient):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined")
    __repr__ = _swig_repr


    @staticmethod
    def DownCast(t):
      return Handle_Aspect_DisplayConnection_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _Aspect.delete_Aspect_DisplayConnection

# Register Aspect_DisplayConnection in _Aspect:
_Aspect.Aspect_DisplayConnection_swigregister(Aspect_DisplayConnection)

class Aspect_GenId(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr

    def __init__(self, *args):
        r"""
        * Creates an available set of identifiers with the lower bound 0 and the upper bound INT_MAX / 2.
        	:rtype: None* Creates an available set of identifiers with specified range. Raises IdentDefinitionError if theUpper is less than theLow.
        	:param theLow:
        	:type theLow: int
        	:param theUpper:
        	:type theUpper: int
        	:rtype: None
        """
        _Aspect.Aspect_GenId_swiginit(self, _Aspect.new_Aspect_GenId(*args))
    Available = _swig_new_instance_method(_Aspect.Aspect_GenId_Available)
    Free = _swig_new_instance_method(_Aspect.Aspect_GenId_Free)
    HasFree = _swig_new_instance_method(_Aspect.Aspect_GenId_HasFree)
    Lower = _swig_new_instance_method(_Aspect.Aspect_GenId_Lower)
    Next = _swig_new_instance_method(_Aspect.Aspect_GenId_Next)
    Upper = _swig_new_instance_method(_Aspect.Aspect_GenId_Upper)

    __repr__ = _dumps_object

    __swig_destroy__ = _Aspect.delete_Aspect_GenId

# Register Aspect_GenId in _Aspect:
_Aspect.Aspect_GenId_swigregister(Aspect_GenId)

class Aspect_Grid(OCC.Core.Standard.Standard_Transient):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined")
    __repr__ = _swig_repr
    Activate = _swig_new_instance_method(_Aspect.Aspect_Grid_Activate)
    Colors = _swig_new_instance_method(_Aspect.Aspect_Grid_Colors)
    Compute = _swig_new_instance_method(_Aspect.Aspect_Grid_Compute)
    Deactivate = _swig_new_instance_method(_Aspect.Aspect_Grid_Deactivate)
    Display = _swig_new_instance_method(_Aspect.Aspect_Grid_Display)
    DrawMode = _swig_new_instance_method(_Aspect.Aspect_Grid_DrawMode)
    Erase = _swig_new_instance_method(_Aspect.Aspect_Grid_Erase)
    Hit = _swig_new_instance_method(_Aspect.Aspect_Grid_Hit)
    Init = _swig_new_instance_method(_Aspect.Aspect_Grid_Init)
    IsActive = _swig_new_instance_method(_Aspect.Aspect_Grid_IsActive)
    IsDisplayed = _swig_new_instance_method(_Aspect.Aspect_Grid_IsDisplayed)
    Rotate = _swig_new_instance_method(_Aspect.Aspect_Grid_Rotate)
    RotationAngle = _swig_new_instance_method(_Aspect.Aspect_Grid_RotationAngle)
    SetColors = _swig_new_instance_method(_Aspect.Aspect_Grid_SetColors)
    SetDrawMode = _swig_new_instance_method(_Aspect.Aspect_Grid_SetDrawMode)
    SetRotationAngle = _swig_new_instance_method(_Aspect.Aspect_Grid_SetRotationAngle)
    SetXOrigin = _swig_new_instance_method(_Aspect.Aspect_Grid_SetXOrigin)
    SetYOrigin = _swig_new_instance_method(_Aspect.Aspect_Grid_SetYOrigin)
    Translate = _swig_new_instance_method(_Aspect.Aspect_Grid_Translate)
    XOrigin = _swig_new_instance_method(_Aspect.Aspect_Grid_XOrigin)
    YOrigin = _swig_new_instance_method(_Aspect.Aspect_Grid_YOrigin)


    @staticmethod
    def DownCast(t):
      return Handle_Aspect_Grid_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _Aspect.delete_Aspect_Grid

# Register Aspect_Grid in _Aspect:
_Aspect.Aspect_Grid_swigregister(Aspect_Grid)

class Aspect_ScrollDelta(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    Delta = property(_Aspect.Aspect_ScrollDelta_Delta_get, _Aspect.Aspect_ScrollDelta_Delta_set)
    Flags = property(_Aspect.Aspect_ScrollDelta_Flags_get, _Aspect.Aspect_ScrollDelta_Flags_set)

    def __init__(self, *args):
        r"""
        * Empty constructor.
        	:rtype: None* Constructor.
        	:param thePnt:
        	:type thePnt: NCollection_Vec2<int>
        	:param theValue:
        	:type theValue: float
        	:param theFlags: default value is Aspect_VKeyFlags_NONE
        	:type theFlags: Aspect_VKeyFlags
        	:rtype: None* Constructor with undefined point.
        	:param theValue:
        	:type theValue: float
        	:param theFlags: default value is Aspect_VKeyFlags_NONE
        	:type theFlags: Aspect_VKeyFlags
        	:rtype: None
        """
        _Aspect.Aspect_ScrollDelta_swiginit(self, _Aspect.new_Aspect_ScrollDelta(*args))
    HasPoint = _swig_new_instance_method(_Aspect.Aspect_ScrollDelta_HasPoint)
    ResetPoint = _swig_new_instance_method(_Aspect.Aspect_ScrollDelta_ResetPoint)

    __repr__ = _dumps_object

    __swig_destroy__ = _Aspect.delete_Aspect_ScrollDelta

# Register Aspect_ScrollDelta in _Aspect:
_Aspect.Aspect_ScrollDelta_swigregister(Aspect_ScrollDelta)

class Aspect_Touch(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    IsPreciseDevice = property(_Aspect.Aspect_Touch_IsPreciseDevice_get, _Aspect.Aspect_Touch_IsPreciseDevice_set)

    def __init__(self, *args):
        r"""
        * Empty constructor
        	:rtype: None* Constructor with initialization.
        	:param thePnt:
        	:type thePnt: NCollection_Vec2<float>
        	:param theIsPreciseDevice:
        	:type theIsPreciseDevice: bool
        	:rtype: None* Constructor with initialization.
        	:param theX:
        	:type theX: float
        	:param theY:
        	:type theY: float
        	:param theIsPreciseDevice:
        	:type theIsPreciseDevice: bool
        	:rtype: None
        """
        _Aspect.Aspect_Touch_swiginit(self, _Aspect.new_Aspect_Touch(*args))
    Delta = _swig_new_instance_method(_Aspect.Aspect_Touch_Delta)

    __repr__ = _dumps_object

    __swig_destroy__ = _Aspect.delete_Aspect_Touch

# Register Aspect_Touch in _Aspect:
_Aspect.Aspect_Touch_swigregister(Aspect_Touch)

class Aspect_Window(OCC.Core.Standard.Standard_Transient):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined")
    __repr__ = _swig_repr
    Background = _swig_new_instance_method(_Aspect.Aspect_Window_Background)
    BackgroundFillMethod = _swig_new_instance_method(_Aspect.Aspect_Window_BackgroundFillMethod)
    DoMapping = _swig_new_instance_method(_Aspect.Aspect_Window_DoMapping)
    DoResize = _swig_new_instance_method(_Aspect.Aspect_Window_DoResize)
    GradientBackground = _swig_new_instance_method(_Aspect.Aspect_Window_GradientBackground)
    InvalidateContent = _swig_new_instance_method(_Aspect.Aspect_Window_InvalidateContent)
    IsMapped = _swig_new_instance_method(_Aspect.Aspect_Window_IsMapped)
    IsVirtual = _swig_new_instance_method(_Aspect.Aspect_Window_IsVirtual)
    Map = _swig_new_instance_method(_Aspect.Aspect_Window_Map)
    NativeFBConfig = _swig_new_instance_method(_Aspect.Aspect_Window_NativeFBConfig)
    Position = _swig_new_instance_method(_Aspect.Aspect_Window_Position)
    Ratio = _swig_new_instance_method(_Aspect.Aspect_Window_Ratio)
    SetBackground = _swig_new_instance_method(_Aspect.Aspect_Window_SetBackground)
    SetTitle = _swig_new_instance_method(_Aspect.Aspect_Window_SetTitle)
    SetVirtual = _swig_new_instance_method(_Aspect.Aspect_Window_SetVirtual)
    Size = _swig_new_instance_method(_Aspect.Aspect_Window_Size)
    Unmap = _swig_new_instance_method(_Aspect.Aspect_Window_Unmap)


    @staticmethod
    def DownCast(t):
      return Handle_Aspect_Window_DownCast(t)


    __repr__ = _dumps_object

    __swig_destroy__ = _Aspect.delete_Aspect_Window

# Register Aspect_Window in _Aspect:
_Aspect.Aspect_Window_swigregister(Aspect_Window)

class Aspect_GradientBackground(Aspect_Background):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr

    def __init__(self, *args):
        r"""
        * Creates a window gradient background. Default colors : Quantity_NOC_BLACK. Default fill method : Aspect_GFM_NONE
        	:rtype: None* Creates a window gradient background with colours <AColor1, AColor2>.
        	:param AColor1:
        	:type AColor1: Quantity_Color
        	:param AColor2:
        	:type AColor2: Quantity_Color
        	:param AMethod: default value is Aspect_GFM_HOR
        	:type AMethod: Aspect_GradientFillMethod
        	:rtype: None
        """
        _Aspect.Aspect_GradientBackground_swiginit(self, _Aspect.new_Aspect_GradientBackground(*args))
    BgGradientFillMethod = _swig_new_instance_method(_Aspect.Aspect_GradientBackground_BgGradientFillMethod)
    Colors = _swig_new_instance_method(_Aspect.Aspect_GradientBackground_Colors)
    SetColors = _swig_new_instance_method(_Aspect.Aspect_GradientBackground_SetColors)

    __repr__ = _dumps_object

    __swig_destroy__ = _Aspect.delete_Aspect_GradientBackground

# Register Aspect_GradientBackground in _Aspect:
_Aspect.Aspect_GradientBackground_swigregister(Aspect_GradientBackground)



