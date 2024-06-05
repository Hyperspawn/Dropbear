# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#
# Copyright 2021 Tommi Hyppänen

bl_info = {
    "name": "STEP OCC import",
    "author": "ambi",
    "description": "STEP OpenCASCADE import",
    "blender": (2, 93, 0),
    "version": (1, 1, 1),
    "location": "3D View > Tools panel > Stepper",
    "category": "Import",
}


INSIDE_BLENDER = True
try:
    import bpy
except ModuleNotFoundError:
    print("Stepper not running inside Blender.")
    INSIDE_BLENDER = False


if INSIDE_BLENDER:
    # Normally don't do import star, but here it's basically a file concatenation
    # File concatenation is because the test framework breaks on __init__.py import bpy
    from .main import *
