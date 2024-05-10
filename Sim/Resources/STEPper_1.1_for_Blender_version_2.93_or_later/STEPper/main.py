# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# Created Date: Thursday, April 15th 2021, 4:38:48 pm
# Copyright: Tommi Hyppänen

import bpy
import bmesh
from bpy_extras.io_utils import ImportHelper
from bpy.props import StringProperty

import os

import numpy as np
import ntpath
import time
import dataclasses

# import sys
# import math

# from collections import defaultdict

# from .sizeof import total_size
# utils.memorytrace_start()

global_file_cache = {}


def tri_ori(p1, p2, p3):
    """Triangle orientation without colinearity test"""
    return (p2[1] - p1[1]) * (p3[0] - p2[0]) - (p2[0] - p1[0]) * (p3[1] - p2[1]) > 0


def scalemat(mat, sl):
    scaling = np.zeros_like(mat)
    scaling[np.diag_indices(4)] = sl
    # print(scaling)
    return np.matmul(scaling, mat)


def obj_unlink_all(obj):
    """Unlink object from all collections"""
    old_col = obj.users_collection

    # bugfix: not in master collection bug
    # collection_name.objects.unlink(obj)
    if len(old_col) > 0:
        for c in old_col:
            c.objects.unlink(obj)


def rotate_selection_on_axis(angle, axis):
    """Angle in radians, axis is X, Y or Z"""
    assert axis in {"X", "Y", "Z"}
    constraint = (axis == "X", axis == "Y", axis == "Z")
    bpy.ops.transform.rotate(
        value=angle,
        orient_axis=axis,
        orient_type="GLOBAL",
        orient_matrix=((1, 0, 0), (0, 1, 0), (0, 0, 1)),
        orient_matrix_type="GLOBAL",
        constraint_axis=constraint,
        mirror=True,
        use_proportional_edit=False,
        proportional_edit_falloff="SMOOTH",
        proportional_size=1,
        use_proportional_connected=False,
        use_proportional_projected=False,
    )


def scale_selection_on_axis(amount, axis):
    """Axis is X, Y or Z"""
    axii = {"X": 0, "Y": 1, "Z": 2}
    assert axis in axii
    constraint = [False, False, False]
    constraint[axii[axis]] = True
    scaling = [1, 1, 1]
    scaling[axii[axis]] = amount
    print(scaling, constraint)
    bpy.ops.transform.resize(
        value=scaling,
        orient_type="GLOBAL",
        orient_matrix=((1, 0, 0), (0, 1, 0), (0, 0, 1)),
        orient_matrix_type="GLOBAL",
        constraint_axis=constraint,
        mirror=True,
        use_proportional_edit=False,
        proportional_edit_falloff="SMOOTH",
        proportional_size=1,
        use_proportional_connected=False,
        use_proportional_projected=False,
    )


def add_material(name, color, link_vertex_color=False, overwrite=False):
    assert len(color) == 3
    assert isinstance(color, tuple)
    if len(name) > 60:
        name = name[:60]
    if name not in bpy.data.materials.keys() or overwrite:
        mat = bpy.data.materials.new(name)
        mat.use_nodes = True

        bsdf = mat.node_tree.nodes["Principled BSDF"]

        # Set base color
        bsdf.inputs["Base Color"].default_value = (*color, 1.0)

        # # Connect alpha
        # a = mat.node_tree.nodes["Principled BSDF"].inputs["Alpha"]
        # mat.node_tree.links.new(sn.outputs["Alpha"], a)

        vcol = mat.node_tree.nodes.new(type="ShaderNodeVertexColor")
        vcol.location = [-400.0, 300.0]
        vcol.layer_name = "Colors"

        if link_vertex_color:
            mat.node_tree.links.new(vcol.outputs[0], bsdf.inputs[0])
    else:
        mat = bpy.data.materials[name]

    # mat.blend_method = "BLEND"
    # mat.shadow_method = "CLIP"
    # mat.node_tree.nodes["Image Texture"].image = image
    return mat


def bpy_update_object_data(
    objdata, bm, vcol_name, colors, uvs, norms, mat_names, build_materials=True
):
    if build_materials:
        # set colors and mats
        obj_mats = {}
        for obi, ob_mat in enumerate(objdata.materials):
            obj_mats[ob_mat.name] = obi
        mat_counter = 0

    if len(colors) > 0:
        color_layer = bm.loops.layers.color.get(vcol_name)
        if color_layer is None:
            color_layer = bm.loops.layers.color.new(vcol_name)
        # uv_layer = bm.loops.layers.uv.verify()
        i = 0
        for face in bm.faces:
            mat_col = (0.5, 0.5, 0.5)
            mat_col_name = None
            for loop in face.loops:
                # TODO: good, proper aspect ratio UV
                # loop[uv_layer].uv = uvs[i]
                if colors[i][0] >= 0.0:
                    loop[color_layer] = (*colors[i], 1.0)
                    mat_col = colors[i]
                    mat_col_name = mat_names[i]
                else:
                    # No color: set it to default gray
                    loop[color_layer] = (0.5, 0.5, 0.5, 1.0)
                i += 1

            if build_materials:
                # Translate color into name, if not defined
                if mat_col_name is None:
                    mat_col_name = "STEP_" + "".join(
                        "{0:0{1}x}".format(int(mat_col[i] * 255), 2) for i in range(3)
                    )

                # If material doesn't exist, create it
                if mat_col_name not in bpy.data.materials:
                    add_material(mat_col_name, mat_col, link_vertex_color=True)

                # If material exists but it's not yet in object material slot, add it
                if mat_col_name not in obj_mats:
                    obj_mats[mat_col_name] = mat_counter
                    objdata.materials.append(bpy.data.materials[mat_col_name])
                    mat_counter += 1

                face.material_index = obj_mats[mat_col_name]
    else:
        # TODO: if no colors defined, create and apply default material
        pass

    # print("Polys: {}, Verts: {}".format(len(bm.faces), len(bm.verts)))

    # Save face situation so we can adjust accordingly later
    # pre_faces = bm.faces[:]

    # # Merge verts near each other
    # if merge_distance > 0.0:
    #     print("Removing doubles at distance:", merge_distance)
    #     bmesh.ops.remove_doubles(bm, verts=bm.verts[:], dist=merge_distance)

    # Remove normals from array which don't exist in the mesh anymore
    # removed = set()
    # for fi, f in enumerate(pre_faces):
    #     if not f.is_valid:
    #         for i in range(fi * 3, fi * 3 + 3):
    #             removed.add(i)

    # Update mesh from Bmesh
    # Apply also in edit mode, not just object mode
    prev_mode = bpy.context.object.mode
    bpy.ops.object.mode_set(mode="OBJECT")

    bm.to_mesh(objdata)

    objdata.use_auto_smooth = True
    objdata.auto_smooth_angle = 3.14159

    # Filter removed items from norms
    # norms = [n for ni, n in enumerate(norms) if ni not in removed]
    objdata.normals_split_custom_set(np.array(norms))

    # Return to previous object/edit mode
    bpy.ops.object.mode_set(mode=prev_mode)


def calculate_detail_level(dlev):
    """Angular defflection, Linear deflection"""
    if dlev < 100:
        l_def = 100.0 / float(dlev)
    else:
        l_def = (100.0 / float(dlev)) ** 2.0
    return 0.8, l_def


def set_obj_matrix_world(obj, mtx):
    """
    Copy Numpy matrix into Blender matrix
    """
    for row in range(mtx.shape[0]):
        for col in range(mtx.shape[1]):
            obj.matrix_world[row][col] = mtx[row][col]


def create_new_obj_with_mesh(name, set_active=True):
    """
    Create new empty object and mesh, link them, and optionally set to active
    """
    empty_mesh = bpy.data.meshes.new(name)
    obj = bpy.data.objects.new(name, empty_mesh)
    bpy.context.collection.objects.link(obj)
    if set_active:
        bpy.context.view_layer.objects.active = obj
    return obj


def choose_hierarchy_types(htypes):
    """
    Return hierarchy types selection from input string
    """
    hierarchy_flat = False
    hierarchy_tree = False
    hierarchy_empties = False

    if htypes == "FLAT_AND_TREE":
        hierarchy_flat = True
        hierarchy_tree = True
    elif htypes == "TREE":
        hierarchy_tree = True
    elif htypes == "FLAT":
        hierarchy_flat = True
    elif htypes == "EMPTIES":
        hierarchy_empties = True
    else:
        assert False, "Invalid input parameter"

    return hierarchy_flat, hierarchy_tree, hierarchy_empties


def transform_to_up(up, chosen_objects, scale, to_cursor=True):
    """
    Set all chosen_objects transforms <up>["X", "Y", "Z"] as up
    Optionally move to cursor <to_cursor>
    Set scale to scale
    """

    # transforms and processing of objects
    # bpy.ops.object.select_all(action="DESELECT")

    cursor_pos = bpy.context.scene.cursor.location

    # up
    # up_as = self.up_as
    up_axis = {"X": 0, "Y": 1, "Z": 2}[up]

    # forward
    # fw_as = self.prg.fw_as
    # fw_axis = {"X": 0, "Y": 1, "Z": 2}[fw_as[0]]

    for obj in chosen_objects:
        # up, forward
        mat = np.array(obj.matrix_world)

        # blender default: Y(1) = forward, Z(2) = up
        if up_axis != 2:
            # if negate axis, do mirror
            # if up_as[1] == "N":
            #     dg = [1, 1, 1, 1]
            #     dg[up_axis] = -1
            #     mat = _scalemat(mat, dg)

            mat[[up_axis, 2]] = mat[[2, up_axis]]
            mat[up_axis] *= -1

        # scale
        mat = scalemat(mat, [*([scale] * 3), 1])

        # move to cursor position
        mat[0][3] += cursor_pos.x
        mat[1][3] += cursor_pos.y
        mat[2][3] += cursor_pos.z

        # apply
        set_obj_matrix_world(obj, mat)

    # Apply scale
    # for obj in created_objs:
    #     # Apply object scale
    #     obj.select_set(True)
    #     bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    #     obj.select_set(False)

    # for obj in created_objs:
    #     obj.select_set(True)


def build_mesh(step_reader, obj, shp, lind, angd, vcol_name="Colors"):
    hacks = set([])
    if bpy.context.scene.stepper.hack_skip_zero_solids:
        hacks.add("skip_solids")
    mesh = step_reader.build_trimesh(shp, lin_def=lind, ang_def=angd, hacks=hacks)

    mesh.fuse_verts()
    mesh.filter_zero_area()
    mesh.filter_same_face()

    print("[bm]", end="")
    bm = bmesh.new()
    mesh.add_to_bm(bm, edges_as_seams=True, discontinuity_as_sharp=True)
    mesh.fill_empty_color()
    bpy_update_object_data(
        obj.data,
        bm,
        vcol_name,
        mesh.get_loop_colors(),
        mesh.get_loop_uvs(),
        mesh.get_loop_normals(),
        mesh.get_loop_material_names(),
    )

    return mesh.matrix


def load_step(
    context,
    filepath,
    custom_scale=None,
    lin_deflection=0.8,
    ang_deflection=0.5,
    # merge_distance=0.001,
    up_as="Y",
    htypes="TREE",
):
    from . import importer

    hierarchy_flat, hierarchy_tree, hierarchy_empties = choose_hierarchy_types(htypes)

    filename = "".join(ntpath.basename(filepath).split(".")[:-1])

    if filepath not in global_file_cache:
        step_reader = importer.ReadSTEP(filepath)
        global_file_cache[filepath] = step_reader
    else:
        step_reader = global_file_cache[filepath]
        print("Loaded file from cache")

    tree = step_reader.tree
    scale = step_reader.scale
    if custom_scale is not None:
        scale = custom_scale

    # divide by Blender unit length
    scale /= context.scene.unit_settings.scale_length
    print("Current Blender scale set at:", context.scene.unit_settings.scale_length)

    wm = bpy.context.window_manager

    created_objs = []
    created_names = {}
    created_uuid = {}

    # traverse shapes, render in "face" mode
    start_time = time.time()
    all_shapes = tree.get_shapes()
    total = len(all_shapes)

    wm.progress_begin(0, total)
    for i, (shp, node_index) in enumerate(all_shapes):
        parent_uuid, self_uuid, tag, name, _, local_t, global_t = tree.nodes[
            node_index
        ].get_values()

        if name == "root":
            name = filename + ".empties"

        shape_name = "tt_" + repr(tag)
        wm.progress_update(i)
        obj = None

        # Shape found in leaf
        if shp:
            print("\nBuilding ({}/{}): {} ".format(i + 1, total, name), end="", flush=True)
            print("[T" + repr(shp.ShapeType()) + "]", end="", flush=True)

            # If object already build, just copy it, using linked mesh data
            if shape_name in created_names:
                print("[Link]", end="", flush=True)

                source_obj = created_names[shape_name]
                obj = source_obj.copy()
                created_objs.append(obj)
            else:
                print("[Build]", end="", flush=True)

                # Create new mesh and object from scratch
                obj = create_new_obj_with_mesh(name)

                created_objs.append(obj)
                created_names[shape_name] = obj

                bpy.ops.object.mode_set(mode="OBJECT")

                build_mesh(step_reader, obj, shp, lin_deflection, ang_deflection)

        # No shape in leaf, empty creation enabled, do this
        elif hierarchy_empties:
            # Create empty
            obj = bpy.data.objects.new(name, None)
            obj.empty_display_size = 2
            obj.empty_display_type = "PLAIN_AXES"
            created_objs.append(obj)
            # set_obj_matrix_world(obj, global_t)

        # Object has been created
        if obj:
            # assign property to obj
            obj["STEP_tag"] = tag
            obj["STEP_parent"] = parent_uuid
            obj["STEP_uuid"] = self_uuid
            obj["STEP_file"] = filepath
            obj["STEP_name"] = name
            obj["STEP_tree_location"] = node_index
            created_uuid[self_uuid] = obj

    # assert len(created_objs) == len(shapes_labels)
    print("\n" + repr(step_reader.import_problems))

    # remove all temporary links
    for tobj in created_objs:
        obj_unlink_all(tobj)

    # build flat collection
    if hierarchy_flat:
        flat_collection = bpy.data.collections.new(filename + ".flat")
        bpy.context.scene.collection.children.link(flat_collection)

        created_collections = {}
        for obj in created_objs:
            group_name = obj["STEP_name"]

            # max collection name len = 61
            if len(group_name) > 50:
                group_name = group_name[:25] + "_" + group_name[-25:]

            # TODO: check dupe collections for dupe imports
            if group_name not in created_collections:
                group_collection = bpy.data.collections.new(group_name)
                created_collections[group_name] = group_collection
                flat_collection.children.link(group_collection)
            else:
                group_collection = created_collections[group_name]

            global_t = tree.nodes[obj["STEP_tree_location"]].global_transform
            set_obj_matrix_world(obj, global_t)
            group_collection.objects.link(obj)

    # build tree of collections
    if hierarchy_tree:
        tree_collection = bpy.data.collections.new(filename + ".hierarchy")
        bpy.context.scene.collection.children.link(tree_collection)
        hierarchy_collections = {}
        hierarchy_collections[-1] = tree_collection

        def node_parse(node, level, parent_collection):
            # if "name" in node and node["children"] is not None:
            if len(node.children) > 0:
                collection_node = bpy.data.collections.new(node.name)
                assert node.index not in hierarchy_collections
                hierarchy_collections[node.index] = collection_node

                parent_collection.children.link(collection_node)
                for c in node.children:
                    node_parse(tree.nodes[c], level + 1, collection_node)

        root = tree.nodes[0]
        if len(root.children) > 0:
            for c in root.children:
                node_parse(tree.nodes[c], 0, tree_collection)

            # link objects to tree
            if len(hierarchy_collections.items()) > 0:
                for obj in created_objs:
                    hierarchy_collections[obj["STEP_parent"]].objects.link(obj)
                    global_t = tree.nodes[obj["STEP_tree_location"]].global_transform
                    set_obj_matrix_world(obj, global_t)

    # build hierarchy with empties
    if hierarchy_empties:
        for obj in created_objs:
            global_t = tree.nodes[obj["STEP_tree_location"]].global_transform
            set_obj_matrix_world(obj, global_t)
            bpy.context.scene.collection.objects.link(obj)

            # Parent objs
            parent_id = obj["STEP_parent"]
            if parent_id in created_uuid:
                parent = created_uuid[parent_id]
                obj.parent = parent
                obj.matrix_parent_inverse = parent.matrix_world.inverted()

    transform_to_up(up_as[0], created_objs, scale)

    wm.progress_end()
    print(f"STEP loading time elapsed: {time.time()-start_time:.2f}")


class PG_Stepper(bpy.types.PropertyGroup):

    hack_skip_zero_solids: bpy.props.BoolProperty(
        name="Skip faulty solids",
        description="Skip some shapes the library hangs on and fails to load",
        default=False,
    )

    simpler_parameters: bpy.props.BoolProperty(
        name="Artist friendly parameters",
        description="Instead of linear and angle deflection values, use only detail setting",
        default=True,
    )

    detail_level: bpy.props.IntProperty(
        name="Mesh detail",
        description="How detailed you want the mesh to be",
        default=100,
        min=1,
    )

    lin_deflection: bpy.props.FloatProperty(
        name="Linear deflection",
        description="Smaller values increase polygon count. Higher values lower polygon count.",
        default=0.8,
        min=0.002,
        # max=2.0,
    )

    ang_deflection: bpy.props.FloatProperty(
        name="Angular deflection",
        description="Smaller values increase polygon count. Higher values lower polygon count.",
        default=0.5,
        min=0.002,
        # max=2.0,
    )

    fix_ascii_file: bpy.props.StringProperty(
        name="File",
        description="Path to problematic STEP file",
        default="",
        maxlen=1024,
        subtype="FILE_PATH",
    )


class ImportStepCADOperator(bpy.types.Operator, ImportHelper):
    bl_idname = "import_scene.occ_import_step"
    bl_label = "Import STEP"
    bl_description = "Import a STEP file"
    bl_options = {"PRESET"}

    filter_glob: StringProperty(default="*.step;*.stp;*.st", options={"HIDDEN"})
    files: bpy.props.CollectionProperty(type=bpy.types.PropertyGroup)
    # files: bpy.props.CollectionProperty(type=idprop.types.IDPropertyGroup)
    override_file: StringProperty(default="", options={"HIDDEN"})

    fw_as: bpy.props.EnumProperty(
        items=[
            ("XPOS", "X", "", 0),
            # ("XNEG", "X-", "", 1),
            ("YPOS", "Y", "", 2),
            # ("YNEG", "Y-", "", 3),
            ("ZPOS", "Z", "", 4),
            # ("ZNEG", "Z-", "", 5),
        ],
        name="Forward",
        default="ZPOS",
        description="Forward axis of the imported model",
    )

    up_as: bpy.props.EnumProperty(
        items=[
            ("XPOS", "X", "", 0),
            # ("XNEG", "X-", "", 1),
            ("YPOS", "Y", "", 2),
            # ("YNEG", "Y-", "", 3),
            ("ZPOS", "Z", "", 4),
            # ("ZNEG", "Z-", "", 5),
        ],
        name="Up",
        default="YPOS",
        description="Up axis of the imported model",
    )

    hierarchy_types: bpy.props.EnumProperty(
        items=[
            ("FLAT", "Flat collection", "", 2),
            ("TREE", "Tree collection", "", 4),
            ("EMPTIES", "Parented empties", "", 6),
            # ("FLAT_AND_TREE", "Flat and tree collection", "", 0),
        ],
        name="Tree hierarchy",
        default="EMPTIES",
        description="Organization styles of objects",
    )

    user_scale: bpy.props.FloatProperty(
        name="Scale", description="Set object scale", default=0.01, min=0.00001
    )

    lin_deflection: bpy.props.FloatProperty(
        name="Linear deflection",
        description="Smaller values increase polygon count. Higher values lower polygon count.",
        default=0.8,
        min=0.002,
        max=2.0,
    )

    ang_deflection: bpy.props.FloatProperty(
        name="Angular deflection",
        description="Smaller values increase polygon count. Higher values lower polygon count.",
        default=0.5,
        min=0.002,
        max=2.0,
    )

    detail_level: bpy.props.IntProperty(
        name="Mesh detail",
        description="How detailed you want the mesh to be",
        default=100,
        min=1,
    )

    custom_scale: bpy.props.BoolProperty(
        name="Custom scale",
        description="Instead of loading the unit information from the file, determine it manually",
        default=False,
    )

    def draw(self, context):
        layout = self.layout

        def spacer(inpl):
            row = inpl.row()
            row.ui_units_y = 0.5
            row.label(text="")
            return row

        row = layout.row()

        row.label(text="STEPper import options:")

        col = layout.box()
        col = col.column(align=True)

        row = col.row()
        row.prop(self, "custom_scale")
        if self.custom_scale:
            row = col.row()
            row.prop(self, "user_scale")

        # row = col.row()
        # row.prop(self, "merge_distance")

        if bpy.context.scene.stepper.simpler_parameters:
            row = col.row()
            row.prop(self, "detail_level")

        else:
            row = col.row()
            row.prop(self, "lin_deflection")

            row = col.row()
            row.prop(self, "ang_deflection")

        # row = col.row()
        # row.prop(prg, "fw_as")

        row = col.row()
        row.prop(self, "up_as")

        row = col.row()
        row.prop(self, "hierarchy_types", text="Hierarchy")

    def execute(self, context):
        folder = os.path.dirname(self.filepath)

        # print(type(self.files))
        # print(dir(self.files))
        l_def, a_def = self.lin_deflection, self.ang_deflection
        if bpy.context.scene.stepper.simpler_parameters:
            a_def, l_def = calculate_detail_level(self.detail_level)

        import_files = [i.name for i in self.files]

        if self.override_file != "":
            import_files = [self.override_file]

        # iterate through the selected files
        for j, i in enumerate(import_files):
            # generate full path to file
            path_to_file = os.path.join(folder, i)
            print("Opening file:", path_to_file)
            load_step(
                context,
                path_to_file,
                custom_scale=self.user_scale if self.custom_scale else None,
                lin_deflection=l_def,
                ang_deflection=a_def,
                up_as=self.up_as,
                htypes=self.hierarchy_types,
            )
        return {"FINISHED"}


class STEP_OT_ClearCache(bpy.types.Operator):
    bl_idname = "object.occ_clear_cache"
    bl_label = "Clear STEP cache"
    bl_description = "Clear STEP cache, enabling the reload of a file"

    def execute(self, context):
        # utils.memorytrace_print()
        # global global_file_cache
        # items = list(global_file_cache.values())
        # for entry in items:
        #     for i, shp in enumerate(entry):
        #         label, color, tag = entry[shp]
        #         # shp.Nullify()

        global_file_cache.clear()
        return {"FINISHED"}


class STEP_OT_FixASCII(bpy.types.Operator):
    bl_idname = "object.occ_fix_ascii"
    bl_label = "Attempt STEP ASCII fix"
    bl_description = (
        "Attempt repairing invalid STEP characters.\n"
        "For files that crash the program when trying to load.\n"
        "A new file with _fix post-fix is created into the folder."
    )

    def execute(self, context):
        from pathlib import Path

        print("Attempting to format STEP file as ASCII")
        i_file = context.scene.stepper.fix_ascii_file
        p = Path(i_file)
        if i_file == "" or not p.exists():
            self.report(
                {"ERROR"},
                "File does not exist.",
            )
            return {"FINISHED"}
        print(p.stat().st_size // 1024, "kB")

        outf = Path(p.parent, Path(p.stem + "_fix.step"))

        with outf.open("w", encoding="ASCII") as fo:
            with p.open("rb") as f:
                while il := f.readline():
                    # for c in il:
                    #     if c > 127:
                    #         assert False, "Over 127"
                    fo.write(il.decode("ASCII"))

        self.report(
            {"INFO"},
            "Operation finished.",
        )
        return {"FINISHED"}


class STEP_OT_ReloadSTEP(bpy.types.Operator):
    bl_idname = "object.occ_reload_step"
    bl_label = "Reload STEP"
    bl_description = "Reload STEP file"

    @classmethod
    def poll(cls, context):
        return context.object is not None and "STEP_file" in context.object

    def execute(self, context):
        from . import importer

        filepath = context.object["STEP_file"]
        step_reader = importer.ReadSTEP(filepath)
        global_file_cache[filepath] = step_reader
        return {"FINISHED"}


class STEP_OT_RebuildSelected(bpy.types.Operator):
    bl_idname = "object.occ_rebuild_selected"
    bl_label = "Rebuild selected objects from the STEP file"
    bl_description = "Experimental: Causes issues on some shapes\n" + bl_label

    @classmethod
    def poll(cls, context):
        return context.object is not None and "STEP_file" in context.object

    def execute(self, context):
        meshes = {}
        prevname = ""
        curname = ""
        build_tags = set()
        rebuilt_meshes = set()
        my_selection = list(context.selected_objects)

        ang_def = context.scene.stepper.ang_deflection
        lin_def = context.scene.stepper.lin_deflection
        # merge_distance = context.scene.stepper.merge_distance
        if bpy.context.scene.stepper.simpler_parameters:
            ang_def, lin_def = calculate_detail_level(bpy.context.scene.stepper.detail_level)

        # select all objs with the same meshes
        for obj in my_selection:
            for other_obj in context.scene.objects:
                if obj.data == other_obj.data:
                    other_obj.select_set(True)

        # go through all selected and rebuild the meshes
        wm = bpy.context.window_manager
        wm.progress_begin(0, len(my_selection))
        for progress_count, obj in enumerate(my_selection):
            if obj.data.name not in meshes:
                meshes[obj.data.name] = obj.data
                sel_tag = obj["STEP_tag"]
                prevname = curname
                curname = obj["STEP_file"]
            else:
                assert meshes[obj.data.name] == obj.data

            if sel_tag in rebuilt_meshes:
                continue

            if prevname != curname:
                if curname in global_file_cache:
                    step_reader = global_file_cache[curname]
                    # shapes_labels = step_reader.output_shapes
                    tree = step_reader.tree
                else:
                    self.report(
                        {"ERROR"},
                        'STEP loader: Object "{}" not found in cache for file {}. '
                        "Please reload STEP file".format(obj.name, curname),
                    )
                    break

            for shp, node_index in tree.get_shapes():
                _, _, tag, name, _, _, _ = tree.nodes[node_index].get_values()
                if tag == sel_tag:
                    rebuilt_meshes.add(sel_tag)
                    print("Rebuilding:", sel_tag, obj.data.name)
                    build_mesh(step_reader, obj, shp, lin_def, ang_def)
                    obj.display_type = "TEXTURED"
                    build_tags.add(obj["STEP_tag"])
                    break

            wm.progress_update(progress_count)

        wm.progress_end()

        for obj in context.selected_objects:
            obj.display_type = "TEXTURED"

        return {"FINISHED"}


class STEP_PT_STEPper(bpy.types.Panel):
    bl_label = "STEPper: Build"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Stepper"

    def draw(self, context):
        prg = context.scene.stepper

        layout = self.layout

        # def spacer(inpl):
        #     row = inpl.row()
        #     row.ui_units_y = 0.5
        #     row.label(text="")
        #     return row

        row = layout.row()
        col = row.column(align=True)

        # row = col.row()
        # row.prop(prg, "merge_distance")

        if bpy.context.scene.stepper.simpler_parameters:
            row = col.row()
            row.prop(prg, "detail_level")

        else:
            row = col.row()
            row.prop(prg, "lin_deflection")

            row = col.row()
            row.prop(prg, "ang_deflection")

        layout = self.layout
        # layout.label(text="Used memory: {}".format(total_size(global_file_cache)))
        row = layout.row()
        row.operator(STEP_OT_RebuildSelected.bl_idname, text="Rebuild selected")


class STEP_PT_STEPper_Reload(bpy.types.Panel):
    bl_label = "STEPper: File"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Stepper"

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.operator(STEP_OT_ReloadSTEP.bl_idname, text="Reload STEP file")


class STEP_PT_STEPper_Debug(bpy.types.Panel):
    bl_label = "STEPper: Debug"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Stepper"

    def draw(self, context):
        layout = self.layout

        bxp = layout.box()
        bxp.label(text="Enforce ASCII")
        col = bxp.row().column(align=True)

        prg = context.scene.stepper
        row = col.row()
        row.prop(prg, "fix_ascii_file")
        row = col.row()
        row.operator("object.occ_fix_ascii", text="Attempt fix STEP charset")

        # row = layout.row()
        # row.label(text="Error messages:")

        if (
            context.object is not None
            and "STEP_file" in context.object
            and context.object["STEP_file"] in global_file_cache
        ):
            bxp = layout.box()
            bxp.label(text="Reported problems:")

            row = bxp.row()
            col = row.column(align=True)
            step_reader = global_file_cache[context.object["STEP_file"]]
            for k, v in step_reader.import_problems.items():
                row = col.row()
                row.label(text=k + ": " + repr(v))

            bxs = layout.box()
            bxs.label(text="Skipped shapes:")

            row = bxs.row()
            col = row.column(align=True)
            if len(step_reader.skipped_shapes) > 0:
                for v in step_reader.skipped_shapes:
                    row = col.row()
                    row.label(text=repr(v))
            else:
                row = col.row()
                row.label(text="No skipped shapes")

        else:
            bxp = layout.box()
            row = bxp.row()
            row.label(text="Select active STEP object")


class STEP_AddonPreferences(bpy.types.AddonPreferences):
    bl_idname = "STEPper"

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.prop(bpy.context.scene.stepper, "hack_skip_zero_solids")

        row = layout.row()
        row.prop(bpy.context.scene.stepper, "simpler_parameters")

        # row = layout.row()
        # row.prop(bpy.context.scene.stepper, "hierarchy_types")

        # row.operator(PMM_OT_EnsurePIP.bl_idname, text="Ensure PIP")
        # row.operator(PMM_OT_UpgradePIP.bl_idname, text="Upgrade PIP")
        # row.operator(PMM_OT_PIPList.bl_idname, text="List")


def menu_func_import(self, context):
    # TODO: add .stp
    self.layout.operator(ImportStepCADOperator.bl_idname, text="STEP (.step)")


classes = (
    PG_Stepper,
    ImportStepCADOperator,
    STEP_OT_ClearCache,
    STEP_OT_RebuildSelected,
    STEP_OT_ReloadSTEP,
    STEP_OT_FixASCII,
    STEP_PT_STEPper,
    STEP_PT_STEPper_Reload,
    STEP_PT_STEPper_Debug,
    STEP_AddonPreferences,
)


def register():
    for c in classes:
        bpy.utils.register_class(c)
    bpy.types.Scene.stepper = bpy.props.PointerProperty(type=PG_Stepper)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)


def unregister():
    for c in classes[::-1]:
        bpy.utils.unregister_class(c)
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)
    del bpy.types.Scene.stepper
