import numpy as np


def make_tri_hash(f):
    # TODO: should this be sorted?
    return frozenset([tuple(f[0]), tuple(f[1]), tuple(f[2])])


class TriData:
    """Associated data for a single triangle"""

    def __init__(self, indices, norms, uvs, color, material, mat_name, batch):
        assert len(norms) == 3, f"Norms len ({len(norms)}:{type(norms)}) should be 3"
        assert len(indices) == 3, f"Value: {indices}"
        assert len(uvs) == 3
        assert not isinstance(color, list)
        assert not isinstance(material, list)
        assert indices[0] != indices[1]
        assert indices[1] != indices[2]
        assert indices[2] != indices[0]
        assert mat_name == None or (isinstance(mat_name, str) and len(mat_name) > 0)

        self.indices = indices
        self.norms = norms
        # If color == None: means has no color defined
        self.color = color
        self.material = material
        self.material_name = mat_name
        self.batch = batch
        self.uvs = uvs


class TriMesh:
    """Triangle mesh. Array of triangles of which each item has three pointers
    to locations inside array of verts. Each triangle data is defined through TriData class"""

    def __init__(self, verts=None, tris=None, matrix=None):

        if verts is None and tris is None:
            self.tris = []
            self.verts = []
            self.batch_index = 0
            self.tri_hash = {}
            self.matrix = np.empty((3, 4), dtype=np.float32)

        else:
            assert verts is not None
            # assert tris is not None

            if matrix is None:
                matrix = np.empty((3, 4), dtype=np.float32)
            assert hasattr(matrix, "shape")
            assert matrix.shape == (3, 4)

            # TODO: batch_index only works inside one TriMesh instance, should be outside
            self.batch_index = 0
            self.matrix = matrix
            self.verts = verts
            self.tri_hash = {}

            # Only process tris if they exist
            # Might be vert-only input
            if tris is not None:
                self.tris = tris
                for i, t in enumerate(tris):
                    self.tris[i].batch = self.batch_index

                # Create tri hashes for overwrite detection
                for ti, t in enumerate(self.tris):
                    h = make_tri_hash([tuple(self.verts[i]) for i in t.indices])
                    self.tri_hash[h] = ti
            else:
                self.tris = None

    def check_same_face(self):
        """Check all tris for overlap. Return None if none found"""
        faces = set([])
        # TODO: all matches, not just the first one
        for ti, t in enumerate(self.tris):
            # Filter zero area and existing faces
            locs = [self.verts[i] for i in t.indices]
            same_loc = locs[0] == locs[1] or locs[1] == locs[2] or locs[2] == locs[0]
            same_face = False
            f_hash = tuple(sorted(locs))
            if f_hash not in faces:
                faces.add(f_hash)
            else:
                same_face = True
            if same_loc or same_face:
                res = {}
                if same_loc:
                    res["same_loc"] = (ti, t.indices, locs)
                if same_face:
                    res["same_face"] = (ti, t.indices, f_hash)
                return res
        return None

    def filter_zero_area(self):
        """Remove all tris with zero area"""
        new_tris = []
        for t in self.tris:
            locs = tuple(self.verts[i] for i in t.indices)
            same_loc = locs[0] == locs[1] or locs[1] == locs[2] or locs[2] == locs[0]
            if not same_loc:
                new_tris.append(t)
        self.tris = new_tris

    def filter_same_face(self):
        """Remove all duplicate tris"""
        new_tris = []
        faces = set([])
        for t in self.tris:
            locs = tuple(self.verts[i] for i in t.indices)
            f_hash = tuple(sorted(locs))
            if f_hash not in faces:
                faces.add(f_hash)
                new_tris.append(t)
        self.tris = new_tris

    def fuse_verts(self):
        """Make verts in identical locations the same, update tris"""

        verts = {}
        tri_map = {}
        new_verts = []
        new_index = 0
        for vi, v in enumerate(self.verts):
            # Find duplicate verts based on location "hash"
            v_hash = tuple(v)
            if v_hash not in verts:
                new_verts.append(v)
                verts[v_hash] = new_index
                tri_map[vi] = new_index
                new_index += 1
            else:
                # Vert already exists in new_verts
                tri_map[vi] = verts[v_hash]

        # Change tri indices to match new fused verts
        new_tris = []
        for ti, t in enumerate(self.tris):
            idcs = tuple(tri_map[i] for i in t.indices)
            new_tris.append(self.tris[ti])
            new_tris[-1].indices = idcs

        # Save new data
        self.verts = new_verts
        self.tris = new_tris

    def add_mesh(self, other):
        assert type(other) == TriMesh
        for i in range(len(other.tris)):
            otr = other.tris[i]
            self.add_tri(
                [other.verts[t] for t in otr.indices],
                otr.norms,
                otr.color,
                otr.material,
                otr.material_name,
                otr.uvs,
                self.batch_index,
            )
        self.batch_index += 1

    def add_tri(self, verts, norms, colors, material, mat_name, uvs, batch_id):
        # TODO: write tests for this
        assert len(verts) == 3
        vc_s = len(self.verts)
        self.verts += verts
        tri = (vc_s, vc_s + 1, vc_s + 2)
        self.tris.append(TriData(tri, norms, uvs, colors, material, mat_name, batch_id))
        self.tri_hash[make_tri_hash([tuple(verts[i]) for i in range(3)])] = len(self.tris) - 1
        assert self.verts[self.tris[-1].indices[0]] == verts[0]
        assert self.verts[self.tris[-1].indices[2]] == verts[2]

    def add_tri_overwrite(self, verts, otr, batch_id, no_color_overwrite=True):
        assert len(verts) == 3
        fhash = make_tri_hash([tuple(verts[i]) for i in range(3)])
        vc_s = len(self.verts)

        if fhash not in self.tri_hash:
            self.verts += verts
            tri = (vc_s, vc_s + 1, vc_s + 2)
            self.tris.append(
                TriData(
                    tri,
                    otr.norms,
                    otr.uvs,
                    otr.color,
                    otr.material,
                    otr.material_name,
                    otr.batch,
                )
            )
            self.tri_hash[fhash] = len(self.tris) - 1
            assert self.verts[self.tris[-1].indices[0]] == verts[0]
            assert self.verts[self.tris[-1].indices[2]] == verts[2]
        else:
            tri = self.tri_hash[fhash]
            # Don't overwrite color with no-color if no_color_overwrite
            if not (no_color_overwrite and self.tris[tri].color and otr.color == None):
                # tri indices are the same
                self.tris[tri].color = otr.color
                self.tris[tri].material = otr.material
                self.tris[tri].material_name = otr.material_name
                self.tris[tri].uvs = otr.uvs

    def add_mesh_overwrite_identical(self, other):
        assert type(other) == TriMesh
        for i in range(len(other.tris)):
            otr = other.tris[i]
            self.add_tri_overwrite([other.verts[t] for t in otr.indices], otr, self.batch_index)
        # TODO: batch_index should come from outside, not managed by the TriMesh class
        self.batch_index += 1

    def colorize(self, col):
        "Fill with color, color can be None"
        # assert len(self.tris) > 0
        for t in range(len(self.tris)):
            self.tris[t].color = col

    def set_material_name(self, name):
        "Set material name for all tris"
        # assert len(self.tris) > 0
        for t in range(len(self.tris)):
            self.tris[t].material_name = name

    def fill_empty_color(self):
        "Fill color==None with undef_color (currently pink)"
        # pink
        undef_color = (1.0, 0.0, 1.0)
        if len(self.tris) == 0:
            # Empty mesh
            return
        for t in range(len(self.tris)):
            if self.tris[t].color == None:
                self.tris[t].color = undef_color

    def add_to_bm(self, bm, edges_as_seams=False, discontinuity_as_sharp=False):
        """ASSUMPTION: empty BMesh"""
        assert len(bm.faces) == 0
        assert len(bm.verts) == 0

        verts = []
        for vi, v in enumerate(self.verts):
            nv = bm.verts.new(v)
            nv.index = vi
            verts.append(nv)

        if self.tris:
            for ti, t in enumerate(self.tris):
                nf = bm.faces.new((verts[i] for i in t.indices))
                nf.index = ti

            # mark all non-manifold edges as seams
            if edges_as_seams:
                for e in bm.edges:
                    f = e.link_faces
                    if len(f) == 2 and self.tris[f[0].index].batch != self.tris[f[1].index].batch:
                        e.seam = True
                    # if not e.is_manifold:
                    #     e.seam = True

            # gather all normals based on vert index
            # compare dot products for those gathered
            # if above threshold, vert index discontinuity = true
            # go through edges and based on vert indices match mark as sharp

            def _project_plane_normalize(plane, vec):
                if False:
                    l0 = np.linalg.norm(plane)
                    l1 = np.linalg.norm(vec)
                    assert l0 > 0.99 and l0 < 1.01
                    assert l1 > 0.99 and l1 < 1.01
                prj = vec - (plane * np.dot(plane, vec))
                prjn = np.linalg.norm(prj)
                if prjn == 0.0:
                    prj = np.array([0.0, 0.0, 1.0])
                else:
                    prj /= prjn
                return prj

            def _prjtest(plane, norms, margin):
                # Project norms to plane: norm - (plane * dot(plane, norm))
                # .. and calc dots
                p0 = _project_plane_normalize(plane, norms[0])
                dmax = 1.0
                for i in norms[1:]:
                    prj = _project_plane_normalize(plane, i)
                    dd = np.dot(p0, prj)
                    if dd < dmax:
                        dmax = dd
                if dmax < 1.0 - margin:
                    return True
                return False

            if discontinuity_as_sharp:
                # TODO: promote margin to an actual function input/parameter
                margin = 0.02

                # norms_in_vert = defaultdict(list)
                # for t in self.tris:
                #     for ni, n in enumerate(t.norms):
                #         norms_in_vert[t.indices[ni]].append(n)

                # Needs to be based on (2) face verts of the edge, not all faces of vert
                for e in bm.edges:
                    fi = [f.index for f in e.link_faces]
                    if len(fi) != 2:
                        continue

                    t0, t1 = self.tris[fi[0]], self.tris[fi[1]]

                    # find connected face filtered norms for edge verts
                    # TODO: maybe optimize this, (test if all are clockwise)
                    e_norms = [[], []]
                    for i in range(3):
                        if t0.indices[i] == e.verts[0].index:
                            e_norms[0].append(t0.norms[i])
                        if t1.indices[i] == e.verts[0].index:
                            e_norms[0].append(t1.norms[i])
                        if t0.indices[i] == e.verts[1].index:
                            e_norms[1].append(t0.norms[i])
                        if t1.indices[i] == e.verts[1].index:
                            e_norms[1].append(t1.norms[i])

                    # Check the dots on plane defined by the edge as normal
                    plane = np.array((e.verts[0].co - e.verts[1].co).normalized())
                    if _prjtest(plane, e_norms[0], margin) and _prjtest(plane, e_norms[1], margin):
                        e.smooth = False
                    else:
                        e.smooth = True

    def get_loop_colors(self):
        "Return colors in triangle loop creation order"
        cols = []
        for t in self.tris:
            for _ in range(3):
                cols.append(t.color)
        return cols

    def get_loop_material_names(self):
        "Return material names in triangle loop creation order"
        cols = []
        for t in self.tris:
            for _ in range(3):
                cols.append(t.material_name)
        return cols

    def get_loop_normals(self):
        "Return normals in triangle loop creation order"
        norms = []
        for t in self.tris:
            for i in range(3):
                norms.append(t.norms[i])
        return norms

    def get_loop_uvs(self):
        "Return UVs in triangle loop creation order"
        uvs = []
        for t in self.tris:
            for i in range(3):
                uvs.append(t.uvs[i])
        return uvs
