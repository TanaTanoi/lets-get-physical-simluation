import numpy as np
import face

class Model:
    def __init__(self, verts, faces, uvs = []):
        self.verts = verts
        self.faces = faces
        self.uvs = uvs

    def center(self):
        middle_point = np.array((0., 0., 0.))
        for vert in self.verts:
            middle_point += vert
        middle_point = middle_point / float(len(self.verts))
        middle_point *= -1
        print(middle_point)
        for vert_id in range(len(self.verts)):
            self.verts[vert_id] += middle_point

    def generate_plane(width, height):

        MAX_WIDTH_SIZE = 500
        MAX_HEIGHT_SIZE = 300

        n = width * height
        width_gap = MAX_WIDTH_SIZE / width
        height_gap = MAX_HEIGHT_SIZE / height

        verts = np.zeros((n, 3))
        faces = []
        uvs = np.zeros((n, 2))
        for x in range(width):
            for y in range(height):
                verts[ x + (y * width) ] = np.array((x * width_gap, y * height_gap, 0 ))
                uvs[ x + (y * width) ]   = np.array(( (x % width) / width, (y % height) / height ))

        for v_id in range(n):
            # if its a thing on the end
            if v_id % width == width - 1:
                continue
            if v_id < n - width:
                faces.append(face.Face(v_id, v_id + width, v_id + 1))
            if v_id >= width:
                faces.append(face.Face(v_id, v_id + 1, v_id - (width - 1)))

        return Model(verts, faces, uvs)