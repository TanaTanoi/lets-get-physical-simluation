import numpy as np

class Model:
    def __init__(self, verts, faces):
        self.verts = verts
        self.faces = faces

    def center(self):
        middle_point = np.array((0., 0., 0.))
        for vert in self.verts:
            middle_point += vert
        middle_point = middle_point / float(len(self.verts))
        middle_point *= -1
        print(middle_point)
        for vert_id in range(len(self.verts)):
            self.verts[vert_id] += middle_point