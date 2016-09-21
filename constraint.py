import numpy as np

class Constraint:
    def __init__(self, vert_a, vert_b=False, rest_length=False):
        self.vert_a = vert_a
        self.vert_b = vert_b
        self.rest_length = rest_length
        self.A = self.A_matrix()

    def type(self):
        if self.vert_a and self.vert_b:
            return "SPRING"
        else:
            return "FIXED"

    def A_matrix(self):
        if self.type() == "SPRING":
            A = np.matrix([[0.5,0,0,-0.5,0,0],
                [0,0.5,0,0,-0.5,0],
                [0,0,0.5,0,0,-0.5],
                [-0.5,0,0,0.5,0,0],
                [0,-0.5,0,0,0.5,0],
                [0,0,-0.5,0,0,0.5]])
        else:
            A = np.identity(3)
        return A