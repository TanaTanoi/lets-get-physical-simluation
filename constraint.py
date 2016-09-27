import numpy as np

class Constraint:
    def __init__(self, number_of_verts, vert_a, vert_b=False, rest_length=False, fixed_point=False):
        self.vert_a = vert_a
        self.vert_b = vert_b
        self.rest_length = rest_length
        self.fixed_point = fixed_point
        self.A = Constraint.A_matrix(self.type())
        self.S = Constraint.S_matrix(self.type(), number_of_verts, vert_a, vert_b=vert_b)

    def type(self):
        if self.vert_a is not False and self.vert_b is not False:
            return "SPRING"
        else:
            return "FIXED"

    def A_matrix(constraint_type):
        if constraint_type == "SPRING":
            # A = np.matrix(
            #     [[0.5,0,0,-0.5,0,0],
            #     [0,0.5,0,0,-0.5,0],
            #     [0,0,0.5,0,0,-0.5],
            #     [-0.5,0,0,0.5,0,0],
            #     [0,-0.5,0,0,0.5,0],
            #     [0,0,-0.5,0,0,0.5]]
            # )
            A = np.identity(6)
        else:
            A = np.identity(3)
        return A

    def calculateRHS(self, verts, b_array):
        if self.type() == "SPRING":
            dir_vec = verts[self.vert_a] - verts[self.vert_b]
            dir_vec_length = np.linalg.norm(dir_vec)
            strech_amount = dir_vec_length - self.rest_length

            dir_vec_normalized = dir_vec / dir_vec_length
            update_vec = (strech_amount  * dir_vec_normalized) * 0.5

            v_a = verts[self.vert_a] - update_vec
            v_b = verts[self.vert_b] + update_vec
            diff = (v_a - v_b)
            b_array[self.vert_a] += v_a
            b_array[self.vert_b] += v_b
        else:
            b_array[self.vert_a] += self.fixed_point

    def S_matrix(constraint_type, n, vert_a, vert_b=False):
        if constraint_type  == "SPRING":
            S = np.zeros((6, n * 3))
            S[0, vert_a * 3] = 1
            S[1, vert_a * 3 + 1] = 1
            S[2, vert_a * 3 + 2] = 1

            S[3, vert_b * 3] = 1
            S[4, vert_b * 3 + 1] = 1
            S[5, vert_b * 3 + 2] = 1
        else:
            S = np.zeros((3, n * 3))
            S[0, vert_a * 3] = 1
            S[1, vert_a * 3 + 1] = 1
            S[2, vert_a * 3 + 2] = 1
        return S