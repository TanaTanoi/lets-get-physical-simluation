import numpy as np

class Constraint:
    def __init__(self, number_of_verts, vert_a, vert_b=False, rest_length=False):
        self.vert_a = vert_a
        self.vert_b = vert_b
        self.rest_length = rest_length
        self.A = Constraint.A_matrix(self.type())
        self.S = Constraint.S_matrix(number_of_verts, vert_a, vert_b=vert_b)

    def type(self):
        if self.vert_a and self.vert_b:
            return "SPRING"
        else:
            return "FIXED"

    def A_matrix(constraint_type):
        if constraint_type == "SPRING":
            A = np.matrix([[0.5,0,0,-0.5,0,0],
                [0,0.5,0,0,-0.5,0],
                [0,0,0.5,0,0,-0.5],
                [-0.5,0,0,0.5,0,0],
                [0,-0.5,0,0,0.5,0],
                [0,0,-0.5,0,0,0.5]])
        else:
            A = np.identity(3)
        return A

    # def S_matrix(self, n):
    #     if self.type() == "SPRING":
    #         S = np.zeros((6, n * 3))
    #         S[0, self.vert_a] = 1
    #         S[1, self.vert_a + 1] = 1
    #         S[2, self.vert_a + 3] = 1
    #         S[3, self.vert_b] = 1
    #         S[4, self.vert_b + 1] = 1
    #         S[5, self.vert_b + 2] = 1
    #     else:
    #         S = np.zeros((3, n * 3))
    #         S[0, self.vert_a] = 1
    #         S[1, self.vert_a + 1] = 1
    #         S[2, self.vert_a + 3] = 1
    #     return S

    def calculateLHS(self, verts):
        if self.vert_a and self.vert_b:
            dir_vec = verts[self.vert_b] - verts[self.vert_b]
            dir_vec_length = np.linalg.norm(dir_vec)
            strech_amount = dir_vec_length - self.rest_length
            update_vec = strech_amount * 0.5  * (dir_vec / max(dir_vec_length, 0.001))
            v_a = verts[self.vert_a] - update_vec
            v_b = verts[self.vert_b] + update_vec
            S = self.S.T
            A = self.A.T
            B = A.T
            return S.dot(A).dot(B).dot(np.matrix(np.append(v_a, v_b)).T)
        else:
            S = self.S.T
            return S * verts[self.vert_a]

    def S_matrix(n, vert_a, vert_b=False):
        if vert_a and vert_b:
            S = np.zeros((6, n))
            S[0, vert_a] = 1
            S[1, vert_a] = 1
            S[2, vert_a] = 1

            S[3, vert_b] = 1
            S[4, vert_b] = 1
            S[5, vert_b] = 1
        else:
            S = np.zeros((3, n))
            S[:, vert_a] = 1
        return S