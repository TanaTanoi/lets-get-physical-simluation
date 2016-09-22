import numpy as np
import numpy.linalg as linalg
import face
import constraint
np.set_printoptions(linewidth=200)
class Model:
    def __init__(self, verts, faces, uvs = [], constraints=[]):
        self.n = len(verts)
        self.verts = verts
        self.faces = faces
        self.uvs = uvs
        self.velocities = np.zeros((self.n, 3))
        self.mass_matrix = np.identity(self.n)
        self.constraints = constraints
        self.stepsize = 0.1
        self.global_matrix = self.calculate_global_matrix()

    def center(self):
        middle_point = np.array((0., 0., 0.))
        for vert in self.verts:
            middle_point += vert
        middle_point = middle_point / float(self.n)
        middle_point *= -1
        print(middle_point)
        for vert_id in range(self.n):
            self.verts[vert_id] += middle_point

    def simulate(self):
        forces = np.zeros(((self.n, 3)))
        forces[:, 1] =- 100
        acc = (self.stepsize * self.stepsize) *  linalg.inv(self.mass_matrix).dot(forces)
        dist = self.velocities * self.stepsize
        new_pos = self.verts + dist + acc

        b_array = np.zeros((self.n, 3)) + new_pos
        for i in range(10):
            for con in self.constraints:
                if con.type() == "SPRING":
                    dir_vec = self.verts[con.vert_b] - self.verts[con.vert_b]
                    dir_vec_length = linalg.norm(dir_vec)
                    strech_amount = dir_vec_length - con.rest_length
                    update_vec = strech_amount * 0.5  * (dir_vec / max(dir_vec_length, 0.001))
                    v_a = self.verts[con.vert_a] - update_vec
                    v_b = self.verts[con.vert_b] + update_vec
                    S = con.S.T
                    A = con.A.T
                    B = A.T
                    v = S.dot(A).dot(B)
                    qwe = np.matrix(np.append(v_a, v_b)).T
                    v = v * qwe

                    b_array += v
                else:
                    S = con.S.T
                    # A = con.A
                    # B = A
                    # v = S.dot(A).dot(B)
                    v = S * self.verts[con.vert_a]
                    b_array += v

        print(new_pos)
        print(b_array)
        self.verts = linalg.solve(self.global_matrix, b_array)
        print(self.verts)

    def calculate_global_matrix(self):
        M = np.identity(self.n) / (self.stepsize * self.stepsize) / 10
        sum_m = np.zeros((self.n, self.n))

        for con in self.constraints:
            S = con.S
            A = con.A
            sum_m += S.T.dot(A.T).dot(A).dot(S)
        return M + sum_m

    def generate_plane(width, height, MAX_WIDTH_SIZE=500, MAX_HEIGHT_SIZE=300):

        n = width * height
        width_gap = MAX_WIDTH_SIZE / width
        height_gap = MAX_HEIGHT_SIZE / height

        verts = np.zeros((n, 3))
        faces = []
        constraints = []
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
                v_1 = v_id
                v_2 = v_id + width
                v_3 = v_id + 1
                faces.append(face.Face(v_1, v_2, v_3))
                Model.add_spring_constraint_set(verts, v_1, v_2, v_3, constraints)
            if v_id >= width:
                v_1 = v_id
                v_2 = v_id + 1
                v_3 = v_id - (width - 1)
                faces.append(face.Face(v_1, v_2, v_3))
                Model.add_spring_constraint_set(verts, v_1, v_2, v_3, constraints)
            if v_id < width:
                Model.add_fixed_constraint(n, v_id, constraints)
        return Model(verts, faces, uvs, constraints=constraints)

    def add_fixed_constraint(number_of_verts, v_id, constraints):
        constraints.append(constraint.Constraint(number_of_verts, v_id))

    def add_spring_constraint_set(verts, v_1, v_2, v_3, constraints):
        Model.add_spring_constraint(verts, v_1, v_2, constraints)
        Model.add_spring_constraint(verts, v_2, v_3, constraints)
        Model.add_spring_constraint(verts, v_3, v_1, constraints)

    def add_spring_constraint(verts, v_1, v_2, constraints):
        rest_length = linalg.norm(verts[v_1] - verts[v_2])
        number_of_verts = len(verts)
        constraints.append(constraint.Constraint(number_of_verts, v_1, v_2, rest_length))