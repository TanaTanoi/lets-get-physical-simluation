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
        forces=np.zeros(((self.n, 3)))
        forces[:, 0] =+ 10
        acc = (self.stepsize * self.stepsize) *  linalg.inv(self.mass_matrix).dot(forces)
        dist = (self.velocities) * self.stepsize
        new_pos = self.verts + dist + acc
        # new_pos = self.verts
        # new_pos[:, 1] += 10
        q_n_1 = new_pos

        b_array = np.zeros((self.n, 3)) + new_pos

        for con in self.constraints:
            if con.type() == "SPRING":
                dir_vec = self.verts[con.vert_b] - self.verts[con.vert_b]
                dir_vec_length = linalg.norm(dir_vec)
                strech_amount = dir_vec_length - con.rest_length
                update_vec = strech_amount * 0.5  * (dir_vec / max(dir_vec_length, 0.001))
                b_array[con.vert_a] += update_vec
                b_array[con.vert_b] -= update_vec
            else:
                b_array[con.vert_a] += self.verts[con.vert_a]

        print(new_pos)
        print(b_array)
        gb = self.global_matrix()
        print(gb)
        self.verts = linalg.solve(gb, b_array.flatten())
        self.verts = np.reshape(self.verts,(self.n, 3)) * 100
        print(self.verts)

    def global_matrix(self):
        M = np.identity(self.n * 3) / (self.stepsize * self.stepsize)
        sum_m = np.zeros((self.n * 3, self.n * 3))

        for con in self.constraints:
            if con.type() == "SPRING":
                S = np.zeros((6, self.n * 3))
                S[0, con.vert_a] = 1
                S[1, con.vert_a + 1] = 1
                S[2, con.vert_a + 3] = 1
                S[3, con.vert_b] = 1
                S[4, con.vert_b + 1] = 1
                S[5, con.vert_b + 2] = 1
            else:
                S = np.zeros((3, self.n * 3))
                S[0, con.vert_a] = 1
                S[1, con.vert_a + 1] = 1
                S[2, con.vert_a + 3] = 1
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
                Model.add_fixed_constraint(v_id, constraints)
        return Model(verts, faces, uvs, constraints=constraints)

    def add_fixed_constraint(v_id, constraints):
        constraints.append(constraint.Constraint(v_id))

    def add_spring_constraint_set(verts, v_1, v_2, v_3, constraints):
        Model.add_spring_constraint(verts, v_1, v_2, constraints)
        Model.add_spring_constraint(verts, v_2, v_3, constraints)
        Model.add_spring_constraint(verts, v_3, v_1, constraints)

    def add_spring_constraint(verts, v_1, v_2, constraints):
        rest_length = linalg.norm(verts[v_1] - verts[v_2])
        constraints.append(constraint.Constraint(v_1, v_2, rest_length))