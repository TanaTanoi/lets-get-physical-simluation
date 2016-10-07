import numpy as np
import numpy.linalg as linalg
import face
import constraint
import noise
import math

np.set_printoptions(linewidth=2000)
class Model:
    drag = 0.95
    def __init__(self, verts, faces, neighbours=[], uvs = [], constraints=[]):
        self.n = len(verts)
        self.verts = verts
        self.rendering_verts = verts
        self.faces = faces
        self.neighbours = neighbours
        for i in range(len(neighbours)):
            neighbours[i] = list(set(neighbours[i]))
        self.uvs = uvs
        self.velocities = np.zeros((self.n, 3))
        self.mass_matrix = np.identity(self.n)
        self.constraints = constraints
        self.stepsize = 1
        self.global_matrix = self.calculate_global_matrix()
        self.count = 0
        self.wind_magnitude = 0.3
        print(self.global_matrix)

    def center(self):
        middle_point = np.array((0., 0., 0.))
        for vert in self.verts:
            middle_point += vert
        middle_point = middle_point / float(self.n)
        middle_point *= -1
        for vert_id in range(self.n):
            self.verts[vert_id] += middle_point

    def simulate(self):
        self.count += 1
        forces = self.wind_forces(self.count)
        # forces = np.zeros(((self.n, 3)))
        # forces[1, 2] = 1
        # forces[0:-1, 1] = 1
        acc = (self.stepsize * self.stepsize) *  linalg.inv(self.mass_matrix).dot(forces)
        dist = self.velocities * self.stepsize
        s_n = self.verts + dist + acc
        q_n_1 = s_n
        M = self.mass_matrix / (self.stepsize * self.stepsize)
        self.calculate_cell_rotations(s_n)
        for i in range(1):
            # b_array = M.dot(q_n_1)
            b_array = np.zeros((self.n + 2, 3))
            for i in range(self.n):
                b_array[i] = self.calculate_b_for(i)
            b_array[-2] = self.verts[0]
            # b_array[-1] = self.verts[self.n - math.sqrt(self.n)]
            b_array[-1] = self.verts[1]
            # b_array[-1] = 1
            # print(b_array)
            q_n_1 = np.linalg.solve(self.global_matrix, b_array)
            q_n_1 = q_n_1[0:-2, :] # all but last constrainted point

        self.velocities = (q_n_1 - self.verts) * self.drag / self.stepsize
        self.rendering_verts = q_n_1
        self.verts = self.rendering_verts

    def calculate_b_for(self, i):
        b = np.zeros((1, 3))
        neighbours = self.neighbours[i]
        for j in neighbours:
            r_ij = self.cell_rotations[i] + self.cell_rotations[j]
            p_ij = self.verts[i] - self.verts[j]
            b += r_ij.dot(p_ij) * 0.5
            # 1/2 for weight
        return b

    def wind_forces(self, time):
        time /= 10000
        forces = np.zeros(((self.n, 3)))
        angle = noise.pnoise1(time) * math.pi * 2
        forces[0:-1, 0] = math.cos(angle) * self.wind_magnitude
        forces[0:-1, 2] = math.sin(angle) * self.wind_magnitude
        return forces

    def calculate_cell_rotations(self, s_n):
        self.cell_rotations = np.zeros((self.n, 3, 3))
        print("Calculating Cell Rotations")
        for vert_id in range(self.n):
            rotation = self.calculate_rotation_matrix_for_cell(vert_id, s_n)
            print(rotation)
            self.cell_rotations[vert_id] = rotation

    def potential_for_triangle(self, face, prime_verts):
        v1 = self.verts[face.v1]
        v2 = self.verts[face.v2]
        v3 = self.verts[face.v3]
        x_f = np.matrix((v2 - v1, v3 - v1, v2 - v3))

        v1 = prime_verts[face.v1]
        v2 = prime_verts[face.v2]
        v3 = prime_verts[face.v3]
        x_g = np.matrix((v2 - v1, v3 - v1, v2 - v3))

        combined = x_g.dot(np.linalg.pinv(x_f))
        U, s, V_t = np.linalg.svd(combined)

        s_p = np.diag(np.clip(s, 0, 1))
        return np.linalg.norm(s - s_p) ** 2

    def clamped_svd_for_matrix(self, matrix):
        U, s, V_t = np.linalg.svd(matrix)

        s = np.diag(np.clip(s, 0, 1))
        return U.dot(s).dot(V_t)

    def calculate_global_matrix(self):
        # print(self.n)
        # M = np.identity(self.n * 3)# / (self.stepsize * self.stepsize)
        # sum_m = np.zeros((self.n * 3, self.n * 3))
        # 
        # for con in self.constraints:
        #     S = con.S
        #     A = con.A
        #     x = S.T.dot(A.T.dot(A.dot(S)))
        #     sum_m += x
        # 
        # return M + sum_m

        M = np.identity(self.n + 2) / (self.stepsize * self.stepsize)
        weights = np.zeros((self.n + 2, self.n + 2))
        weight_sum = np.zeros((self.n + 2, self.n + 2))
        for con in self.constraints:
            if con.type() == "SPRING":
                weights[con.vert_a, con.vert_b] = 1
                weights[con.vert_b, con.vert_a] = 1
                weight_sum[con.vert_a, con.vert_a] += 1
                weight_sum[con.vert_b, con.vert_b] += 1

        # Fix some arbitraty last start point and end point
        x = weight_sum - weights
        x[0, -2] = 1
        x[-2, 0] = 1
        x[1, -1] = 1
        x[-1, 1] = 1
        return x
        # x = np.zeros((self.n, self.n))
        # x = np.identity(self.n)
        # for face in self.faces:
        #     for v_id in face.vertex_ids():
        #         x[v_id, v_id] += 2
        # return x

    def calculate_rotation_matrix_for_cell(self, vert_id, s_n):
        covariance_matrix = self.calculate_covariance_matrix_for_cell(vert_id, s_n)

        U, s, V_transpose = np.linalg.svd(covariance_matrix)

        rotation = V_transpose.transpose().dot(U.transpose())# * 0.5
        # if np.linalg.det(rotation) <= 0:
        #     U[:0] *= -1
        #     rotation = V_transpose.transpose().dot(U.transpose())
        return rotation

    def calculate_covariance_matrix_for_cell(self, vert_id, s_n):
        # result = np.zeros((3, 3))
        # neighbours = self.neighbours[vert_id]
        # for n_id in neighbours:
        #     e_ij = self.verts[vert_id] - self.verts[n_id]
        #     e_ij_p = s_n[vert_id] - s_n[n_id]
        #     e_ij_p = np.matrix(e_ij_p)
        #     result += e_ij.dot(e_ij_p.T)
        # return result
        vert_i_prime = s_n[vert_id]
        vert_i = self.verts[vert_id]

        neighbour_ids = self.neighbours[vert_id]
        number_of_neighbours = len(neighbour_ids)

        P_i =       np.zeros((3, number_of_neighbours))
        P_i_prime = np.zeros((3, number_of_neighbours))

        for n_i in range(number_of_neighbours):
            n_id = neighbour_ids[n_i]

            vert_j = self.verts[n_id]
            vert_j_prime = s_n[n_id]

            P_i[:, n_i] = (vert_i - vert_j)
            P_i_prime[:, n_i] = (vert_i_prime - vert_j_prime)

        P_i_prime = P_i_prime.transpose()
        return P_i.dot(P_i_prime)

    def generate_plane(width, height, MAX_WIDTH_SIZE=500, MAX_HEIGHT_SIZE=300):

        n = width * height
        width_gap = MAX_WIDTH_SIZE / width
        height_gap = -MAX_HEIGHT_SIZE / height

        verts = np.zeros((n, 3))
        faces = []
        constraints = []
        neighbours = []
        uvs = np.zeros((n, 2))
        for x in range(width):
            for y in range(height):
                neighbours.append([]) # might as well add neigbour arrays here too
                verts[ x + (y * width) ] = np.array((x * width_gap, y * height_gap, 0 ))
                uvs[ x + (y * width) ]   = np.array(( (x % width) / width, 1 - (y % height) / height ))

        for v_id in range(n):
            # if its a thing on the end
            if v_id % width == width - 1:
                if v_id < n - 1:
                    Model.add_spring_constraint(verts, v_id, v_id + width, constraints)
                continue
            # points before the bottom line
            if v_id < n - width:
                v_1 = v_id
                v_2 = v_id + width
                v_3 = v_id + 1
                Model.add_face(v_1, v_2, v_3, faces, neighbours)
                Model.add_spring_constraint_set(verts, v_1, v_2, v_3, constraints)
            # points after the first line
            if v_id >= width:
                v_1 = v_id
                v_2 = v_id + 1
                v_3 = v_id - (width - 1)
                Model.add_face(v_1, v_2, v_3, faces,neighbours)
            # the lines along the bottom
            if v_id >= n - width and v_id < n:
                Model.add_spring_constraint(verts, v_id, v_id + 1, constraints)
        # fix top and bottom left corners
        Model.add_fixed_constraint(n, 0, verts[0], constraints)
        bottom_left = width * (height - 1)
        Model.add_fixed_constraint(n, bottom_left, verts[bottom_left], constraints)
        return Model(verts, faces, neighbours, uvs, constraints=constraints)

    def add_face(v_1, v_2, v_3, faces, neighbours):
        neighbours[v_1].append(v_2);neighbours[v_1].append(v_3)
        neighbours[v_2].append(v_1);neighbours[v_2].append(v_3)
        neighbours[v_3].append(v_2);neighbours[v_3].append(v_1)
        faces.append(face.Face(v_1, v_2, v_3))

    def add_fixed_constraint(number_of_verts, v_id, vert, constraints):
        constraints.append(constraint.Constraint(number_of_verts, v_id, fixed_point=vert))

    def add_spring_constraint_set(verts, v_1, v_2, v_3, constraints):
        Model.add_spring_constraint(verts, v_1, v_2, constraints)
        Model.add_spring_constraint(verts, v_2, v_3, constraints)
        Model.add_spring_constraint(verts, v_3, v_1, constraints)

    def add_spring_constraint(verts, v_1, v_2, constraints):
        rest_length = linalg.norm(verts[v_1] - verts[v_2])
        number_of_verts = len(verts)
        constraints.append(constraint.Constraint(number_of_verts, v_1, v_2, rest_length=rest_length))