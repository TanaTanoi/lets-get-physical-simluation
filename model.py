import numpy as np
import numpy.linalg as linalg
import face
import constraint
import noise
import math

np.set_printoptions(linewidth=2000)
class Model:
    drag = 0.95
    arap = False
    def __init__(self, verts, faces, neighbours=[], uvs = [], constraints=[]):
        self.n = len(verts)
        self.verts = verts
        self.rendering_verts = verts
        self.faces = faces
        self.neighbours = neighbours
        for i in range(len(neighbours)):
            neighbours[i] = list(set(neighbours[i]))
        self.verts_to_tri = []
        for i in range(self.n):
            in_faces = []
            for face in self.faces:
                if i in face.vertex_ids():
                    in_faces.append(face)
            self.verts_to_tri.append(in_faces)
        self.uvs = uvs
        self.stepsize = 1
        self.velocities = np.zeros((self.n, 3))
        self.mass_matrix = np.identity(self.n)
        self.constraints = constraints
        self.fixed_points = []
        for con in self.constraints:
            if con.type() == "FIXED":
                self.fixed_points.append(con)
        if self.arap:
            self.global_matrix = self.calculate_cell_global_matrix()
        else:
            self.global_matrix = self.calculate_triangle_global_matrix()
        self.count = 0
        self.wind_magnitude = 0
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
        forces[:, 1] = -10
        forces[0, :] = 0
        forces[20, :] = 0
        # forces = np.zeros(((self.n, 3)))
        # if self.count < 50:
        #     forces[1:, 2] = -100
        # else:
        # forces[1:, 2] = 10
        # if self.count > 100:
        #     self.count = 0
        acc = (self.stepsize * self.stepsize) *  linalg.inv(self.mass_matrix).dot(forces)
        dist = self.velocities * self.stepsize
        s_n = self.verts + dist + acc
        q_n_1 = s_n
        M = self.mass_matrix / (self.stepsize * self.stepsize)
        if(self.arap):
            self.calculate_cell_rotations(s_n)
        for i in range(1):
            b_array = np.zeros((self.n + len(self.fixed_points), 3))
            # b_array[0:-1] = M.dot(q_n_1)
            for i in range(self.n):
                if(self.arap):
                    b_array[i] = self.calculate_b_for_cell(i)
                else:
                    b_array[i] = self.calculate_b_for_triangle(i, s_n)
            for con_i in range(len(self.fixed_points)):
                con = self.fixed_points[con_i]
                b_array[-(con_i + 1)] = self.verts[con.vert_a]
            q_n_1 = np.linalg.solve(self.global_matrix, b_array)
            q_n_1 = q_n_1[0:-len(self.fixed_points), :] # Don't grab the unwanted fixed points

        self.velocities = np.around(((q_n_1 - self.verts)) / self.stepsize, 11)
        self.rendering_verts = q_n_1
        # self.verts = self.rendering_verts

    def calculate_b_for_triangle(self, i, s_n):
        b = np.zeros((1, 3))
        for face in self.verts_to_tri[i]:
            T = self.potential_for_triangle(face, s_n, i)
            for o_v in face.other_points(i):
                b += T.dot(self.verts[i] - self.verts[o_v])
        return b

    def calculate_b_for_cell(self, i):
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
        forces[1:, 0] = math.cos(angle) * round(self.wind_magnitude)
        forces[1:, 2] = math.sin(angle) * round(self.wind_magnitude)
        return forces

    def calculate_cell_rotations(self, s_n):
        self.cell_rotations = np.zeros((self.n, 3, 3))
        for vert_id in range(self.n):
            rotation = self.calculate_rotation_matrix_for_cell(vert_id, s_n)
            self.cell_rotations[vert_id] = rotation

    def potential_for_triangle(self, face, prime_verts, point):
        other_points = face.other_points(point)
        v1 = self.verts[point]
        v2 = self.verts[other_points[0]]
        v3 = self.verts[other_points[1]]
        x_f = np.matrix((v3 - v1, v2 - v1, (0,0,0))).T

        v1 = prime_verts[point]
        v2 = prime_verts[other_points[0]]
        v3 = prime_verts[other_points[1]]
        x_g = np.matrix((v3 - v1, v2 - v1, (0,0,0))).T

        combined = x_g.dot(np.linalg.pinv(x_f))
        return self.clamped_svd_for_matrix(combined)
        # return np.identity(3)

    def clamped_svd_for_matrix(self, matrix):
        U, s, V_t = np.linalg.svd(matrix)
        # s = np.diag(s)
        s = np.diag(np.clip(s, -1000, 1))
        return np.around(U.dot(s).dot(V_t), 11)

    def calculate_triangle_global_matrix(self):
        fixed_point_num = len(self.fixed_points)

        weights = np.zeros((self.n + fixed_point_num, self.n + fixed_point_num))
        weight_sum = np.zeros((self.n + fixed_point_num, self.n + fixed_point_num))
        for face in self.faces:
            verts = face.vertex_ids()
            for k in range(3):
                v_1 = verts[k]
                v_2 = verts[(k + 1) % 3]
                weights[v_1, v_2] += 1
                weights[v_2, v_1] += 1
                weight_sum[v_1, v_1] += 1
                weight_sum[v_2, v_2] += 1

        x = weight_sum - weights
        for  i in range(fixed_point_num):
            con = self.fixed_points[i]
            x[con.vert_a, -(i + 1)] = 1
            x[-(i + 1), con.vert_a] = 1
        return x

    def calculate_cell_global_matrix(self):
        fixed_point_num = len(self.fixed_points)

        M = np.identity(self.n + fixed_point_num) / (self.stepsize * self.stepsize)
        weights = np.zeros((self.n + fixed_point_num, self.n + fixed_point_num))
        weight_sum = np.zeros((self.n + fixed_point_num, self.n + fixed_point_num))

        for con in self.constraints:
            if con.type() == "SPRING":
                weights[con.vert_a, con.vert_b] = 1
                weights[con.vert_b, con.vert_a] = 1
                weight_sum[con.vert_a, con.vert_a] += 1
                weight_sum[con.vert_b, con.vert_b] += 1

        # Fix some arbitraty last start point and end point
        x = weight_sum - weights
        for  i in range(fixed_point_num):
            con = self.fixed_points[i]
            x[con.vert_a, -(i + 1)] = 1
            x[-(i + 1), con.vert_a] = 1
        print(x)
        return x

    def calculate_rotation_matrix_for_cell(self, vert_id, s_n):
        covariance_matrix = self.calculate_covariance_matrix_for_cell(vert_id, s_n)

        U, s, V_transpose = np.linalg.svd(covariance_matrix)

        rotation = V_transpose.transpose().dot(U.transpose())
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