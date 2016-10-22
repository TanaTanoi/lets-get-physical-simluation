import numpy as np
import numpy.linalg as linalg
import face
import constraint
import noise
import math

np.set_printoptions(linewidth=2000)

default_flag_type = "spring"
class Model:
    drag = 0.95
    flag_type = default_flag_type # spring" or "exp_tri" or "imp_tri"
    def __init__(self, verts, faces, neighbours=[], uvs = [], constraints=[], flag_type=default_flag_type):
        self.flag_type = flag_type
        self.n = len(verts)
        self.verts = verts
        self.rendering_verts = np.copy(verts)
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
        self.stepsize = 0.3
        self.velocities = np.zeros((self.n, 3))
        self.mass_matrix = np.identity(self.n)
        self.constraints = constraints
        self.fixed_points = []
        for con in self.constraints:
            if con.type() == "FIXED":
                self.fixed_points.append(con)
        if self.flag_type == "imp_tri":
            self.mass_matrix /= (len(self.faces))
            self.global_matrix = self.calculate_triangle_global_matrix()
        self.count = 0
        self.wind_magnitude = 5

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
        if self.flag_type == "spring":
            self.simulate_spring()
        elif self.flag_type == "imp_tri":
            self.simulate_triangle()
        elif self.flag_type == "exp_tri":
            self.simulate_triangle_explicit()

    def simulate_triangle_explicit(self):
        forces = self.wind_forces(self.count)
        forces[:, 1] = -10
        s_n = self.rendering_verts
        for face in self.faces:
            res_tri_center = face.center_of_triangle(self.verts)
            def_tri_center = face.center_of_triangle(s_n)
            for vert_id in face.vertex_ids():
                T = self.potential_for_triangle(face, s_n, vert_id)
                x = self.verts[vert_id] - res_tri_center # bring to origin
                x = T.dot(x) # apply rotation
                x = x + def_tri_center # reset to original spot
                y = (x - s_n[vert_id])
                forces[vert_id] = forces[vert_id] + y

        acc = (self.stepsize * self.stepsize) * linalg.inv(self.mass_matrix).dot(forces)
        dist = self.velocities * self.stepsize
        s_n = self.rendering_verts + dist + acc

        for con_i in range(len(self.fixed_points)):
            con = self.fixed_points[con_i]
            s_n[con.vert_a] = self.verts[con.vert_a]
        self.rendering_verts = s_n

    def simulate_triangle(self):
        forces = self.wind_forces(self.count)
        forces[:, 1] = -1

        acc = (self.stepsize * self.stepsize) *  linalg.inv(self.mass_matrix).dot(forces)
        dist = self.velocities * self.stepsize
        s_n = self.rendering_verts + dist + acc
        q_n_1 = np.copy(s_n)
        b_array = np.zeros((self.n + len(self.fixed_points), 3))
        M = self.mass_matrix / (self.stepsize * self.stepsize)

        for _ in range(1):
            b_array[:self.n] = M.dot(s_n)
            for face in self.faces:
                f_verts = face.vertex_ids()
                for i in range(3):
                    v1 = f_verts[i]
                    v2 = f_verts[(i + 1) % 3]
                    T = self.potential_for_triangle(face, q_n_1, v2)
                    edge = self.verts[v2] - self.verts[v1]
                    g = T.dot(edge)
                    b_array[v1] = b_array[v1] - g
                    b_array[v2] = b_array[v2] + g

        # Assign fixed points
            for con_i in range(len(self.fixed_points)):
                con = self.fixed_points[con_i]
                b_array[-(con_i + 1)] = self.verts[con.vert_a]

            q_n_1 = np.linalg.solve(self.global_matrix, b_array)
            q_n_1 = q_n_1[:-len(self.fixed_points), :] # Don't grab the unwanted fixed points
        self.velocities = ((q_n_1 - self.rendering_verts) * 0.9) / self.stepsize

        self.rendering_verts = q_n_1

    def simulate_spring(self):
        forces = self.wind_forces(self.count)
        forces[:, 1] = -5

        for con in self.constraints:
            con.calculateRHS(self.rendering_verts, forces)
        for con_i in range(len(self.fixed_points)):
            con = self.fixed_points[con_i]
            forces[con.vert_a] *= 0
        acc = (self.stepsize * self.stepsize) *  linalg.inv(self.mass_matrix).dot(forces)
        dist = self.velocities * self.stepsize
        q_n_1 = self.rendering_verts + dist + acc

        self.velocities = (q_n_1 - self.rendering_verts) * self.drag / self.stepsize

        self.rendering_verts = q_n_1

    def calculate_b_for_triangle(self, i, s_n):
        b = np.zeros((1, 3))
        for face in self.verts_to_tri[i]:
            T = self.potential_for_triangle(face, s_n, i)
            for o_v in face.other_points(i):
                b += T.dot(self.verts[i] - self.verts[o_v])
        return b

    def wind_forces(self, time):
        time /= 500
        forces = np.zeros(((self.n, 3)))
        angle = noise.pnoise1(time) * math.pi * 0.5
        forces[:, 0] = math.cos(angle)
        forces[:, 2] = math.sin(angle)
        # Forces * max magnitude * perlins random * randomness influence between 0.5 and 1
        return forces * self.wind_magnitude * (noise.pnoise1(time)  + 0.2) * (noise.pnoise1(noise.pnoise1(time)) * 0.5 + 0.5)

    def potential_for_triangle(self, face, prime_verts, point):
        other_points = face.other_points(point)
        v1 = self.verts[point]
        v2 = self.verts[other_points[0]]
        v3 = self.verts[other_points[1]]
        x_g = np.matrix((v3 - v1, v2 - v1, (0,0,0))).T

        v1 = prime_verts[point]
        v2 = prime_verts[other_points[0]]
        v3 = prime_verts[other_points[1]]
        x_f = np.matrix((v3 - v1, v2 - v1, (0,0,0))).T

        combined = x_f.dot(np.linalg.pinv(x_g))
        return self.clamped_svd_for_matrix(combined)

    def clamped_svd_for_matrix(self, matrix):
        U, s, V_t = np.linalg.svd(matrix)
        s = np.diag(np.clip(s, 0, 1.1))
        return np.around(U.dot(s).dot(V_t), 11)

    def calculate_triangle_global_matrix(self):
        fixed_point_num = len(self.fixed_points)
        M = np.zeros((self.n + fixed_point_num, self.n + fixed_point_num))
        M[:self.n, :self.n] = self.mass_matrix
        M /= (self.stepsize * self.stepsize)

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
            M[-(i + 1), -(i + 1)] = 0
        return x + M

    def generate_plane(width, height, MAX_WIDTH_SIZE=500, MAX_HEIGHT_SIZE=300, flag_type=default_flag_type):

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
        return Model(verts, faces, neighbours, uvs, constraints=constraints, flag_type=flag_type)

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