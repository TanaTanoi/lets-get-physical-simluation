import numpy as np
import numpy.linalg as linalg
import face
import constraint
np.set_printoptions(linewidth=2000)
class Model:
    def __init__(self, verts, faces, uvs = [], constraints=[]):
        self.n = len(verts)
        self.verts = verts
        self.faces = faces
        self.uvs = uvs
        self.velocities = np.zeros((self.n, 3))
        self.mass_matrix = np.identity(self.n)
        self.constraints = constraints
        self.stepsize = 0.9
        self.global_matrix = self.calculate_global_matrix()
        self.count = 0
        print(self.global_matrix)

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
        self.count += 1
        forces = np.zeros(((self.n, 3)))
        forces[:, 1] =- 0.01
        if(self.count < 300):
            forces[:,2] = 0.20
        elif(self.count < 600):
            forces[:,2] = -0.20
        else:
            self.count = 0
        acc = (self.stepsize * self.stepsize) *  linalg.inv(self.mass_matrix).dot(forces)
        dist = self.velocities * self.stepsize
        s_n = self.verts + dist + acc
        q_n_1 = s_n
        M = self.mass_matrix / (self.stepsize * self.stepsize)

        for i in range(10):
            b_array = M.dot(q_n_1)
            for con in self.constraints:
                con.calculateRHS(s_n, b_array)
            q_n_1 = linalg.solve(self.global_matrix, b_array.flatten())
            q_n_1 = np.reshape(q_n_1, (self.n, 3))
        self.velocities = (q_n_1 - self.verts) / self.stepsize
        self.verts = np.reshape(q_n_1, (self.n, 3))

    def wind_forces(time):
        forces = np.zeros(((self.n, 3)))

    def calculate_global_matrix(self):
        print(self.n)
        M = np.identity(self.n * 3) / (self.stepsize * self.stepsize)
        sum_m = np.zeros((self.n * 3, self.n * 3))

        for con in self.constraints:
            S = con.S
            A = con.A
            x = S.T.dot(A.T.dot(A.dot(S)))
            sum_m += x

        return M + sum_m

    def generate_plane(width, height, MAX_WIDTH_SIZE=500, MAX_HEIGHT_SIZE=300):

        n = width * height
        width_gap = MAX_WIDTH_SIZE / width
        height_gap = -MAX_HEIGHT_SIZE / height

        verts = np.zeros((n, 3))
        faces = []
        constraints = []
        uvs = np.zeros((n, 2))
        for x in range(width):
            for y in range(height):
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
                faces.append(face.Face(v_1, v_2, v_3))
                Model.add_spring_constraint_set(verts, v_1, v_2, v_3, constraints)
            # points after the first line
            if v_id >= width:
                v_1 = v_id
                v_2 = v_id + 1
                v_3 = v_id - (width - 1)
                faces.append(face.Face(v_1, v_2, v_3))
            # the lines along the bottom
            if v_id >= n - width and v_id < n:
                Model.add_spring_constraint(verts, v_id, v_id + 1, constraints)
        # fix top and bottom left corners
        Model.add_fixed_constraint(n, 0, verts[0], constraints)
        bottom_left = width * (height - 1)
        Model.add_fixed_constraint(n, bottom_left, verts[bottom_left], constraints)
        return Model(verts, faces, uvs, constraints=constraints)

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