import numpy as np
import face

class OffFile:
    def __init__(self, filename):
        self.lines = open(filename, 'r').read().split("\n")
        #Remove the OFF in the first line
        if 'OFF' in self.lines[0]:
            self.lines.pop(0)

    # Grabs the next line, ignoring comments
    def nextLine(self):
        line = self.lines.pop(0)
        while(len(line) == 0 or line[0] == '#'):
            line = self.lines.pop(0)
        return line

    def read_file(self):

        first_line = self.nextLine().split()

        number_of_verticies =   int(first_line[0])
        number_of_faces =       int(first_line[1])
        number_of_edges =       int(first_line[2])

        n = number_of_verticies

        # Every vertex in the .off
        verts = []
        # Every face in the .off
        faces = []
        # The ID of the faces related to this vertx ID (i.e. vtf[i] contains faces that contain ID i)
        verts_to_face = []

        for i in range(n):
            vert_line = self.nextLine().split()
            x = float(vert_line[0])
            y = float(vert_line[1])
            z = float(vert_line[2])
            verts.append(np.array([x, y, z]))

            verts_to_face.append([])

        for i in range(number_of_faces):
            face_line = self.nextLine().split()
            v1_id = int(face_line[1])
            v2_id = int(face_line[2])
            v3_id = int(face_line[3])
            faces.append(face.Face(v1_id, v2_id, v3_id))
            # Add this face to each vertex face map
            verts_to_face[v1_id].append(i)
            verts_to_face[v2_id].append(i)
            verts_to_face[v3_id].append(i)
        print("Num of verts ", len(verts))
        print("Num of faces ", len(faces))
        return (verts, faces, verts_to_face)