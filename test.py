from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
from random import randint
import sys
import offfile
import face
import numpy as np
import math

name = 'ball_glut'
class Renderer:
    def __init__(self, filename):
        self.c = -10.
        self.z = 5.
        values = offfile.OffFile(filename)
        values = values.read_file()
        self.verts = values[0]
        self.faces = values[1]
        self.center_object()

    def main(self):
        # self.c = 0
        glutInit(sys.argv)
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
        glutInitWindowSize(400,400)
        glutCreateWindow(name)

        glClearColor(0.,0.,0.,1.)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_CULL_FACE)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        lightZeroPosition = [10.,4.,10.,1.]
        lightZeroColor = [0.8,1.0,0.8,1.0] #green tinged
        glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition)
        glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor)
        glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.1)
        glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.05)
        glEnable(GL_LIGHT0)
        glutDisplayFunc(self.display)
        glutKeyboardFunc(self.keyboard)
        glutIdleFunc(self.display)
        glMatrixMode(GL_PROJECTION)
        gluPerspective(40.,1.,1.,40.)
        glMatrixMode(GL_MODELVIEW)
        gluLookAt(0,0,10,
                  0,0,0,
                  0,1,0)
        glPushMatrix()
        glutMainLoop()
        return

    def keyboard(self, key, x, y):
        print("KEYBOARD ", key, " x: ",x, " y: ", y)
        char = key.decode("utf-8")
        # escape key
        if(char == '\x1b' or char == 'q'):
            exit()
        elif(char == 'w'):
            self.z -= 2
        elif(char == 's'):
            self.z += 2

    def display(self):
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        glPushMatrix()

        glTranslatef(0, 0, self.z)
        glRotatef(self.c * 1000, 0, 1, 0)
        glRotatef(90, -1, 0, 0)
        self.c += 0.001
        color = [1.0,0.,0.,1.]
        glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
        # glutSolidSphere(2,20,20)
        self.draw_object()
        glPopMatrix()
        glutSwapBuffers()
        return

    def draw_object(self):
        glBegin(GL_TRIANGLES)
        for face in self.faces:
            for v_id in face.vertex_ids():
                vert = self.verts[v_id]
                glVertex3f(vert[0], vert[1], vert[2])
        glEnd()

    def center_object(self):
        middle_point = np.array((0., 0., 0.))
        for vert in self.verts:
            middle_point += vert
        middle_point = middle_point / float(len(self.verts))
        middle_point *= -1
        print(middle_point)
        for vert_id in range(len(self.verts)):
            self.verts[vert_id] += middle_point

# if __name__ == '__main__': main()
r = Renderer("cactus_small.off")
r.main()