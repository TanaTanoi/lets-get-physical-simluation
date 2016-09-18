from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
import sys
import offfile
import face
import numpy as np
import math
import model

name = 'ball_glut'
class Renderer:
    def __init__(self, filename):
        # m_ is the mouse prefix
        self.m_x = 0
        self.m_y = 0
        self.m_down = False
        # r_ is the rotations prefix
        self.r_x = 0
        self.z = 5.
        values = offfile.OffFile(filename)
        values = values.read_file()
        self.model = model.Model(values[0], values[1])
        self.model.center()

    def main(self):
        # self.c = 0
        glutInit(sys.argv)
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
        glutInitWindowSize(400,400)
        glutCreateWindow(name)

        glClearColor(0.8,0.8,0.8,1.)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_CULL_FACE)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        lightZeroPosition = [0.,10.,0.,1.]
        lightZeroColor = [0.8,1.0,0.8,1.0] #green tinged
        glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition)
        glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor)
        glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.1)
        glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.05)
        glEnable(GL_LIGHT0)
        glutDisplayFunc(self.display)
        glutKeyboardFunc(self.keyboard)
        glutMouseFunc(self.mouse_input)
        glutMotionFunc(self.mouse_motion)
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
            self.z -= 1
        elif(char == 's'):
            self.z += 1

    def mouse_input(self, button, state, x, y):
        print("MOUSE ", button, " state: ", state, " x: ", x, " y: ", y)
        self.m_x = x
        self.m_y = y
        self.m_down = state == 0

    def mouse_motion(self, x, y):
        print("DRAG x: ", x,  " y: ",y )
        if(self.m_down):
            self.r_x += x - self.m_x
            self.m_x = x

    def display(self):
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        self.draw_background()
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()

        glTranslatef(0, 0, self.z)
        glRotatef(self.r_x, 0, 1, 0)
        glRotatef(90, -1, 0, 0)
        color = [0.7,0.0,0.5,1.]
        glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
        self.draw_object()
        glPopMatrix()
        glutSwapBuffers()
        return

    def draw_object(self):
        glBegin(GL_TRIANGLES)
        for face in self.model.faces:
            for v_id in face.vertex_ids():
                vert = self.model.verts[v_id]
                glVertex3f(vert[0], vert[1], vert[2])
        glEnd()


    def draw_background(self):
        glMatrixMode(GL_PROJECTION)
        glClear(GL_DEPTH_BUFFER_BIT)
        glShadeModel(GL_FLAT)
        glDisable(GL_DEPTH_TEST)
        glPushMatrix()
        glLoadIdentity()
        w = glutGet(GLUT_WINDOW_WIDTH)
        h = glutGet(GLUT_WINDOW_HEIGHT)
        gluOrtho2D(0, w, h, 0)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()
        color = [0.99,0.99,0.99,1.]
        glMaterialfv(GL_FRONT, GL_DIFFUSE, color)
        glMaterialfv(GL_FRONT, GL_AMBIENT, 1)
        glBegin(GL_QUADS);
        glVertex2f(0, 0)
        glVertex2f(0, h / 2.)
        glVertex2f(w, h / 2.)
        glVertex2f(w, 0)
        glEnd();
        glPopMatrix()
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glEnable(GL_DEPTH_TEST)
# if __name__ == '__main__': main()
r = Renderer("data/cactus_small.off")
r.main()