import pyglet
from pyglet.window import key
from pyglet.gl import *
import sys
import offfile
import face
import numpy as np
import math
import model
from PIL import Image

class Renderer:
    def __init__(self, filename=""):
        # m_ is the mouse prefix
        self.m_x = 0
        self.m_y = 0
        self.m_down = False
        # r_ is the rotations prefix
        self.r_x = 0
        self.z = 0
        self.x = 0
        if len(filename) > 0:
            values = offfile.OffFile(filename)
            values = values.read_file()
            self.model = model.Model(values[0], values[1])
            self.model.center()
        else:
            self.model = model.Model.generate_plane(5, 3)
            self.model.center()

    def main(self):
        self.window = pyglet.window.Window()
        self.initialize_window_callbacks()
        self.setup()
        # glClearColor(0.8,0.8,0.8,1.)
        # glShadeModel(GL_SMOOTH)
        # glEnable(GL_DEPTH_TEST)
        # glEnable(GL_LIGHTING)
        # lightZeroPosition = Renderer.opengl_array([0.,10.,0.,1.])
        # lightZeroColor = Renderer.opengl_array([0.8,1.0,0.8,1.0]) #green tinged
        # glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition)
        # glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor)
        # glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.1)
        # glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.05)
        # glEnable(GL_LIGHT0)
        # glMatrixMode(GL_PROJECTION)
        # gluPerspective(40.,1.,1.,40.)
        # gluLookAt(0,0,-10,
        #           0,0,0,
        #           0,1,0)

        # glMatrixMode(gl.GL_PROJECTION)
        # glLoadIdentity()
        # gluPerspective(60., self.window.width / self.window.height, 1, 100.)
        # glMatrixMode(GL_MODELVIEW)
        # glMatrixMode(GL_PROJECTION);
        # glLoadIdentity();
        # glOrtho(-8.0, 8.0, -8.0, 8.0, -8.0, 8.0);
        glPushMatrix()
        textures = []
        self.load_texture("doggerflag.png", textures)

        pyglet.app.run()
        return

    def initialize_window_callbacks(self):
        self.window.on_draw = self.on_draw
        self.window.on_key_press = self.on_key_press

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

    def on_draw(self):
        self.setup_camera()

        glTranslatef((self.window.width / 2), self.window.height / 2, -900)
        glRotatef(self.r_x * 50, 0, 1, 0)
        self.draw_object()

    def setup_camera(self):
        glMatrixMode(gl.GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(60, self.window.width / self.window.height, 0.1, 10000)
        glMatrixMode(gl.GL_MODELVIEW)
        glLoadIdentity()

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

    def draw_object(self):
        glClear(GL_COLOR_BUFFER_BIT)

        glBegin(GL_TRIANGLES)
        for face in self.model.faces:
            for v_id in face.vertex_ids():
                vert = self.model.verts[v_id]
                uv = self.model.uvs[v_id]
                glTexCoord2f(uv[0], uv[1])
                glVertex3f(vert[0], vert[1], vert[2])
        glEnd()

    def draw_lines(self):
        glBegin(GL_LINES)
        for x in range(0, self.window.width, 10):
            for y in range(0, self.window.height, 10):
                glVertex3f(x, y, -x / 100)
        glEnd()

    def on_key_press(self, symbol, modifiers):
        if(symbol == key.Q):
            self.r_x += 0.1
        elif(symbol == key.E):
            self.r_x -= 0.1
        elif(symbol == key.S):
            self.z -= 0.5
        elif(symbol == key.W):
            self.z += 0.5
        elif(symbol == key.A):
            self.x += 2
        elif(symbol == key.D):
            self.x -= 2
        elif(symbol == key.Z):
            self.r_x += 90
        elif(symbol == key.SPACE):
            self.model.simulate()

    def setup(self):
        glMatrixMode(gl.GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(60, self.window.width / self.window.height, 0.001, 100)
        glMatrixMode(gl.GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(-0.7, -0.3, -1.5)
        glEnable(GL_DEPTH_TEST)

    def draw_background(self):
        glMatrixMode(GL_PROJECTION)
        glClear(GL_DEPTH_BUFFER_BIT)
        glShadeModel(GL_FLAT)
        glDisable(GL_DEPTH_TEST)
        glPushMatrix()
        glLoadIdentity()
        w = self.window.width
        h = self.window.height
        gluOrtho2D(0, w, h, 0)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()
        color = Renderer.opengl_array([0.99,0.99,0.99,1.])
        glMaterialfv(GL_FRONT, GL_DIFFUSE, color)
        glMaterialfv(GL_FRONT, GL_AMBIENT, 1.)
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

    def load_texture(self, filename, textures):
        image = pyglet.image.load(filename)
        textures.append(image.get_texture())

        glEnable(textures[-1].target)
        glBindTexture(textures[-1].target, textures[-1].id)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.width, image.height,
        0, GL_RGBA, GL_UNSIGNED_BYTE,
        image.get_image_data().get_data('RGBA',
        image.width * 4))

    def opengl_array(array):
        return (GLfloat * len(array))(*array)

# if __name__ == '__main__': main()
r = Renderer()
r.main()