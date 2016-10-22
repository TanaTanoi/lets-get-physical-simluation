import pyglet
from pyglet.window import key
from pyglet.window import mouse
from pyglet.gl import *
import sys
import offfile
import face
import numpy as np
import math
import model
from PIL import Image

class Renderer:
    SAVE_TO_IMAGES = False
    DRAW_LINES = True
    def __init__(self, size=8, flags=["spring"]):
        # m_ is the mouse prefix
        self.m_x = 0
        self.m_y = 0
        self.m_down = False
        # r_ is the rotations prefix
        self.r_x = 0
        self.z = -1000
        self.y = 100
        self.x = 0
        self.models = []
        self.FPS = 25
        for flag_type in flags:
            self.models.append(model.Model.generate_plane(size, size, flag_type=flag_type))

    def main(self):
        self.window = pyglet.window.Window()
        self.initialize_window_callbacks()
        self.setup()
        glPushMatrix()
        textures = []
        self.load_texture("doggerflag.png", textures)

        pyglet.app.run()
        return

    def update(self, dt):
        for model in self.models:
            model.simulate()
        if self.SAVE_TO_IMAGES:

            a = [0] * (self.window.width * self.window.height * 3)
            a = (GLuint * len(a))(*a)
            glReadPixels(0, 0, self.window.width, self.window.height, GL_RGB, GL_UNSIGNED_BYTE, a)
            a = np.array(a)
            a = a.reshape((self.window.height, self.window.width, 3))
            image = Image.fromarray(a, "RGB")
            image = image.transpose(Image.FLIP_TOP_BOTTOM)
            image.save("/Users/tanatanoi/Documents/university/CGRA409/project/vid/" + str(self.models[0].count)+".jpg")

    def initialize_window_callbacks(self):
        self.window.on_draw = self.on_draw
        self.window.on_key_press = self.on_key_press
        self.window.on_mouse_drag = self.on_mouse_drag
        pyglet.clock.schedule_interval(self.update, 1 / self.FPS)

    def mouse_input(self, button, state, x, y):
        print("MOUSE ", button, " state: ", state, " x: ", x, " y: ", y)
        self.m_x = x
        self.m_y = y
        self.m_down = state == 0

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        if(buttons & mouse.LEFT):
            self.r_x += dx

    def on_mouse_motion(self, x, y, dx, dy):
        if(self.m_down):
            self.r_x += dx
            self.m_x = x

    def on_draw(self):
        self.setup_camera()
        glClear(GL_COLOR_BUFFER_BIT)
        glTranslatef(0, self.y, self.z)
        glRotatef(self.r_x, 0, 1, 0)
        glPushMatrix()
        glTranslatef(0, -75 * (len(self.models) - 1), 0)
        for model in self.models:
            self.draw_object(model)
            glTranslatef(0, 300, 0)
        glPopMatrix()

    def setup_camera(self):
        glMatrixMode(gl.GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(60, self.window.width / self.window.height, 0.1, 10000)
        glMatrixMode(gl.GL_MODELVIEW)
        glLoadIdentity()

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

    def draw_object(self, model):
        if self.DRAW_LINES:
            glLineWidth(1)
            glPolygonMode( GL_FRONT_AND_BACK, GL_LINE )
            glBegin(GL_TRIANGLES)
            for face in model.faces:
                for v_id in face.vertex_ids():
                    vert = model.rendering_verts[v_id]
                    glColor3f(1,0,0)
                    glVertex3f(vert[0], vert[1], vert[2])
            glEnd()
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        glBegin(GL_TRIANGLES)
        for face in model.faces:
            for v_id in face.vertex_ids():
                vert = model.rendering_verts[v_id]
                uv = model.uvs[v_id]
                glColor3f(1,1,1)
                glTexCoord2f(uv[0], uv[1])
                glVertex3f(vert[0], vert[1], vert[2])
        glEnd()


        glBegin(GL_LINES)
        glVertex3f(0,-300,0)
        vert = model.verts[0]
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
            self.r_x += 0.3
        elif(symbol == key.E):
            self.r_x -= 0.3
        elif(symbol == key.S):
            self.z -= 50
        elif(symbol == key.W):
            self.z += 50
        elif(symbol == key.A):
            self.x += 200
        elif(symbol == key.D):
            self.x -= 200
        elif(symbol == key.Z):
            self.r_x += 90
        elif(symbol == key.R):
            if(self.models[0].wind_magnitude == 0):
                for model in self.models:
                    model.wind_magnitude = 1
            for model in self.models:
                model.wind_magnitude *= 1.5
            print("Wind magnitude is ", self.models[0].wind_magnitude)
        elif(symbol == key.T):
            if(abs(self.models[0].wind_magnitude) < 1):
                for model in self.models:
                    model.wind_magnitude = 0
            for model in self.models:
                model.wind_magnitude /= 1.5
            print("Wind magnitude is ", self.models[0].wind_magnitude)
        elif(symbol == key.Y):
            for model in self.models:
                model.wind_magnitude *= -1
            print("Wind magnitude is ", self.models[0].wind_magnitude)
        elif(symbol == key.SPACE):
            for model in self.models:
                model.simulate()
        elif(symbol == key.F):
            self.DRAW_LINES = not self.DRAW_LINES
        elif(symbol == key.ESCAPE): # escape
            exit()

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

    def opengl_int_array(array):
        return (GLuint * len(array))(*array)

# if __name__ == '__main__': main()
size = 8
flags = ["spring"]
if len(sys.argv) > 1:
    size = int(sys.argv[1])
if len(sys.argv) > 2:
    flags = sys.argv[2:]
print(sys.argv[2:-1])
r = Renderer(size=size, flags=flags)
r.main()