import OpenGL
import numpy as np

OpenGL.ERROR_CHECKING = True
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *


def map_texture_on_sphere(texture, sphere_radius, h_division, v_division):
    r = float(sphere_radius)
    h_div = float(h_division)
    v_div = float(v_division)

    def draw_point(_s, _t, _r, _theta, _phi):
        x = _r * np.sin(_theta) * np.cos(_phi)
        y = _r * np.sin(_theta) * np.sin(_phi)
        z = _r * np.cos(_theta)
        glTexCoord2f(_s / h_div, _t / v_div)
        glVertex3f(x, y, z)

    # glPolygonMode(GL_FRONT, GL_FILL)
    glPolygonMode(GL_BACK, GL_FILL)

    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)

    glEnable(GL_TEXTURE_2D)
    glBindTexture(GL_TEXTURE_2D, texture)

    glBegin(GL_QUAD_STRIP)
    for t in range(0, int(v_div) + 1):
        for s in range(0, int(h_div) + 1):
            phi = (s / h_div) * 2 * np.pi
            theta = (t / v_div) * np.pi
            draw_point(s, t, r, theta, phi)
            theta = ((t + 1) / v_div) * np.pi
            draw_point(s, t, r, theta, phi)
    glEnd()
    glDisable(GL_TEXTURE_2D)
    # glBindTexture(GL_TEXTURE_2D, 0)

# http://www.cask.cc/wp/archives/51
def map_texture_on_sphere2(texture, sphere_radius, h_division, v_division):
    r = float(sphere_radius)
    h_div = int(h_division)
    v_div = int(v_division)

    glBindTexture(GL_TEXTURE_2D, texture)

    _sphere = gluNewQuadric()
    gluQuadricDrawStyle(_sphere, GLU_FILL)
    gluQuadricNormals(_sphere, GLU_SMOOTH)
    gluQuadricOrientation(_sphere, GLU_INSIDE)
    gluQuadricTexture(_sphere, GL_TRUE)

    glEnable(GL_TEXTURE_2D)
    # http://seesaawiki.jp/w/mikk_ni3_92/d/GLU%A4%CB%A4%E8%A4%EB%CE%A9%C2%CE%C9%BD%BC%A8
    gluSphere(_sphere, r, h_div, v_div)

    glDisable(GL_TEXTURE_2D)
    # glBindTexture(GL_TEXTURE_2D, 0)


def map_texture_on_plain(texture, tex_width, tex_height):
    glEnable(GL_TEXTURE_2D)
    glBindTexture(GL_TEXTURE_2D, texture)

    glBegin(GL_QUAD_STRIP)
    for s in [0.0, 1.0]:
        for t in [0.0, 1.0]:
            glTexCoord2f(s, t)
            glVertex3f(0.0, (s - 0.5) * tex_width, (t - 0.5) * tex_height)
    glEnd()
    glDisable(GL_TEXTURE_2D)
    glBindTexture(GL_TEXTURE_2D, 0)


def draw_cylinder(radius, div, width):
    dth = 2 * np.pi / div
    glBegin(GL_LINE_LOOP)
    for th in np.arange(0, 2 * np.pi, dth):
        x = radius * np.cos(th)
        z = radius * np.sin(th)
        glVertex3f(x, 0, z)
    glEnd()

    glBegin(GL_LINE_LOOP)
    for th in np.arange(0, 2 * np.pi, dth):
        x = radius * np.cos(th)
        z = radius * np.sin(th)
        glVertex3f(x, width, z)
    glEnd()

    glBegin(GL_LINES)
    for th in np.arange(0, 2 * np.pi, dth):
        x = radius * np.cos(th)
        z = radius * np.sin(th)
        glVertex3f(x, 0, z)
        glVertex3f(x, width, z)
    glEnd()


def draw_cylinderSurface(radius, div, width):
    dth = 2 * np.pi / div
    glBegin(GL_TRIANGLE_STRIP)
    for th in np.arange(0, 2 * np.pi, dth):
        x = radius * np.cos(th)
        z = radius * np.sin(th)
        x2 = radius * np.cos(th + dth)
        z2 = radius * np.sin(th + dth)
        glVertex3f(x, 0.0, z)
        glVertex3f(x2, 0.0, z2)
        glVertex3f(0.0, 0.0, 0.0)
    glEnd()

    glBegin(GL_TRIANGLE_STRIP)
    for th in np.arange(0, 2 * np.pi, dth):
        x = radius * np.cos(th)
        z = radius * np.sin(th)
        x2 = radius * np.cos(th + dth)
        z2 = radius * np.sin(th + dth)
        glVertex3f(x, width, z)
        glVertex3f(x2, width, z2)
        glVertex3f(0.0, width, 0.0)
    glEnd()

    glBegin(GL_QUADS)
    for th in np.arange(0, 2 * np.pi, dth):
        x = radius * np.cos(th)
        z = radius * np.sin(th)
        x2 = radius * np.cos(th + dth)
        z2 = radius * np.sin(th + dth)
        glVertex3f(x, 0, z)
        glVertex3f(x, width, z)
        glVertex3f(x2, width, z2)
        glVertex3f(x2, 0, z2)
    glEnd()


def draw_sphere(sphereSize):
    glutSolidSphere(sphereSize, 10, 5)


def draw_circle(radius, div):
    dth = 2 * np.pi / div
    glBegin(GL_POLYGON)
    for th in np.arange(0, 2 * np.pi, dth):
        x = radius * np.cos(th)
        y = radius * np.sin(th)
        glVertex3f(x, y, 0.0)
    glEnd()


def draw_circle_edge(radius, div):
    dth = 2 * np.pi / div
    glBegin(GL_LINE_LOOP)
    for th in np.arange(0.0, 2.0 * np.pi, dth):
        x = radius * np.cos(th)
        y = radius * np.sin(th)
        glVertex3f(x, y, 0.0)
    glEnd()


def draw_axis():
    glColor4d(1.0, 0.0, 0.0, 1.0)
    glBegin(GL_LINES)
    glVertex3d(0.0, 0.0, 0.0)
    glVertex3d(1000.0, 0.0, 0.0)
    glEnd()

    glColor4d(0.0, 1.0, 0.0, 1.0)
    glBegin(GL_LINES)
    glVertex3d(0.0, 0.0, 0.0)
    glVertex3d(0.0, 1000.0, 0.0)
    glEnd()

    glColor4d(0.0, 0.0, 1.0, 1.0)
    glBegin(GL_LINES)
    glVertex3d(0.0, 0.0, 0.0)
    glVertex3d(0.0, 0.0, 1000.0)
    glEnd()


def draw_grand(width, depth, interval):
    glBegin(GL_LINES)
    for i in np.arange(-width / 2.0, width / 2.0, interval):
        glVertex3d(i, -depth / 2.0, 0.0)
        glVertex3d(i, depth / 2.0, 0.0)

    for i in np.arange(-depth / 2.0, depth / 2.0, interval):
        glVertex3d(-width / 2.0, i, 0.0)
        glVertex3d(width / 2.0, i, 0.0)
    glEnd()


def draw_grand_gradation(width, depth, interval, power):
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    glBegin(GL_LINES)
    for i in np.arange(-depth / 2.0, depth / 2.0, interval):
        for j in np.arange(-width / 2.0, width / 2.0, interval):
            glColor4f(1.0, 1.0, 1.0, 1.0 - pow(abs(i / (width / 2.0)), power) - pow(abs(j / (width / 2.0)), power))
            glVertex3d(j, i, 0.0)
            glVertex3d(j + interval, i, 0.0)

    for i in np.arange(-width / 2.0, width / 2.0, interval):
        for j in np.arange(-depth / 2.0, depth / 2.0, interval):
            glColor4f(1.0, 1.0, 1.0, 1.0 - pow(abs(i / (depth / 2.0)), power) - pow(abs(j / (depth / 2.0)), power))
            glVertex3d(i, j, 0.0)
            glVertex3d(i, j + interval, 0.0)
    glEnd()
    glDisable(GL_BLEND)


def draw_grand_radial(radius, degree):
    glBegin(GL_LINES)
    for theta in np.arange(0.0, 2.0 * np.pi, np.radians(degree)):
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0 + radius * np.cos(theta), 0.0 + radius * np.sin(theta), 0.0)
    glEnd()


def draw_grand_equal_distance(radius, interval):
    delta = np.radians(5)
    for r in np.arange(interval, radius, interval):
        glBegin(GL_LINE_LOOP)
        for theta in np.arange(0.0, 2 * np.pi, delta):
            glVertex3f(0.0 + r * np.cos(theta), 0.0 + r * np.sin(theta), 0.0)
    glEnd()


def draw_line(startPoint, endPoint):
    glBegin(GL_LINES)
    glVertex3f(startPoint[0], startPoint[1], startPoint[2])
    glVertex3f(endPoint[0], endPoint[1], endPoint[2])
    glEnd()


def draw_eye(width, height, h_gakaku):
    aspect = width / height
    distance = width / np.tan(np.radiuns(h_gakaku))
    y = 0.1
    z = y / aspect

    glLineWidth(3.0)
    glColor3f(1.0, 0.0, 0.0)
    glPointSize(5)
    glBegin(GL_POINTS)
    glVertex3d(0.0, 0.0, 0.0)
    glEnd()
    glBegin(GL_LINES)
    glVertex3d(0.0, 0.0, 0.0)
    glVertex3d(distance, y, -z)

    glVertex3d(0.0, 0.0, 0.0)
    glVertex3d(distance, y, z)

    glVertex3d(0.0, 0.0, 0.0)
    glVertex3d(distance, -y, z)

    glVertex3d(0.0, 0.0, 0.0)
    glVertex3d(distance, -y, -z)
    glVertex3d(distance, y, -z)
    glVertex3d(distance, y, z)

    glVertex3d(distance, y, z)
    glVertex3d(distance, -y, z)

    glVertex3d(distance, -y, z)
    glVertex3d(distance, -y, -z)

    glVertex3d(distance, -y, -z)
    glVertex3d(distance, y, -z)
    glEnd()


def draw_layer_on_screen(width, height, alpha):
    glLoadIdentity()
    glMatrixMode(GL_PROJECTION)

    glLoadIdentity()
    gluOrtho2D(0, width, 0, height)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glColor4d(1.0, 0.0, 0.0, alpha)
    glBegin(GL_QUADS)
    glVertex3d(0.0, 0.0, 0.0)
    glVertex3d(width, 0.0, 0.0)
    glVertex3d(width, height, 0)
    glVertex3d(0.0, height, 0)
    glEnd()
    glDisable(GL_BLEND)

    glMatrixMode(GL_MODELVIEW)


def draw_colored_box():
    glBegin(GL_QUADS)  # Start Drawing The Box
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 1.0, -1.0)  # Top Right Of The Quad (Top)
    glVertex3f(-1.0, 1.0, -1.0)  # Top Left Of The Quad (Top)
    glVertex3f(-1.0, 1.0, 1.0)  # Bottom Left Of The Quad (Top)
    glVertex3f(1.0, 1.0, 1.0)  # Bottom Right Of The Quad (Top)

    glColor3f(0.5, 1.0, 0.5)
    glVertex3f(1.0, -1.0, 1.0)  # Top Right Of The Quad (Bottom)
    glVertex3f(-1.0, -1.0, 1.0)  # Top Left Of The Quad (Bottom)
    glVertex3f(-1.0, -1.0, -1.0)  # Bottom Left Of The Quad (Bottom)
    glVertex3f(1.0, -1.0, -1.0)  # Bottom Right Of The Quad (Bottom)

    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(1.0, 1.0, 1.0)  # Top Right Of The Quad (Front)
    glVertex3f(-1.0, 1.0, 1.0)  # Top Left Of The Quad (Front)
    glVertex3f(-1.0, -1.0, 1.0)  # Bottom Left Of The Quad (Front)
    glVertex3f(1.0, -1.0, 1.0)  # Bottom Right Of The Quad (Front)

    glColor3f(0.5, 0.5, 1.0)
    glVertex3f(1.0, -1.0, -1.0)  # Bottom Left Of The Quad (Back)
    glVertex3f(-1.0, -1.0, -1.0)  # Bottom Right Of The Quad (Back)
    glVertex3f(-1.0, 1.0, -1.0)  # Top Right Of The Quad (Back)
    glVertex3f(1.0, 1.0, -1.0)  # Top Left Of The Quad (Back)

    glColor3f(1.0, 0.5, 0.5)
    glVertex3f(-1.0, 1.0, 1.0)  # Top Right Of The Quad (Left)
    glVertex3f(-1.0, 1.0, -1.0)  # Top Left Of The Quad (Left)
    glVertex3f(-1.0, -1.0, -1.0)  # Bottom Left Of The Quad (Left)
    glVertex3f(-1.0, -1.0, 1.0)  # Bottom Right Of The Quad (Left)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 1.0, -1.0)  # Top Right Of The Quad (Right)
    glVertex3f(1.0, 1.0, 1.0)  # Top Left Of The Quad (Right)
    glVertex3f(1.0, -1.0, 1.0)  # Bottom Left Of The Quad (Right)
    glVertex3f(1.0, -1.0, -1.0)  # Bottom Right Of The Quad (Right)
    glEnd()  # Done Drawing The Quad


