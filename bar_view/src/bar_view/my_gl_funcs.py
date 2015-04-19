
import math
import OpenGL
OpenGL.ERROR_CHECKING = True
from OpenGL.GL import*
from OpenGL.GLUT import*


def drawCylinder(radius, div, width):
	dth = 2*math.pi/div
	glBegin(GL_LINE_LOOP)
	for th in range(0, 2*math.pi, dth):
		x = radius * cos(th)
		z = radius * sin(th)
		glVertex3f(x, 0, z)
	glEnd()

	glBegin(GL_LINE_LOOP)
	for th in range(0, 2*math.pi, dth):
		x = radius * cos(th)
		z = radius * sin(th)
		glVertex3f(x, width, z)
	glEnd()

	glBegin(GL_LINES)
	for th in range(0, 2*math.pi, dth):
		x = radius * cos(th)
		z = radius * sin(th)
		glVertex3f(x, 0, z)
		glVertex3f(x, width, z)
	glEnd()


def drawCylinderSurface(radius, div, width):
	dth = 2*math.pi/div
	glBegin(GL_TRIANGLE_STRIP)
	for th in range(0, 2*math.pi, dth):
		x = radius * cos(th)
		z = radius * sin(th)
		x2 = radius * cos(th + dth)
		z2 = radius * sin(th + dth)
		glVertex3f(x, 0.0, z)
		glVertex3f(x2, 0.0, z2)
		glVertex3f(0.0, 0.0, 0.0)
	glEnd()

	glBegin(GL_TRIANGLE_STRIP)
	for th in range(0, 2*math.pi, dth):
		x = radius * cos(th)
		z = radius * sin(th)
		x2 = radius * cos(th + dth)
		z2 = radius * sin(th + dth)
		glVertex3f(x, width, z)
		glVertex3f(x2, width, z2)
		glVertex3f(0.0, width, 0.0)
	glEnd()


	glBegin(GL_QUADS)
	for th in range(0, 2*math.pi, dth):
		x = radius * cos(th)
		z = radius * sin(th)
		x2 = radius * cos(th + dth)
		z2 = radius * sin(th + dth)
		glVertex3f(x, 0, z)
		glVertex3f(x, width, z)
		glVertex3f(x2, width, z2)
		glVertex3f(x2, 0, z2)
	glEnd()

def drawSphere(sphereSize):
	glutSolidSphere(sphereSize, 10, 5)


def drawCircle (radius, div):
	dth = 2*math.pi/div
	glBegin(GL_POLYGON)
	for th in range(0, 2*math.pi, dth):
		x = radius * cos(th)
		y = radius * sin(th)
		glVertex3f(x, y, 0.0)
	glEnd()


def drawCircleEdge (radius, div):
	dth = 2*math.pi/div
	glBegin(GL_LINE_LOOP)
	for th in range(0.0, 2.0*math.pi, dth):
		x = radius * cos(th)
		y = radius * sin(th)
		glVertex3f(x, y, 0.0)
	glEnd()



def drawGrandAxis ():

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



def drawGrand (width, depth, interval):
	glBegin(GL_LINES)
	for i in range(-width/2.0, width/2.0, interval):
		glVertex3d(i, -depth, 0.0)
		glVertex3d(i, depth, 0.0)

	for i in range(-depth/2.0, depth/2.0, interval):
		glVertex3d(-width, i, 0.0)
		glVertex3d(width, i, 0.0)
	glEnd()



def drawGrand2 (width, depth, interval, power):
	glEnable (GL_BLEND)
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

	glBegin(GL_LINES)
	for i in range(-depth/2.0, depth/2.0, interval):
		for j in range(-width/2.0, width/2.0, interval):
			glColor4f(1.0, 1.0, 1.0, 1.0 - pow(abs(i/(width/2.0)),power) - pow(abs(j/(width/2.0)),power) )
			glVertex3d(j, i, 0.0)
			glVertex3d(j+interval, i, 0.0 )

	for i in range(-width/2.0, width/2.0, interval):
		for j in range(-depth/2.0, depth/2.0, interval):
			glColor4f(1.0, 1.0, 1.0, 1.0-pow(abs(i/(depth/2.0)),power)-pow(abs(j/(depth/2.0)),power) )
			glVertex3d(i, j, 0.0 )
			glVertex3d(i, j+interval , 0.0 )
	glEnd()
	glDisable (GL_BLEND)

def drawGrandRadial (radius, degree):
	glBegin(GL_LINES)
	for theta in range(0.0, 2.0*math.pi, math.radians(degree)):
		glVertex3f(0.0 , 0.0 , 0.0)
		glVertex3f(0.0+radius*cos(theta) , 0.0+radius*sin(theta) , 0.0)
	glEnd()


def drawGrandEqualDistance (radius, interval):
	delta = math.radians(5)
	for r in range(interval, radius, interval):
		glBegin(GL_LINE_LOOP)
		for theta in range(0.0, 2*math.pi, delta):
			glVertex3f(0.0+r*cos(theta) , 0.0+r*sin(theta) , 0.0)
	glEnd()


def drawLine(startPoint, endPoint):
	glBegin(GL_LINES)
	glVertex3f(startPoint[0], startPoint[1], startPoint[2])
	glVertex3f(endPoint[0], endPoint[1], endPoint[2])
	glEnd()


def drawEye(width, height, h_gakaku):

	aspect = width / height
	distance = (width)/math.tan(math.radiuns(h_gakaku))
	y = 0.1
	z = y/aspect

	glPushMatrix()
	glLineWidth(3.0)
	glColor3f(1.0, 0.0, 0.0)
	glPointSize (5)
	glBegin (GL_POINTS)
	glVertex3d (0.0, 0.0, 0.0)
	glEnd ()
	glBegin (GL_LINES)
	glVertex3d (0.0, 0.0, 0.0)
	glVertex3d (distance, y, -z)

	glVertex3d (0.0, 0.0, 0.0)
	glVertex3d (distance, y, z)

	glVertex3d (0.0, 0.0, 0.0)
	glVertex3d (distance, -y, z)

	glVertex3d (0.0, 0.0, 0.0)
	glVertex3d (distance, -y, -z)
	glVertex3d (distance, y, -z)
	glVertex3d (distance, y, z)

	glVertex3d (distance, y, z)
	glVertex3d (distance, -y, z)

	glVertex3d (distance, -y, z)
	glVertex3d (distance, -y, -z)

	glVertex3d (distance, -y, -z)
	glVertex3d (distance, y, -z)
	glEnd ()

	glPopMatrix()



def drawLayerOnScreen(width, height, alpha):

	glPushMatrix()
	glLoadIdentity()
	glMatrixMode(GL_PROJECTION)

	glPushMatrix()
	glLoadIdentity()
	gluOrtho2D(0, width, 0, height)
	glEnable(GL_BLEND)
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
	glColor4d(1.0, 0.0, 0.0, alpha)
	glBegin (GL_QUADS)
	glVertex3d (0.0, 0.0, 0.0)
	glVertex3d (width, 0.0, 0.0)
	glVertex3d (width, height, 0)
	glVertex3d (0.0, height, 0)
	glEnd ()
	glDisable(GL_BLEND)
	glPopMatrix()

	glMatrixMode(GL_MODELVIEW)
	glPopMatrix()


