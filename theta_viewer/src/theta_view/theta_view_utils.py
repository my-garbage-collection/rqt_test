from OpenGL.GL import *
from OpenGL.GLUT import *


def drawTest(self):
    if self.qimage.isNull():
        print 'fail to grab image'
        self.include_image_from_local()
    else:
        print self.qimage.width()
        myGLFuncs.map_texture(self.texture, self.qimage.width() / 10.0, self.qimage.height() / 10.0)


def test(self, qimage):
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

    texture = self.bindTexture(qImage)
    width = qImage.width() / 1000000000
    height = qImage.height() / 1000000000

    myGLFuncs.map_texture(texture, width, height)  # http://stackoverflow.com/questions/5335218/using-qimage-with-opengl















