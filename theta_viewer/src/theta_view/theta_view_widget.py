from __future__ import division
import os
import numpy as np
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot
from python_qt_binding.QtGui import QAction, QMenu, QWidget, QImage, QColor, qBlue, qGreen, qRed

import rospy
from rostopic import get_topic_class
from tf.transformations import quaternion_matrix, quaternion_about_axis

from OpenGL.GL import *
from OpenGL.GLUT import *

from .gl_widget import GLWidget as MyGLWidget
import my_gl_funcs as my_glfuncs

## main class inherits from the ui window class
## hint: http://vivi.dyndns.org/tech/Qt/openGL.html


class Theta360ViewWidget(QWidget):
    def __init__(self, plugin):
        super(Theta360ViewWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('theta_viewer'), 'resource', 'Theta360ViewWidget.ui')
        loadUi(ui_file, self)
        self.plugin = plugin

        self.position = (0.0, 0.0, 0.0)
        self.orientation = quaternion_about_axis(0.0, (1.0, 0.0, 0.0))
        self.topicName = None
        self.subscriber = None

        # create GL view
        self._glview = MyGLWidget()
        self._glview.setAcceptDrops(True)

        # backup and replace original paint method
        # self.glView.paintGL is callbacked from QGLWidget
        self._glview.paintGL_original = self._glview.paintGL
        self._glview.paintGL = self.glview_paintGL

        # backup and replace original mouse release method
        self._glview.mouseReleaseEvent_original = self._glview.mouseReleaseEvent
        self._glview.mouseReleaseEvent = self.glview_mouseReleaseEvent

        # add GL view to widget layout
        self.layout().addWidget(self._glview)

        # init and start update timer with 40ms (25fps)
        # updateTimeout is called with interval time
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_timeout)
        self.update_timer.start(40)
        self.cnt = 0


    ## ==============================================
    ## rqt requires
    def save_settings(self, plugin_settings, instance_settings):
        view_matrix_string = repr(self._glview.get_view_matrix())
        instance_settings.set_value('view_matrix', view_matrix_string)

    def restore_settings(self, plugin_settings, instance_settings):
        view_matrix_string = instance_settings.value('view_matrix')
        try:
            view_matrix = eval(view_matrix_string)
        except Exception:
            view_matrix = None

        if view_matrix is not None:
            self._glview.set_view_matrix(view_matrix)
        else:
            self.set_default_view()

    def shutdown_plugin(self):
        self.unregister_topic()

    ## ==============================================
    ## QGLWidget requires
    def set_default_view(self):
        self._glview.makeCurrent()
        self._glview.reset_view()
        self._glview.rotate((0, 0, 1), 45)
        self._glview.rotate((1, 0, 0), -45)
        self._glview.translate((0, 0, -200))

    def update_timeout(self):
        print self.cnt
        self._glview.makeCurrent()
        self._glview.updateGL()
        glRotated(45 + self.cnt, 0, 0, 1)

    def glview_paintGL(self):
        self._glview.paintGL_original()

        self.draw_basic_objects()

        if self.cnt == 0:
            print 'DRAW!'
            self.qimage = QImage('/home/matsunolab/Pictures/testimage_big.jpg', 'JPEG')  # GL_TEXTURE_2D
            self.texture = self._glview.get_texture(self.qimage)

        my_glfuncs.map_texture_on_sphere2(self.texture, 1500, 30, 30)
        self.cnt += 1

    def glview_mouseReleaseEvent(self, event):
        if event.button() == Qt.RightButton:
            menu = QMenu(self._glview)
            action = QAction(self._glview.tr("Reset view"), self._glview)
            menu.addAction(action)
            action.triggered.connect(self.set_default_view)
            menu.exec_(self._glview.mapToGlobal(event.pos()))


    ## ==============================================
    ## ROS requires
    def message_callback(self, message):
        self.position = (message.position.x, message.position.y, message.position.z)
        self.orientation = (message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w)

    def unregister_topic(self):
        if self.subscriber:
            self.subscriber.unregister()

    def subscribe_topic(self, topicName):
        msgClass, self.topicName, _ = get_topic_class(topicName)
        self.subscriber = rospy.Subscriber(self.topicName, msgClass, self.message_callback)


    ## ==============================================
    ## QT requires(?)
    @Slot('QDragEnterEvent*')
    def dragEnterEvent(self, event):
        if not event.mimeData().hasText():
            if not hasattr(event.source(), 'selectedItems') or len(event.source().selectedItems()) == 0:
                qWarning(
                    'Plot.dragEnterEvent(): not hasattr(event.source(), selectedItems) or len(event.source().selectedItems()) == 0')
                return
            item = event.source().selectedItems()[0]
            ros_topic_name = item.data(0, Qt.UserRole)
            if ros_topic_name is None:
                qWarning('Plot.dragEnterEvent(): not hasattr(item, ros_topicName_)')
                return

        # TODO: do some checks for valid topic here
        event.acceptProposedAction()

    @Slot('QDropEvent*')
    def dropEvent(self, event):
        if event.mimeData().hasText():
            topic_name = str(event.mimeData().text())
        else:
            droped_item = event.source().selectedItems()[0]
            topic_name = str(droped_item.data(0, Qt.UserRole))

        self.unregister_topic()
        self.subscribe_topic(topic_name)


    ## ==============================================
    ## For drawing
    def draw_basic_objects(self):

        glLineWidth(5)
        my_glfuncs.draw_axis()

        glLineWidth(1)
        glColor4f(1.0, 1.0, 1.0, 1.0)
        my_glfuncs.draw_grand_gradation(200, 200, 10, 2)














