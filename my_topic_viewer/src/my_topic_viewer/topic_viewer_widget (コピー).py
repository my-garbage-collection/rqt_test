#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import os
import sys
import rospkg

from python_qt_binding.QtCore import Qt, QTimer, qWarning
from python_qt_binding.QtGui import QWidget, QVBoxLayout, QLabel, QLineEdit, QFormLayout, QGroupBox

import rospy
from rostopic import get_topic_class
from std_msgs.msg import Float32



class DataDialog(QWidget):

	TOPIC_NAME = 'chatter'
	MSG_CLASS = Float32

	def __init__(self):
		super(DataDialog, self).__init__()
		rp = rospkg.RosPack()
		uiFile = os.path.join(rp.get_path('my_topic_viewer'), 'resource', 'TopicViewer.ui')
		loadUi(uiFile, self)

		self._displayedData = None
		self._displayedName = None
		self._groupBox = QGroupBox()
		self._layout = QFormLayout()
		self._mainLayout = QVBoxLayout();
		self._mainLayout.addWidget(self._groupBox)

#		self.setLayout(self._mainLayout);

	def setDisplayedData(self, data, name):
		if isinstance(data, dict):
			self.displayData(data)
		else:
			dictData = {}
			if name is None:
				dictData['unknown'] = data
			elif isinstance(data, list):
				dictData = dict(zip(name, data))
			else:
				dictData[name] = data
			self.displayData(dictData)

	def displayData(self, dictData):
		for key in dictData.keys():
			self._layout.addRow(QLabel(key+': '), QLineEdit(str(dictData[key])))
#		self._groupBox.setLayout(self._layout)

class TopicDataWidget(DataDialog):

	def __init__(self, plugin):
		super(TopicDataWidget, self).__init__()
		rospkgPack = rospkg.RosPack()
		self._plugin = plugin
		self._topicName = TOPIC_NAME
		self._subscriber = None

		self._updateTimer = QTimer(self)
		self._updateTimer.timeout.connect(self.timeoutCallback)
		self._updateTimer.start(40)
		self.subscribeTopic(self._topicName)


	# rqt override
	def save_settings(self, plugin_settings, instance_settings):
		instance_settings.set_value('topic_name', self._topicName)

	def restore_settings(self, plugin_settings, instance_settings):
		topicName = instance_settings.value('topic_name')
		try:
			self._topicName = eval(topicName)
		except Exception:
			topicName = None

	def shutdown_plugin(self):
		self.unregister_topic()

	# subscribe topic
	def subscribeTopic(self, topicName):
#		msgClass, self._topicName, _ = get_topic_class(topicName)
		self._subscriber = rospy.Subscriber(TOPIC_NAME, MSG_CLASS, self.messageCallback)

	def messageCallback(self, msg):
		self.setDisplayedData(msg.data, TOPIC_NAME)

	def unregisterTopic(self):
		if self._subscriber:
			self._subscriber.unregister()

	def timeoutCallback(self):
		pass
#		print 'time out'


