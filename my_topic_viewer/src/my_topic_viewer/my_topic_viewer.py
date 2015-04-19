#!/usr/bin/env python

from rqt_gui_py.plugin import Plugin
from .topic_viewer_widget import TopicDataWidget

class TopicDataViewer(Plugin):

	def __init__(self, context):
		super(TopicDataViewer, self).__init__(context)
		self.setObjectName('TopicDataViewer')
		self._widget = TopicDataWidget(self)
		self._widget.start()
		context.add_widget(self._widget)

	def save_settings(self, plugin_settings, instance_settings):
		self._widget.save_settings(plugin_settings, instance_settings)

	def restore_settings(self, plugin_settings, instance_settings):
		self._widget.restore_settings(plugin_settings, instance_settings)

	def shutdown_plugin(self):
		self._widget.shutdown_plugin()


