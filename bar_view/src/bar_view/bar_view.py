
from rqt_gui_py.plugin import Plugin

from .bar_view_widget import BarViewWidget


class BarView(Plugin):

	def __init__(self, context):
		super(BarView, self).__init__(context)
		self.setObjectName('BarView')

		self._widget = BarViewWidget(self)
		if context.serial_number() > 1:
			self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
		context.add_widget(self._widget)

	def save_settings(self, plugin_settings, instance_settings):
		self._widget.save_settings(plugin_settings, instance_settings)

	def restore_settings(self, plugin_settings, instance_settings):
		self._widget.restore_settings(plugin_settings, instance_settings)

	def shutdown_plugin(self):
		self._widget.shutdown_plugin()
