from __future__ import (print_function, absolute_import, division, unicode_literals)

import os
import rospkg
import rospy
import rosnode
from rospkg import RosPack
from python_qt_binding import loadUi
from PyQt5 import QtCore, QtGui, QtWidgets
from .rover_map_gui_widget import RoverMapGuiWidget
from qt_gui.plugin import Plugin
import subprocess

class RoverMapGuiPlugin(Plugin):

    def __init__(self, context):
        super(RoverMapGuiPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('RoverMapGui')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true", dest="quiet", help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = RoverMapGuiWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    ####### Tutorial functions #########
    def shutdown_plugin(self):
        rospy.signal_shutdown('Shutting down')

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
