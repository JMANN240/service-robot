import os
import rospy
import rospkg
import tf
import math

from std_msgs.msg import Int64
from geometry_msgs.msg import PoseStamped

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow

def makePoseStamped(x, y, z, z_rot, frame_id):
    pos = PoseStamped()

    pos.header.frame_id = frame_id

    pos.pose.position.x = x
    pos.pose.position.y = y
    pos.pose.position.z = z

    quat = tf.transformations.quaternion_from_euler(0, 0, math.radians(z_rot))
    pos.pose.orientation.x = quat[0]
    pos.pose.orientation.y = quat[1]
    pos.pose.orientation.z = quat[2]
    pos.pose.orientation.w = quat[3]

    return pos

top_left_pose = makePoseStamped(-1, 4, 0, 225, 'map')
bottom_left_pose = makePoseStamped(-4, 4, 0, 315, 'map')
top_right_pose = makePoseStamped(-1, 1, 0, 135, 'map')
bottom_right_pose = makePoseStamped(-4, 1, 0, 45, 'map')

def tlCallback():
    goal_publisher.publish(top_left_pose)

def blCallback():
    goal_publisher.publish(bottom_left_pose)

def trCallback():
    goal_publisher.publish(top_right_pose)

def brCallback():
    goal_publisher.publish(bottom_right_pose)

goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

class MenuPlugin(Plugin):

    def __init__(self, context):
        super(MenuPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MenuPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QMainWindow()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('service_robot'), 'resource', 'corners.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MenuPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.

        self._widget.top_left_button.clicked.connect(tlCallback)
        self._widget.bottom_left_button.clicked.connect(blCallback)
        self._widget.top_right_button.clicked.connect(trCallback)
        self._widget.bottom_right_button.clicked.connect(brCallback)
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
