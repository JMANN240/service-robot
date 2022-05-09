import os
import rospy
import rospkg
import tf
import math

from std_msgs.msg import Int64
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalID

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow, QListWidgetItem

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

### CONFIGURATION ###

chef_foods = {
    'chef_a': ['pizza', 'spaghetti'],
    'chef_b': ['burger']
}

testing_location = 'MSB_104' # MSB_104 or GAZEBO_HOUSE

if testing_location == 'GAZEBO_HOUSE':
    poses = {
        'chef_a': makePoseStamped(-3.2, 4.7, 0, 90, 'map'),
        'chef_b': makePoseStamped(-1.2, 4.7, 0, 90, 'map'),
        'customer_a': makePoseStamped(-1, 1, 0, 270, 'map'),
        'customer_b': makePoseStamped(-2.5, 1, 0, 270, 'map'),
        'customer_c': makePoseStamped(-4, 1, 0, 270, 'map'),
        'robot_home': makePoseStamped(-5, 3.5, 0, 0, 'map')
    }
elif testing_location == 'MSB_104':
    poses = {
        'chef_a': makePoseStamped(-3.2, 4.7, 0, 90, 'map'),
        'chef_b': makePoseStamped(-1.2, 4.7, 0, 90, 'map'),
        'customer_a': makePoseStamped(-1, 1, 0, 270, 'map'),
        'customer_b': makePoseStamped(-2.5, 1, 0, 270, 'map'),
        'customer_c': makePoseStamped(-4, 1, 0, 270, 'map'),
        'robot_home': makePoseStamped(-5, 3.5, 0, 0, 'map')
    }

orders = []

location = 'robot_home'
last_move_status = None

def getChefByFood(food):
    for chef, foods in chef_foods.items():
        if food in foods:
            return chef
    return None

def updateOrderList(qtlist):
    global orders
    for order in orders:
        qtlist.takeItem(qtlist.row(order['list_item']))
        print("Removing list item")
    for order in orders:
        if order['status'] == 'delivered':
            continue
        order['list_item'] = QListWidgetItem(f"{order['customer']}, {order['food']}, {order['chef']}, {order['status']}")
        qtlist.addItem(order['list_item'])
        print("Adding list item")

def makeOrderCallback(qtlist, customer, food):
    def cb():
        global orders, location
        chef = getChefByFood(food)
        order = {
            'customer': customer, 
            'food': food,
            'chef': chef,
            'status': 'ordered',
            'list_item': QListWidgetItem(f"{customer}, {food}, {chef}, {'ordered'}")
        }
        orders.append(order)
        updateOrderList(qtlist)
        print(f"{order['customer']} made order of {order['food']} to {order['chef']}")
        location = chef
        goal_publisher.publish(poses[location])
        #self.last_id = status.status_list[0].goal_id
    return cb

def pickupOrdersFromChef(chef):
    global orders
    for order in orders:
        if order['chef'] == chef:
            order['status'] = 'picked up'
    print(f"Picked up orders from {chef}")

def deliverOrdersToCustomer(customer):
    global orders
    for order in orders:
        if order['customer'] == customer:
            order['status'] = 'delivered'
    print(f"Delivered orders to {customer}")

def getOrdersByStatus(status):
    global orders
    return [order for order in orders if order['status'] == status]

goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
cancel_publisher = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

class OrderPlugin(Plugin):

    def __init__(self, context):
        super(OrderPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('OrderPlugin')

        self.last_order = "GO!"
        self.last_id = None
        
        # Create QWidget
        self._widget = QMainWindow()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('service_robot'), 'resource', 'order.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('OrderPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.

        self._widget.customer_1_pizza_button.clicked.connect(makeOrderCallback(self._widget.orders_list, 'customer_a', "pizza"))
        self._widget.customer_1_burger_button.clicked.connect(makeOrderCallback(self._widget.orders_list, 'customer_a', "burger"))
        self._widget.customer_1_spaghetti_button.clicked.connect(makeOrderCallback(self._widget.orders_list, 'customer_a', "spaghetti"))
        self._widget.customer_2_pizza_button.clicked.connect(makeOrderCallback(self._widget.orders_list, 'customer_b', "pizza"))
        self._widget.customer_2_burger_button.clicked.connect(makeOrderCallback(self._widget.orders_list, 'customer_b', "burger"))
        self._widget.customer_2_spaghetti_button.clicked.connect(makeOrderCallback(self._widget.orders_list, 'customer_b', "spaghetti"))
        self._widget.customer_3_pizza_button.clicked.connect(makeOrderCallback(self._widget.orders_list, 'customer_c', "pizza"))
        self._widget.customer_3_burger_button.clicked.connect(makeOrderCallback(self._widget.orders_list, 'customer_c', "burger"))
        self._widget.customer_3_spaghetti_button.clicked.connect(makeOrderCallback(self._widget.orders_list, 'customer_c', "spaghetti"))

        rospy.Subscriber('/move_base/status', GoalStatusArray, self.makeGetStatusCallback())
        rospy.Subscriber('/stop_sign', String, self.makeHandleSigns())
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    def makeHandleSigns(self):
        def handle_signs(data):
            print(self.last_order, data.data)
            if data.data == "STOP!" and self.last_order != 'STOP':
                #print(self.last_id)
                cancel_publisher.publish(self.last_id)
            elif data.data == "GO!" and self.last_order != 'GO!':
                goal_publisher.publish(poses[location])
            self.last_order = data.data

        return handle_signs

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

    def makeGetStatusCallback(self):
        def getStatusCallback(status: GoalStatusArray):
            global orders, last_move_status, location

            # Guard clause: empty status list
            if len(status.status_list) == 0:
                return

            self.last_id = status.status_list[-1].goal_id

            # Get the new move status
            new_move_status = status.status_list[-1].status
            
            # Guard clause: didn't newly arrive somewhere
            if new_move_status != 3 or last_move_status == 3:
                last_move_status = new_move_status
                return

            print(f"Arrived at {location}!")

            if location == 'chef_a' or location == 'chef_b':
                pickupOrdersFromChef(location)
                updateOrderList(self._widget.orders_list)
            elif location == 'customer_a' or location == 'customer_b' or location == 'customer_c':
                deliverOrdersToCustomer(location)
                updateOrderList(self._widget.orders_list)

            print(f"Orders: {orders}")

            ordered_orders = getOrdersByStatus('ordered')
            picked_up_orders = getOrdersByStatus('picked up')
            
            if len(ordered_orders) == 0:
                if len(picked_up_orders) == 0:
                    location = 'robot_home'
                else:
                    location = picked_up_orders[0]['customer']
            else:
                location = ordered_orders[0]['chef']
            
            goal_publisher.publish(poses[location])
            #self.last_id = status.status_list[0].goal_id
            
            last_move_status = new_move_status

        return getStatusCallback

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
