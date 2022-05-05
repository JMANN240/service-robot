#!/user/bin/env python

import random

import rospy
from std_msgs.msg import Int64

NUM_OF_CUSTS = 1
NUM_OF_CHEFS = 2
CUSTOMERS = []
for i in range(NUM_OF_CUSTS):
    CUSTOMERS.append(rospy.Publisher('customer_'+str(i+1), Int64, queue_size=10))


def getOrder(data: Int64):
    #print(data.data)
    #print(CUSTOMERS[data.data-1])
    CUSTOMERS[data.data-1].publish(random.randint(1,NUM_OF_CHEFS))





def listener():
    rospy.init_node('service_bot_customers', anonymous=True)

    rospy.Subscriber('bot_at_customer', Int64, getOrder)

    rospy.spin()


if __name__ == "__main__":
    listener()
