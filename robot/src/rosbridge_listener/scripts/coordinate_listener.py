#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray

output_file = "write.txt"

def callback(data):
    coordinates = data.data
    rospy.loginfo("Received coordinates: %s" % str(coordinates))
    with open(output_file, 'w') as file:
        file.write("Received coordinates: %s\n" % str(coordinates))

def listener():
    rospy.init_node('coordinates_listener_computer2', anonymous=True)
    rospy.Subscriber("/coordinates", Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
