#! /usr/bin/python
import rospy
from std_msgs.msg import Float64
import threading
from gazebo_msgs.srv import *
import geometry_msgs.msg
import tf2_ros.transform_broadcaster
import math  
import tf
from sensor_msgs.msg import  LaserScan

rospy.init_node('change', anonymous=True)
pub = rospy.Publisher("scan",LaserScan,queue_size=10)

def callBack(scan):
    res=scan
    res.time_increment=0.000001
    pub.publish(res)
    pass

def pubOdom():
    rospy.Subscriber("test",LaserScan,callBack)
    rospy.spin()
    pass
    
if __name__ == "__main__":
    pubOdom()
    pass