import rospy
from std_msgs.msg import Float64
import threading
from gazebo_msgs.srv import *
import geometry_msgs.msg
import tf2_ros.transform_broadcaster
import math  
import tf
from sensor_msgs.msg import  LaserScan
from nav_msgs.msg import Odometry

leftup = "/joint1_position_controller/command"
leftdown = "/joint2_position_controller/command"
rightup = "/joint3_position_controller/command"
rightdown = "/joint4_position_controller/command"
rospy.init_node("control", anonymous=True)
topic_leftup    = rospy.Publisher(leftup    , Float64,  queue_size=10)
topic_leftdown  = rospy.Publisher(leftdown  , Float64,  queue_size=10)
topic_rightup   = rospy.Publisher(rightup   , Float64,  queue_size=10)
topic_rightdown = rospy.Publisher(rightdown , Float64,  queue_size=10)
pub = rospy.Publisher("scan",LaserScan,queue_size=10)
curspeed=0.5

def send(data_leftup,data_leftdown,data_rightup,data_rightdown):
    data=Float64()
    data.data=data_leftup
    topic_leftup.publish(data)
    data.data=data_leftdown
    topic_leftdown.publish(data)
    data.data=data_rightup
    topic_rightup.publish(data)
    data.data=data_rightdown
    topic_rightdown.publish(data)
    print("method be called")
    pass

def pubOdom():
    rospy.wait_for_service("/gazebo/get_model_state")
    client = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState)
    rate = rospy.Rate(25)

    while  not rospy.is_shutdown():
        req = GetModelStateRequest()
        req.model_name="robot"
        req.relative_entity_name="ground_plane"
        rps= client.call(req)
        pose = rps.pose
        m=tf.TransformBroadcaster()
        qu=[rps.pose.orientation.x,rps.pose.orientation.y,rps.pose.orientation.z,rps.pose.orientation.w]
        tf.transformations.euler_from_quaternion
        m.sendTransform((rps.pose.position.x,rps.pose.position.y,rps.pose.position.z),  
                     qu,rospy.Time.now(),  
                     "base_footprint",  
                     "odom")
        rate.sleep()

if __name__ == "__main__":
    threa=threading.Thread(target=pubOdom)
    threa.start()
    # change=threading.Thread(target=pubchange)
    # change.start()
    while(True):
        xx= raw_input()         
        if(xx=="a"):# left
            if(curspeed == 0):
                curspeed=0.5
            send(0,0,curspeed,curspeed)
        if(xx=="d"):
            send(curspeed*4,curspeed,curspeed*-1,curspeed*-1)
        if(xx=="w"):# up
            curspeed=curspeed+0.15
            send(curspeed,curspeed,curspeed,curspeed)
        if(xx=="s" ):
            if(curspeed-0.15>0):
                curspeed=curspeed-0.15
            else:
                curspeed=0.0
            send(curspeed,curspeed,curspeed,curspeed)        
        print "speed:"+ str(curspeed)  
    pass