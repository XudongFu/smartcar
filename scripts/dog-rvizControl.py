import rospy
from std_msgs.msg import Float64
import threading
import geometry_msgs.msg
import tf2_ros.transform_broadcaster
import math
import tf
import math
import time
if __name__ == "__main__":
    rospy.init_node("control", anonymous=True)
    rate=rospy.Rate(20)
    count=0
    while(True):
        br = tf.TransformBroadcaster()
        count=count+10
        x = 0
        y = 2
        z = 1.4
        roll = math.radians(count)
        pitch = 0
        yaw = 0
        br.sendTransform((x,y,z),  
        tf.transformations.quaternion_from_euler(roll,pitch,yaw),  
        rospy.Time.now(),"up_leftup" ,"body")
        x = 0
        y = 0
        z = -0.7
        roll = math.radians(count)
        pitch = 0
        yaw = 0
        br.sendTransform((x,y,z),  
        tf.transformations.quaternion_from_euler(roll,pitch,yaw),  
        rospy.Time.now(), "down_leftup","up_leftup")
        rate.sleep()