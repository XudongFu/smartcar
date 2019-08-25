
import rospy
from sensor_msgs.msg import *
from geometry_msgs.msg import Point32
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

rospy.init_node("pointCloud")
pub= rospy.Publisher("point",PointCloud,queue_size=10)


def getMatchPic(curr,Left,right):
    pp=PointCloud()
    pp.header.frame_id="body"
    row,col=Left.shape
    picwidth=50
    width=picwidth+row
    half=int(row/2)
    for y in range(row):
        for x in range(y):
            color=Left[y]-right[x]
            sum=0
            for c in range(3):
                sum=sum+ abs(color[c])            
            if(sum>4):
                sum=255
            else:
                temp=Point32()
                temp.z=curr*0.1
                temp.x=y*0.1
                temp.y=x*0.1
                pp.points.append(temp)
    pub.publish(pp)  
    pass


def test():
    left = np.array(Image.open("/home/free/roscode/TwoEyeDetect/twoEyePic/left1.png"))
    right=np.array(Image.open("/home/free/roscode/TwoEyeDetect/twoEyePic/right1.png"))
    # left=np.array(Image.open("F:/code/TwoEyeDetect/twoEyePic/left2.png"))
    # right=np.array(Image.open("F:/code/TwoEyeDetect/twoEyePic/right2.png"))
    rows,cols,dims=left.shape
    ra = rospy.Rate(10)
    for curr in range(20):
        dataY=left[curr*30,:,:]
        dataZ=right[curr*30,:,:]
        getMatchPic(curr*30,dataY,dataZ)
        ra.sleep()


if __name__ == "__main__":
    ra = rospy.Rate(1)
    test()
    while(True):
        # pp=PointCloud()
        # pp.header.frame_id="body"
        # for x in range(100):
        #     temp=Point32()
        #     temp.x=x
        #     temp.y=x
        #     temp.z=x
        #     pp.points.append(temp)
        # pub.publish(pp)
        ra.sleep()
    pass

