import rospy
from std_msgs.msg import Float64
import threading
import geometry_msgs.msg
import tf2_ros.transform_broadcaster
import math  
import tf
import math

rospy.init_node("control", anonymous=True)

class footState:
    '''

    '''
    def __init__(self,upJointCommandTopic,downJointCommandTopic,maxAngle):
        self.upJointCommandTopic=upJointCommandTopic
        self.downJointCommandTopic=downJointCommandTopic
        self.upSend=rospy.Publisher(upJointCommandTopic,Float64,queue_size=10)
        self.downSend=rospy.Publisher(downJointCommandTopic,Float64,queue_size=10)
        self.singleFootLength=0.7
        self.height=2*self.singleFootLength*math.cos(maxAngle)
        self.upFootAngle=0.0
        self.downFootAngle=0.0
        self.direction=0.0        
        self.readState()
        pass

    def readState(self):
        pass

    def sendDirectionCommand(self,direction):
        directionLength=self.height/math.sin(direction)
        upFootAngle=math.pi- direction-math.acos(directionLength/(2*self.singleFootLength))
        if(upFootAngle<=math.pi/2):
            upFootAngle=upFootAngle+1.5*math.pi
        else:
            upFootAngle=upFootAngle-math.pi/2
        downFootAngle=math.pi-2*math.asin(directionLength/(2*self.singleFootLength))
        command=Float64()
        command.data=upFootAngle
        self.upSend.publish(command)
        command.data=downFootAngle
        self.downSend.publish(command)
        pass
    
    def sendUpAngle(self,angle):
        command=Float64()
        command.data=angle
        self.upSend.publish(command)

    def sendDownAngle(self,angle):
        command=Float64()
        command.data=angle
        self.downSend.publish(command)


    def getDirection(self):
        pass



class fourFootControl:

    def __init__(self):        
        self.height=0.0
        self.footLength=0.0
        self.foots=["upleft","downleft","upright","downright"]        
        self.footdic={}
        temp=1
        for foot in self.foots:
            self.footdic[foot]=footState(self.getTopic(temp),self.getTopic(temp+4),math.pi/6)
            temp=temp+1        
        pass

    def stand(self):
        for foot in self.foots:
            self.sendCommand(foot,math.pi/2)

    def frezz(self):
        for foot in self.foots:
            self.footdic[foot].sendUpAngle(0)
            self.footdic[foot].sendDownAngle(0)
        pass

    def sendCommand(self,foot,direction):
        self.footdic[foot].sendDirectionCommand(direction)
        pass
    
    def getTopic(self,num):
        return "joint"+str(num)+"_position_controller/command"


if __name__ == "__main__":
    control=fourFootControl()
    rate = rospy.Rate(10)
    count=0
    chang = True
    while(True):
        if(count%40==0):
            chang= not chang
        if(chang):
            control.stand()
        else:
            control.frezz()
        rate.sleep()
        count=count+1
    pass