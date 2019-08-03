import rospy
from std_msgs.msg import Float64
import threading
import geometry_msgs.msg
import tf2_ros.transform_broadcaster
import math
import tf
import math
import time

rospy.init_node("control", anonymous=True)
br = tf.TransformBroadcaster()


class footState:
    '''

    '''

    def __init__(self, foot, upfoot, downfoot, upJointCommandTopic, downJointCommandTopic, maxAngle):
        self.upJointCommandTopic = upJointCommandTopic
        self.downJointCommandTopic = downJointCommandTopic
        self.upfoot = upfoot
        self.downfoot = downfoot
        self.upSend = rospy.Publisher(
            upJointCommandTopic, Float64, queue_size=10)
        self.downSend = rospy.Publisher(
            downJointCommandTopic, Float64, queue_size=10)
        self.singleFootLength = 0.7
        self.height = 2*self.singleFootLength*math.cos(maxAngle)
        self.maxAngle = maxAngle
        self.upFootAngle = 0.0
        self.downFootAngle = 0.0
        self.direction = 0.0
        self.readState()
        self.foot = foot
        self.isRviz = False
        pass

    def readState(self):
        pass

    def sendDirectionCommand(self, direction):
        if(direction <= math.pi/2 - self.maxAngle):
            raise Exception("over max angle")
        if(direction > math.pi*3/2):
            direction = direction-math.pi*3/2
        directionLength = self.height/math.sin(direction)
        upFootAngle = math.pi - direction - \
            math.acos(directionLength/(2*self.singleFootLength))
        if(upFootAngle <= math.pi/2):
            upFootAngle = upFootAngle+1.5*math.pi
        else:
            upFootAngle = upFootAngle-math.pi/2
        downFootAngle = math.pi-2 * \
            math.asin(directionLength/(2*self.singleFootLength))
        self.sendUpAngle(upFootAngle)
        self.sendDownAngle(downFootAngle)
        pass

    def SendStepDirectCommand(self, direction, height):
        directionLength = height
        upFootAngle = math.pi - direction - \
            math.acos(directionLength/(2*self.singleFootLength))
        if(upFootAngle <= math.pi/2):
            upFootAngle = upFootAngle+1.5*math.pi
        else:
            upFootAngle = upFootAngle-math.pi/2
        downFootAngle = math.pi-2 * math.asin(directionLength/(2*self.singleFootLength))
        self.sendUpAngle(upFootAngle)
        self.sendDownAngle(downFootAngle)
        pass

    def sendUpAngle(self, angle):
        if(self.isRviz):
            if(self.foot.startswith("left")):
                x = 0
            else:
                x = 1.2
            if(self.foot.endswith("up")):
                y = 2
            else:
                y = 0
            z = 1.5
            roll =angle
            pitch = 0
            yaw = 0
            br.sendTransform((x, y, z),
                             tf.transformations.quaternion_from_euler(
                                 roll, pitch, yaw),
                             rospy.Time.now(), self.upfoot, "body",)
            pass
        else:
            command = Float64()
            command.data = angle
            self.upSend.publish(command)

    def sendDownAngle(self, angle):
        if(self.isRviz):
            x = 0
            y = 0
            z = -0.7
            roll = angle
            pitch = 0
            yaw = 0
            br.sendTransform((x, y, z),
                             tf.transformations.quaternion_from_euler(
                                 roll, pitch, yaw),
                             rospy.Time.now(), self.downfoot, self.upfoot)
            pass
        else:
            command = Float64()
            command.data = angle
            self.downSend.publish(command)

    def getDirection(self):
        pass

    def stepForward(self):
        pass


class fourFootControl:

    def __init__(self):
        self.footLength = 0.0
        self.foots = ["leftup", "leftdown", "rightup", "rightdown"]
        self.footdic = {}
        self.footlocation = {}
        temp = 1
        for foot in self.foots:
            footst = footState(foot, "up_"+foot, "down_"+foot, self.getTopic(
                temp), self.getTopic(temp+4), math.pi/6)
            self.footdic[foot] = footst
            temp = temp+1
        self.resetlocation()
        self.lastlocation = {}
        self.height = self.footdic["leftup"].height
        self.stepAngle = math.pi/6
        pass

    def recordCurrentLocation(self):
        for foot in self.foots:
            self.lastlocation[foot] = self.footlocation[foot]
        pass

    def resetlocation(self):
        for foot in self.foots:
            if(foot.endswith("up")):
                self.footlocation[foot] = 0.0
            else:
                self.footlocation[foot] = 0.0

    def stand(self):
        for foot in self.foots:
            self.sendCommand(foot, math.pi/2)
        self.resetlocation()

    def frezz(self):
        for foot in self.foots:
            self.footdic[foot].sendUpAngle(0)
            self.footdic[foot].sendDownAngle(0)
        self.resetlocation()

    def stepForward(self, foot, angle, time, timeGap):
        self.recordCurrentLocation()
        rate = rospy.Rate(1000/timeGap)
        aviableCount = time/timeGap*1.0
        count = 1.0
        if(self.lastlocation[foot] < math.pi/2):
            distance = angle-self.lastlocation[foot]
        else:
            distance = angle+math.pi*2-self.lastlocation[foot]
        while(count <= aviableCount):
            percent = count/aviableCount
            heightpercent = 1 - (-1.2*(percent-0.5)*(percent-0.5)+0.3)
            if(self.lastlocation[foot] < math.pi/2):
                ambitionAngle = (math.pi/2) - \
                    (self.lastlocation[foot]+distance*percent)
            else:
                ambitionAngle = math.pi-distance*percent - \
                    (self.lastlocation[foot]-math.pi*3/2)
            ambitionHeight = self.height/math.sin(ambitionAngle)*heightpercent
            self.sendStepCommand(foot, ambitionAngle, ambitionHeight)
            self.footlocation[foot] = math.pi/2 - ambitionAngle
            for otherfoot in self.foots:
                if(foot != otherfoot):
                    cAngle = self.getCalculableAngle(
                        self.lastlocation[otherfoot])
                    cAngle = cAngle-distance*percent/3.0
                    readlAngle = self.getRealAngle(cAngle)
                    self.sendCommand(otherfoot, math.pi-cAngle)
                    self.footlocation[otherfoot] = readlAngle
            count = count+1
            self.debugInfo = {}
            for foottemp in self.foots:
                self.debugInfo[foottemp] = self.footlocation[foottemp] / \
                    (math.pi/180)
            rate.sleep()
        pass

    def getCalculableAngle(self, Angle):
        if(Angle < math.pi):
            Angle = Angle+math.pi/2
        else:
            Angle = Angle-math.pi*3/2
        return Angle

    def getRealAngle(self, angle):
        if(angle > math.pi/2):
            return angle-math.pi/2
        else:
            return angle+math.pi*3/2

    def sendCommand(self, foot, direction):
        self.footdic[foot].sendDirectionCommand(direction)
        pass

    def sendStepCommand(self, foot, direction, height):
        self.footdic[foot].SendStepDirectCommand(direction, height)
        pass

    def getTopic(self, num):
        return "joint"+str(num)+"_position_controller/command"


'''
if __name__ == "__main__":
    control = fourFootControl()
    chang = True
    while(True):
        control.stand()
        time.sleep(4)
    pass

'''



if __name__ == "__main__":
    control = fourFootControl()
    chang = True
    control.stand()
    time.sleep(2)
    while(True):
        for foot in ["leftup", "leftdown", "rightup", "rightdown"]:
            control.stepForward(foot, (math.pi/180)*20, 900, 150)
    pass
