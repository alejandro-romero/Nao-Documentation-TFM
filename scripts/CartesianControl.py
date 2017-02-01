# -*- encoding: UTF-8 -*-
import sys
import almath
import motion

from naoqi import ALProxy

import rospy
from geometry_msgs.msg import Point


class CartesianControl:
  
  def __init__(self):
  
    robotIP = "169.254.254.250"
    PORT = 9559

    self.motionProxy  = ALProxy("ALMotion", robotIP, PORT)
    self.postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

    # Wake up robot
    #self.motionProxy.wakeUp()

    # Send robot to Pose Init
    #postureProxy.goToPosture("StandInit", 0.5)
    
    self.Point_sub = rospy.Subscriber("point", Point, self.callback)
    
    
  def callback(self,point):
    
    # Send robot to Pose Init
    self.postureProxy.goToPosture("StandInit", 0.5)
  
    # Example showing how to use positionInterpolations
    frame = motion.FRAME_ROBOT
    useSensorValues = False

    dx = 0.105 # translation axis X (meters)
    dy = 0.05 # translation axis Y (meters)
    
    if point.z > 0.2: # Limit arms movement according to the height
      dz=0.10
    else:
      dz=0.13 #(meters)
    
    # Motion of Arms with block process
    effectorList = []
    pathList     = []

    axisMaskList = [motion.AXIS_MASK_VEL, motion.AXIS_MASK_VEL]
    timeList     = [[1.0], [1.0]]         # seconds
    
    
    ## Move to the ball, but don't grab it yet
    
    effectorList.append("LArm")
    currentPos = self.motionProxy.getPosition("LArm", frame, useSensorValues)
    targetPos = almath.Position6D(currentPos)
    targetPos.x += dx
    targetPos.y -=(dy-0.02)
    targetPos.z += point.z
    pathList.append(list(targetPos.toVector()))

    effectorList.append("RArm")
    currentPos = self.motionProxy.getPosition("RArm", frame, useSensorValues)
    targetPos = almath.Position6D(currentPos)
    targetPos.x += dx
    targetPos.y += (dy-0.02)
    targetPos.z += point.z    
    pathList.append(list(targetPos.toVector()))

    self.motionProxy.positionInterpolations(effectorList, frame, pathList, axisMaskList, timeList)
    
    ## Grab the ball
    
    # OpenHands
    self.motionProxy.openHand('LHand')
    self.motionProxy.openHand('RHand')
    
    pathList     = []
    #effectorList.append("LArm")
    currentPos = self.motionProxy.getPosition("LArm", frame, useSensorValues)
    targetPos = almath.Position6D(currentPos)
    #targetPos.x += dx
    targetPos.y -= (dy+0.02)
    #targetPos.z += dz
    pathList.append(list(targetPos.toVector()))

    #effectorList.append("RArm")
    currentPos = self.motionProxy.getPosition("RArm", frame, useSensorValues)
    targetPos = almath.Position6D(currentPos)
    #targetPos.x += dx
    targetPos.y += (dy+0.02)
    #targetPos.z += dz    
    pathList.append(list(targetPos.toVector()))

    self.motionProxy.positionInterpolations(effectorList, frame, pathList, axisMaskList, timeList)
    
    
    ## Lift arms a bit up and move to the robots chest to not hit the table and to be more stable while moving 
    
    pathList     = []
    #effectorList.append("LArm")
    currentPos = self.motionProxy.getPosition("LArm", frame, useSensorValues)
    targetPos = almath.Position6D(currentPos)
    targetPos.x -= dx
    #targetPos.y -= dy
    targetPos.z += dz
    pathList.append(list(targetPos.toVector()))

    #effectorList.append("RArm")
    currentPos = self.motionProxy.getPosition("RArm", frame, useSensorValues)
    targetPos = almath.Position6D(currentPos)
    targetPos.x -= dx
    #targetPos.y += dy
    targetPos.z += dz  
    pathList.append(list(targetPos.toVector()))

    self.motionProxy.positionInterpolations(effectorList, frame, pathList, axisMaskList, timeList)
    
    ## Move ball to the front to walk with it 
#    pathList     = []
    #effectorList.append("LArm")
#    currentPos = self.motionProxy.getPosition("LArm", frame, useSensorValues)
#    targetPos = almath.Position6D(currentPos)
#    targetPos.x += dx
    #targetPos.y -= dy
#    targetPos.z -= dz
#    pathList.append(list(targetPos.toVector()))

    #effectorList.append("RArm")
#    currentPos = self.motionProxy.getPosition("RArm", frame, useSensorValues)
#    targetPos = almath.Position6D(currentPos)
#    targetPos.x += dx
    #targetPos.y += dy
#    targetPos.z -= dz  
#    pathList.append(list(targetPos.toVector()))

#    self.motionProxy.positionInterpolations(effectorList, frame, pathList, axisMaskList, timeList)

def main():
  
    instancia = CartesianControl()
        
    rospy.init_node('CartesianControl', anonymous=True)

    rospy.loginfo("Starting CartesianControl")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":     
    main()
