#!/usr/bin/env python

# Python libs
import sys

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2
import imutils # collection of OpenCV convenience functions to make a few basic tasks (like resizing) much easier
from cv_bridge import CvBridge

# Ros libraries
import roslib
import rospy

# Ros Messages
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from sensor_msgs.msg import Image 
from sensor_msgs.msg import JointState

from std_srvs.srv import Empty


class face_tracking:

	def __init__(self):
	  #Initialize ros publisher, ros subscriber
	  self.preparado = False
	  self.joint_pub = rospy.Publisher('/joint_angles', JointAnglesWithSpeed, queue_size=1)
	  self.bridge = CvBridge()
	  
	  self.subs = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
	  # Subscription to the top camera of the nao_robot
	  self.subscriber = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, self.callback)
	  self.publisher = rospy.Publisher("/nao_camara_prueba/image", Image, queue_size=1) # Publish the modified image 
	    
	def joint_states_callback(self, msg):
	  # Callback function: when a joint_states message arrives, save the values
	  self.name = msg.name
	  self.position = msg.position
	  self.velocity = msg.velocity
	  self.effort = msg.effort
	  
	  joint_names=['HeadYaw', 'HeadPitch']
	  self.positions = []

	  for i in joint_names:
	    joint_name = i
	  
	    if joint_name in self.name:
	      index = self.name.index(joint_name)
	      position = self.position[index]
	      
	      self.positions.append(position)

	def callback(self, ros_image):
	  self.preparado = True
	  self.ros_data = ros_image 
	  
  
	def facecallback(self, event):
	  # Callback of the top camera of the nao_robot
	  
	  # Use the bridge to convert the ROS Image message to OpenCV message
	
	  if self.preparado is not True:
	    return
	  
	  cv_image = self.bridge.imgmsg_to_cv2(self.ros_data, desired_encoding="passthrough")
	    
	  face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

	  gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)

	  
	  faces = face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5, minSize=(30, 30), flags=cv2.cv.CV_HAAR_SCALE_IMAGE)
	  
	  # Draw a rectangle around the detected faces
	  for (x,y,w,h) in faces:
	    cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2) # The center of the rectangle is: (x+w/2,y+h/2)
	  
	  # Look for the biggest face, the one to track
	  if len(faces) > 0:
	    c=faces[0]
	    for i in faces:
	      if i[2] > c[2]:
	        c=i

	    cv2.rectangle(cv_image,(c[0],c[1]),(c[0]+c[2],c[1]+c[3]),(255,255,0),2)
	  
	    center=(c[0]+c[2]/2,c[1]+c[3]/2) # Center of the face
	    cv2.circle(cv_image, center, 3, (0, 0, 255), -1)
	  
	  else:
	    center=(cv_image.shape[1]/2,cv_image.shape[0]/2) # Image center
	    print "No faces found" 

	  # Convert the image format from OpenCV to ROS and publish the modified image on a new topic
	  ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
	  self.publisher.publish(ros_image)
	  
	  ### Work out the movemento of the head required to keep the ball center in the image center
	  # Velocities message initialization
	  joint = JointAnglesWithSpeed()
	  joint.joint_names.append("HeadYaw")
	  joint.joint_names.append("HeadPitch")
	  joint.speed = 0.1
	  joint.relative = True#True
	  
	  # Get center of detected object and calculate the head turns	
	  target_x =center[0] # centroid coordenate x
	  target_y=center[1] # centroid coordenate y
	  
	  # Image center -> (cv_image.shape[1]/2, cv_image.shape[0]/2)=(320/2,240/2)    (x,y) coordenates
	  sub_x = target_x - cv_image.shape[1]/2
	  sub_y = target_y - cv_image.shape[0]/2

	  var_x = (sub_x / float(cv_image.shape[1]/2))
	  var_y = (sub_y / float(cv_image.shape[0]/2))
	  
	  # Check the Head position before moving it to restrict its movement and prevent it from crashing with the shoulders
	  
	  # Predict the new Y relative position posicion, the one to be restricted to avoid crashes
	  new_position_y=self.positions[1]+var_y*0.10
	  
	  if (new_position_y <= -0.45 ): # Recalculate the maximun variation
	    var_y=0
	  elif (new_position_y >= 0.3):
	    var_y=0
	    
	  joint.joint_angles.append(-var_x*0.35) 
	  joint.joint_angles.append(var_y*0.15) 
	
	  self.joint_pub.publish(joint)
	  
        def iniciarTimer(self):
	  rospy.Timer(rospy.Duration(0.100), self.facecallback)

def rest(): 
  rospy.wait_for_service('rest')
  try:
    rest = rospy.ServiceProxy('rest', Empty)
    resp1 = rest()
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e	  

def main():
  #'''Initializes and cleanup ros node'''
  ic = face_tracking()
  rospy.init_node('face_tracking', anonymous=True) # Important!, create a node to be able to communicate with ROS Master
  rospy.on_shutdown(rest)
  rospy.wait_for_service('wakeup')
  try:
    wakeup = rospy.ServiceProxy('wakeup', Empty)
    resp1 = wakeup()
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e
  
  try:
    ic.iniciarTimer()
    rospy.spin() # spin() simply keeps python from exiting until this node is stopped
  except KeyboardInterrupt:
    print "Shutting down ROS face tracking module"
    
    cv2.destroyAllWindows()
   
if __name__ == '__main__':
    main()