#!/usr/bin/env python

# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# Ros libraries
import roslib
import rospy

# Ros Messages
from naoqi_bridge_msgs.msg import TactileTouch
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from sensor_msgs.msg import Image 
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

from std_srvs.srv import Empty

# OpenCV
import cv2
import imutils # collection of OpenCV convenience functions to make a few basic tasks (like resizing) much easier
from cv_bridge import CvBridge

# Time
import time 


class tracking_tactile:

  def __init__(self):
    
    
    self.tracking = 0 # Global variable to know when it is being made the colour range detection and when the tracking is ON
    self.CabezaInclinada=False
    self.contador=0
    
    self.tactile_sub = rospy.Subscriber("/nao_robot/tactile/tactile_touch", TactileTouch, self.callback_tactile) 
    
    self.image_sub = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, self.callback_camera)
    
    self.publisher = rospy.Publisher("/nao_camara_prueba/image", Image, queue_size=1) # Publish the modified image on a new topic
    
    self.bridge = CvBridge()
    
    self.joint_pub = rospy.Publisher('/joint_angles', JointAnglesWithSpeed, queue_size=1)

    self.subs = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
    
    self.walk_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    
    self.pointPub = rospy.Publisher("point", Point, queue_size=1 )
        

  def callback_camera(self, ros_data):
    
     # When te ball is detected, save the desired frame and process the image to obtain de HSV values
    if self.tracking == 0:
 
      self.cv_image = self.bridge.imgmsg_to_cv2(ros_data, desired_encoding="passthrough")
  
    elif self.tracking == 1:
      
      # Use the bridge to convert the ROS Image message to OpenCV message
      cv_image = self.bridge.imgmsg_to_cv2(ros_data, desired_encoding="passthrough")
	  
      # convert the cv_image to the HSV color space
      hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) 

      # construct a mask for the color "colour", then perform a series of dilations and erosions to remove any small blobs left in the mask
      mask = cv2.inRange(hsv, self.colourLower, self.colourUpper)
      mask = cv2.erode(mask, None, iterations=2) 
      mask = cv2.dilate(mask, None, iterations=2) 

      # find contours in the mask and initialize the current (x, y) center of the ball
      cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)[-2]
      center = None

      # only proceed if at least one contour was found
      if len(cnts) > 0:
	# find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
	c = max(cnts, key=cv2.contourArea)
	((x, y), radius) = cv2.minEnclosingCircle(c)
	M = cv2.moments(c)
	center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	
	# only proceed if the radius meets a minimum size
	if radius > 10:
	  # draw the circle and centroid on the cv_image
	  cv2.circle(cv_image, (int(x), int(y)), int(radius),(0, 255, 255), 2)
	  cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
	  	  
       # Use the bridge to convert the OpenCV message to ROS Image message. Publish the modified image on a new topic
      ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
      self.publisher.publish(ros_image)

      # Work out the head movement required to keep the ball in the image center
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
	  
      if (new_position_y <= -0.45 ): 
	var_y=0
      elif (new_position_y >= 0.3):
	var_y=0
	    
      joint.joint_angles.append(-var_x*0.25) 
      joint.joint_angles.append(var_y*0.10) 
	
      self.joint_pub.publish(joint)
      
      # Movement towards the ball (walk)
      walk=Twist()
      
      
      print " Radius is (px): ", radius
      print "HeadPitch: ", self.positions[1]
      print "HeadYaw: ", self.positions[0] # HeadYaw <-->
      if radius < 58: # Walk forward if the ball is far. When radius = 50, Nao is aprox. 25cm from the ball
	
	# Walk
	if self.positions[0] < (-0.3):
	  walk.linear.x=0.2
	  walk.angular.z=-0.25
	  
	elif (self.positions[0] >= -0.3) and (self.positions[0] < 0.3):
	  walk.linear.x=0.3
	  walk.angular.z=0.0
	
	elif self.positions[0] >= 0.3:
	  walk.linear.x=0.2
	  walk.angular.z=0.25
	
	walk.linear.y=0.0
	walk.linear.z=0.0
	
	walk.angular.x=0.0
	walk.angular.y=0.0
	
	self.walk_pub.publish(walk)
	
	
      elif radius > 68: # If it is too close, walk backwards
	walk.linear.x=-0.2
	walk.linear.y=0.0  	  
	walk.linear.z=0.0
	
	walk.angular.x=0.0
	walk.angular.y=0.0
	walk.angular.z=0.0
	
	self.walk_pub.publish(walk)
	print "Walk backwards"
      
      else:

	CabezaCentrada=False
	PelotaCentro=False
	
	# Check if the ball is in front of Nao, if not: Nao walks to the corresponding side
	# Check it the head is aligned with the body, and that is not turned
	if self.positions[0] < (-0.3) or self.positions[0] > 0.3: # Use a proportional controller to move with more precision
	  walk.linear.y=0.0
	  e=0-self.positions[0]
	  walk.angular.z=e*(-0.5)

	elif self.positions[0] <  (-0.05): # HeadYaw <-->
	  walk.linear.y=-0.3
	  walk.angular.z=-0.0
	  
	elif self.positions[0] > 0.05:
	  walk.linear.y=0.3
	  walk.angular.z=0.0

	else: # Stop
	  CabezaCentrada=True
	  
	  walk.linear.y=0.0
	  walk.angular.z=0.0
	  print " Head centered "
	
	  walk.linear.x=0.0  	  
	  walk.linear.z=0.0
	
	  walk.angular.x=0.0
	  walk.angular.y=0.0
	
	  self.walk_pub.publish(walk)
	  
	  rospy.sleep(0.3) # Wait some time for the robot to become steady
#
	  if center[0]< 152 or center [0]> 168: # Check if the ball is in the image center looking at its X coordinate
	    e=160-center[0]
	    walk.linear.y=e*(-0.005)
	    walk.angular.z=-0.0
	  else:	# Parar
	    PelotaCentro=True
	    walk.linear.y=0.0
	    walk.angular.z=0.0
	    print " Ball centered "
	
	walk.linear.x=0.0  	  
	walk.linear.z=0.0
	
	walk.angular.x=0.0
	walk.angular.y=0.0
	
	self.walk_pub.publish(walk)
	
	print " Center is: ", center
	print "Pelota Centro: ", PelotaCentro
	print "Cabeza Centrada: ", CabezaCentrada
	
	if PelotaCentro and CabezaCentrada:
	
	  self.image_sub.unregister()

	  # Subscribe to another callback
	  self.image_sub = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, self.callback_camera2)

  def callback_camera2(self, ros_data):
    print "callback camera 2: centrado de pelota en la imagen"
    

    # Use the bridge to convert the ROS Image message to OpenCV message
    cv_image = self.bridge.imgmsg_to_cv2(ros_data, desired_encoding="passthrough")
	  
    # convert the cv_image to the HSV color space
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) 

    # construct a mask for the color "colour", then perform a series of dilations and erosions to remove any small blobs left in the mask
    mask = cv2.inRange(hsv, self.colourLower, self.colourUpper)
    mask = cv2.erode(mask, None, iterations=2) 
    mask = cv2.dilate(mask, None, iterations=2) 

    # find contours in the mask and initialize the current (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
      cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
    # find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
      c = max(cnts, key=cv2.contourArea)
      ((x, y), radius) = cv2.minEnclosingCircle(c)
      M = cv2.moments(c)
      center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	
      # only proceed if the radius meets a minimum size
      if radius > 10:
	# draw the circle and centroid on the cv_image
	cv2.circle(cv_image, (int(x), int(y)), int(radius),(0, 255, 255), 2)
	cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
	  	  
    # Use the bridge to convert the OpenCV message to ROS Image message. Publish the modified image on a new topic
    ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
    self.publisher.publish(ros_image)

    if center[0]<155 or center[0]>165: # Check X coordinate
      self.contador=0 # Initialize counter
      # Work out the head movement required to keep the ball in the image center
      # Velocities message initialization
      joint = JointAnglesWithSpeed()
      joint.joint_names.append("HeadYaw")
      joint.joint_names.append("HeadPitch")
      joint.speed = 0.1
      joint.relative = True
	  
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
      new_position_y=self.positions[1]+var_y*0.15

      if (new_position_y <= -0.45 ): 
	var_y=0
      elif (new_position_y >= 0.3):
	var_y=0
	    
      joint.joint_angles.append(-var_x*0.10) #0.25
      joint.joint_angles.append(var_y*0.15) 
	
      self.joint_pub.publish(joint)
    else:
      print "Centrado respecto a X"
      if center[1]<115 or center[1]>125: # Check Y coordinate
	self.contador=0
	# Velocities message initialization
	joint = JointAnglesWithSpeed()
	#joint.joint_names.append("HeadYaw")
	joint.joint_names.append("HeadPitch")
	joint.speed = 0.1
	joint.relative = True#True
	  
	if center[1]<115:
	  var_y=-0.01
	elif center[1]>125:
	  var_y=0.01
	  
	joint.joint_angles.append(var_y)
	
	self.joint_pub.publish(joint)
      else:
	# Counter to ensure the robot is centeredr and it was nota coincidence
	self.contador=self.contador + 1
	print "Centrado respecto a Y"
	print "Contador: ", self.contador
	if self.contador > 10:
	  self.image_sub.unregister()
	  print "Centrado y salimos"
	  print "HeadPitch: ", self.positions[1] # HeadPitch
	  print "HeadYaw: ", self.positions[0] # HeadYaw <--> 
	  print " Center is: ", center
	  
	  # Obtain position
	  Dreal=0.08 # Real Ball diameter (meters)
	  pixelToMeter=Dreal/(radius*2) 
      
	  Yrobot=(cv_image.shape[1]/2 - center[0])*pixelToMeter 
	
	  radiusToMeters=0.25*55 # Distance: 0.25m -> radius = 55px 
	  Xrobot=radiusToMeters/radius
	  
	  # Calculate Z coordinate with the equation that connects it with HeadPitch
	  Zrobot=(-25.866*self.positions[1]+23.843)/100 # Divide by 100 to convert centimeters to meters
	  
	  # Create point
	  point = Point()
	  point.x = Xrobot
	  point.y = Yrobot
	  point.z = Zrobot
	
	  self.pointPub.publish(point)
	  print "Punto publicado: ", point
    
    print " El radio son (px): ", radius
    print "HeadPitch: ", self.positions[1]
    print "HeadYaw: ", self.positions[0] # HeadYaw <--> 
    print " Center is: ", center
	  
#    self.image_sub.unregister()

	  
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
  
  
  def callback_tactile(self, data):
    rospy.loginfo( "I heard %u %u" % (data.button, data.state))
    
    if data.button == 1 and data.state == 1:
      joint_pub = rospy.Publisher('/joint_angles', JointAnglesWithSpeed, queue_size=10)
      
      rospy.loginfo("Pulsado %u , pasamos" % (data.button))
      joint = JointAnglesWithSpeed()
      joint.joint_names.append("LShoulderRoll")
      joint.joint_angles.append(-0.3) 
      joint.joint_names.append("LShoulderPitch")
      joint.joint_angles.append(-0.2) 
      joint.joint_names.append("LElbowYaw")
      joint.joint_angles.append(-1) 
      joint.joint_names.append("LElbowRoll")
      joint.joint_angles.append(0) 
      joint.joint_names.append("LWristYaw")
      joint.joint_angles.append(-0.5) 
      joint.joint_names.append("LHand")
      joint.joint_angles.append(0)
          
      joint.joint_names.append("RShoulderRoll")
      joint.joint_angles.append(0.3) 
      joint.joint_names.append("RShoulderPitch")
      joint.joint_angles.append(-0.2) 
      joint.joint_names.append("RElbowYaw")
      joint.joint_angles.append(1) 
      joint.joint_names.append("RElbowRoll")
      joint.joint_angles.append(0) 
      joint.joint_names.append("RWristYaw")
      joint.joint_angles.append(0.5) 
      joint.joint_names.append("RHand")
      joint.joint_angles.append(0)
      
      joint.joint_names.append("HeadYaw")
      joint.joint_angles.append(0)
      joint.joint_names.append("HeadPitch")
      joint.joint_angles.append(0)

      joint.speed = 0.1
      joint.relative = False#True

      joint_pub.publish(joint)
      
      # Deactivate tracking mode because the HSV range recongition is in process
      self.tracking = 0
      
      print "Tracking desactivado"

    elif data.button == 2 and data.state == 1: 
      # Save a frame from the camera to detect the ball colour
      
      output = self.cv_image.copy() # draw our detected circles without destroying the original image.
      gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY) # the cv2.HoughCircles function requires an 8-bit, single channel image, so we'll go ahead and convert from the RGB color space to grayscale

      # detect circles in the image
      circles = cv2.HoughCircles(gray, cv2.cv.CV_HOUGH_GRADIENT, 1.2, 100, param2=80,minRadius=50,maxRadius=200)
      
      if circles == None:
	print 'No se ha detectado la pelota'
      else:
	print 'Se ha detectado la pelota: ', circles
	# ensure at least some circles were found
	
	
      if circles is not None:
	# convert the (x, y) coordinates and radius of the circles to integers allowing us to draw them on our output image
	circles = np.round(circles[0, :]).astype("int")
 
	# loop over the (x, y) coordinates and radius of the circles
	for (x, y, r) in circles:
		# draw the circle in the output image, then draw a rectangle
		# corresponding to the center of the circle
	  cv2.circle(output, (x, y), r, (0, 255, 0), 4)
	  cv2.rectangle(output, (x - 45, y - 45), (x + 45, y + 45), (0, 128, 255), -1)
	
	hsv=cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
	
	# Calculate the HSV medium values inside a  rectangle near to the ball center.
	s=[0,0,0]
	for i in range (y-45,y+46):
	  for j in range (x-45,x+46):
	    s=s+hsv[i,j]
	    #print j,i, hsv[i,j]
	s=s/((45+46)*(45+46))
	print s
	
	# Calculate the upper and lower limits from the HSV medium values
	if s[1] < 100: # Saturation<100
	  if (s[0]-10) < 0:
	    self.colourLower = (0, 40, 40)	
	  else:
	    self.colourLower = (s[0]-10, 40, 40)
	else:
	  if (s[0]-10) < 0:
	    self.colourLower = (0, 100, 100)	
	  else:
	    self.colourLower = (s[0]-10, 100, 100)
	  
	self.colourUpper = (s[0]+10, 255, 255)
	
	
	print "Colour range. Upper and lower limits: ", self.colourUpper, self.colourLower

    
    elif data.button == 3 and data.state == 1: 
      joint_pub = rospy.Publisher('/joint_angles', JointAnglesWithSpeed, queue_size=10)
      
      rospy.loginfo("Pulsado %u , pasamos" % (data.button))
      joint = JointAnglesWithSpeed()
      joint.joint_names.append("LShoulderRoll")
      joint.joint_angles.append(0) 
      joint.joint_names.append("LShoulderPitch")
      joint.joint_angles.append(1.3) 
      joint.joint_names.append("LElbowYaw")
      joint.joint_angles.append(0) 
      joint.joint_names.append("LElbowRoll")
      joint.joint_angles.append(0) 
      joint.joint_names.append("LWristYaw")
      joint.joint_angles.append(0) 
      joint.joint_names.append("LHand")
      joint.joint_angles.append(0)
      
      joint.joint_names.append("RShoulderRoll")
      joint.joint_angles.append(0) 
      joint.joint_names.append("RShoulderPitch")
      joint.joint_angles.append(1.3) 
      joint.joint_names.append("RElbowYaw")
      joint.joint_angles.append(0) 
      joint.joint_names.append("RElbowRoll")
      joint.joint_angles.append(0) 
      joint.joint_names.append("RWristYaw")
      joint.joint_angles.append(0) 
      joint.joint_names.append("RHand")
      joint.joint_angles.append(0)

      joint.speed = 0.1
      joint.relative = False#True

      joint_pub.publish(joint)
      
      # colour range detected. Activate tracking mode
      self.tracking = 1
      
      print "Tracking ON"

def rest(): 
  rospy.wait_for_service('rest')
  try:
    rest = rospy.ServiceProxy('rest', Empty)
    resp1 = rest()
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

def main():

    instancia = tracking_tactile()
    rospy.init_node('tracking_tactile', anonymous=True)
    #rospy.on_shutdown(rest)
    rospy.wait_for_service('wakeup')
    try:
      wakeup = rospy.ServiceProxy('wakeup', Empty)
      resp1 = wakeup()
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
    
    try:
	    rospy.spin() # spin() simply keeps python from exiting until this node is stopped
    except rospy.ROSInterruptException:
	    print "Shutting down ROS ball tracking module"
	    
    cv2.destroyAllWindows()

   
if __name__ == '__main__':
    main()
    