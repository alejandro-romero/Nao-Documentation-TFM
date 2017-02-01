# -*- encoding: UTF-8 -*-

import time

from naoqi import ALProxy
import motion
import almath

# numpy
import numpy as np

# Ros Messages
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

# Ros libraries
import rospy

# OpenCV
import cv2
from cv_bridge import CvBridge

class tracker:

  def __init__(self):
    
    PORT = 9559
    robotIP = "169.254.254.250"
    
    self.motionProxy  = ALProxy("ALMotion", robotIP, PORT)
      
    self.postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
      
    self.asrProxy = ALProxy("ALSpeechRecognition", robotIP, PORT)
      
    self.ttsProxy = ALProxy("ALTextToSpeech", robotIP, PORT)

    self.Memory = ALProxy("ALMemory", robotIP, PORT)

    self.trackerProxy = ALProxy("ALTracker", robotIP, PORT)
    
    self.joint_pub = rospy.Publisher('/joint_angles', JointAnglesWithSpeed, queue_size=1)
    
    self.bridge = CvBridge()
    
    self.image_sub = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, self.callback_camera)
    
    self.subscribed = False # Variable to know if the robot is subscribed to the speech recognition engine
     
    self.tracking = 0 # Global variable to know when to start the tracking mode 
    
    self.publisher = rospy.Publisher("/nao_camara_prueba/image", Image, queue_size=1) # Publish the modified image in a topic
    
    self.joint_pub = rospy.Publisher('/joint_angles', JointAnglesWithSpeed, queue_size=1)
    
    self.walk_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    
    self.subs = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
    
    self.pointPub = rospy.Publisher("point", Point, queue_size=1 )
    
    self.Point_sub = rospy.Subscriber("point", Point, self.callback_point)
  
    self.end = False # Variable to know when to end the game
    
    self.play_again = False # Variable to know if the child wants to play again

  def callback_point(self,point):
    
    # Send robot to Pose Init
    self.postureProxy.goToPosture("StandInit", 0.5)
  
    frame = motion.FRAME_ROBOT # Establish the reference frame for the coordinates
    useSensorValues = False

    dx = 0.105 # translation axis X (meters)
    dy = 0.05 # translation axis Y (meters)
    
    if point.z > 0.2: # It limits what the arms raise upwards according to the height
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
    
    # Open Hands
    self.motionProxy.openHand('LHand')
    self.motionProxy.openHand('RHand')
    
    pathList     = []
    currentPos = self.motionProxy.getPosition("LArm", frame, useSensorValues)
    targetPos = almath.Position6D(currentPos)
    targetPos.y -= (dy+0.04)
    pathList.append(list(targetPos.toVector()))

    currentPos = self.motionProxy.getPosition("RArm", frame, useSensorValues)
    targetPos = almath.Position6D(currentPos)
    targetPos.y += (dy+0.04)
    pathList.append(list(targetPos.toVector()))

    self.motionProxy.positionInterpolations(effectorList, frame, pathList, axisMaskList, timeList)
    
    
    ## Lift arms a bit up and move to the robots chest to not hit the table and to be more stable while moving 
    
    pathList     = []
    currentPos = self.motionProxy.getPosition("LArm", frame, useSensorValues)
    targetPos = almath.Position6D(currentPos)
    targetPos.x -= dx
    targetPos.z += dz
    pathList.append(list(targetPos.toVector()))

    currentPos = self.motionProxy.getPosition("RArm", frame, useSensorValues)
    targetPos = almath.Position6D(currentPos)
    targetPos.x -= dx
    targetPos.z += dz  
    pathList.append(list(targetPos.toVector()))

    self.motionProxy.positionInterpolations(effectorList, frame, pathList, axisMaskList, timeList)


    self.ttsProxy.say("¡Ya tengo la pelota!")
    
    self.ttsProxy.say("Pero este color ya no me gusta")

    
    ## Throw the ball to the floor
    pathList     = []
    currentPos = self.motionProxy.getPosition("LArm", frame, useSensorValues)
    targetPos = almath.Position6D(currentPos)
    targetPos.x += dx
    targetPos.z -= dz
    pathList.append(list(targetPos.toVector()))

    currentPos = self.motionProxy.getPosition("RArm", frame, useSensorValues)
    targetPos = almath.Position6D(currentPos)
    targetPos.x += dx
    targetPos.z -= dz  
    pathList.append(list(targetPos.toVector()))

    self.motionProxy.positionInterpolations(effectorList, frame, pathList, axisMaskList, timeList)
    
    # End the game
    self.end = True 
    self.Point_sub.unregister()
    
    
  def callback_camera(self, ros_data):
  
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
	  	  
      # Conversion of the OpenCV format image to ROS format and publish it in a topic
      ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
      self.publisher.publish(ros_image)

      # It calculates the movement of the head necessary to keep the ball in the center of the image
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
	  
      # Check the position of the head before moving it to restrict movement and prevent it from clashing with the shoulders

      # Subscribe to the topic / joint_states and save the first two values ​​(HeadYaw and HeadPitch) in reference variables to compare before performing the move

      # Determine the new relative position in Y, which is the one I restrict to avoid collisions
      new_position_y=self.positions[1]+var_y*0.10
	  
      if (new_position_y <= -0.45 ): 
	var_y=0
      elif (new_position_y >= 0.3):
	var_y=0
	    
      joint.joint_angles.append(-var_x*0.25) 
      joint.joint_angles.append(var_y*0.10) 
	
      self.joint_pub.publish(joint)
      
      # Movement towards the ball (walking)
      walk=Twist()
      
     
      print " El radio son (px): ", radius
      print "HeadPitch: ", self.positions[1]
      print "HeadYaw: ", self.positions[0] # HeadYaw <-->
      if radius < 58: # Approach if far away. If I see radio 50 I am approx. 25cm
	
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
	
	
      elif radius > 68: # When Nao is too close to the ball, it walks backwards
	walk.linear.x=-0.2
	walk.linear.y=0.0  	  
	walk.linear.z=0.0
	
	walk.angular.x=0.0
	walk.angular.y=0.0
	walk.angular.z=0.0
	
	self.walk_pub.publish(walk)
	print " Desplazo atras"
      
      else:

	CabezaCentrada=False
	PelotaCentro=False
	# Check if the ball is in front of the robot, otherwise it moves to the corresponding side
	# Check that the head is aligned with the body and that it is not turned to any side
	if self.positions[0] < (-0.3) or self.positions[0] > 0.3:
	  walk.linear.y=0.0
	  e=0-self.positions[0]
	  walk.angular.z=e*(-0.5)

	elif self.positions[0] <  (-0.05): # HeadYaw <-->
	  walk.linear.y=-0.3
	  walk.angular.z=-0.0
	  
	elif self.positions[0] > 0.05:
	  walk.linear.y=0.3
	  walk.angular.z=0.0

	else: # Stop walking
	  CabezaCentrada=True
	  
	  walk.linear.y=0.0
	  walk.angular.z=0.0
	  print " Cabeza centrada"
	
	  walk.linear.x=0.0  	  
	  walk.linear.z=0.0
	
	  walk.angular.x=0.0
	  walk.angular.y=0.0
	
	  self.walk_pub.publish(walk)
	  
	  rospy.sleep(0.3) # For the robot to stabilize

	  if center[0]< 150 or center [0]> 170: # It is verified that the ball is seen in the center of the image through its X coordinate
	    e=160-center[0]
	    walk.linear.y=e*(-0.005)
	    
	    print "Regulador"
	    walk.angular.z=-0.0
	  else:	# Parar
	    PelotaCentro=True
	    walk.linear.y=0.0
	    walk.angular.z=0.0
	    print " Pelota en el centro"
	
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

	  # Subscribe to other callback
	  self.image_sub = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, self.callback_camera2)

  def callback_camera2(self, ros_data):
    print "callback camera 2: centrado de pelota en la imagen"
    

    # Use the bridge to convert the ROS Image message to OpenCV message
    cv_image = self.bridge.imgmsg_to_cv2(ros_data, desired_encoding="passthrough")
	  
    # Convert the cv_image to the HSV color space
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) 

    # Construct a mask for the color "colour", then perform a series of dilations and erosions to remove any small blobs left in the mask
    mask = cv2.inRange(hsv, self.colourLower, self.colourUpper)
    mask = cv2.erode(mask, None, iterations=2) 
    mask = cv2.dilate(mask, None, iterations=2) 

    # Find contours in the mask and initialize the current (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
      cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    # Only proceed if at least one contour was found
    if len(cnts) > 0:
    # Find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
      c = max(cnts, key=cv2.contourArea)
      ((x, y), radius) = cv2.minEnclosingCircle(c)
      M = cv2.moments(c)
      center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	
      # Only proceed if the radius meets a minimum size
      if radius > 10:
	# Draw the circle and centroid on the cv_image
	cv2.circle(cv_image, (int(x), int(y)), int(radius),(0, 255, 255), 2)
	cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
	  	  
    # Conversion of the OpenCV format image to ROS format and publish it in a topic
    ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
    self.publisher.publish(ros_image)

    if center[0]<155 or center[0]>165: # Check X coordinate
      self.contador=0
      # Calculate the movement of the head necessary to keep the ball in the center of the image
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
	  
      # Check the position of the head before moving it to restrict movement and prevent it from clashing with the shoulders

      # Subscribe to the topic / joint_states and save the first two values ​​(HeadYaw and HeadPitch) in reference variables to compare before performing the move

      # Determine the new relative position in Y, which is the one I restrict to avoid collisions
      new_position_y=self.positions[1]+var_y*0.15

      if (new_position_y <= -0.45 ): 
	var_y=0
      elif (new_position_y >= 0.3):
	var_y=0
	    
      joint.joint_angles.append(-var_x*0.10) 
      joint.joint_angles.append(var_y*0.15) 
	
      self.joint_pub.publish(joint)
    else:
      print "Centrado respecto a X"
      if center[1]<115 or center[1]>125: # Check Y coordinate
	self.contador=0
	# Velocities message initialization
	joint = JointAnglesWithSpeed()
	joint.joint_names.append("HeadPitch")
	joint.speed = 0.1
	joint.relative = True
	  
	if center[1]<115:
	  var_y=-0.01
	elif center[1]>125:
	  var_y=0.01
	  
	joint.joint_angles.append(var_y)
	
	self.joint_pub.publish(joint)
      else:
	# Counter to ensure that the ball is centered and it was not a coincidence
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
	  Dreal=0.08 # Real diameter of the ball in meters
	  pixelToMeter=Dreal/(radius*2) 
      
	  Yrobot=(cv_image.shape[1]/2 - center[0])*pixelToMeter
	
	  radiusToMeters=0.25*55 # At 0.25m distance the radius is 55px
	  Xrobot=radiusToMeters/radius
	  
	  # Calculate Z coordinate according to the equation of the line that relates it to HeadPitch
	  Zrobot=(-25.866*self.positions[1]+23.843)/100 # Divide by 100 to move from cm to m
	  
	  # Create point
	  point = Point()
	  point.x = Xrobot
	  point.y = Yrobot
	  point.z = Zrobot
	
	  self.pointPub.publish(point)
	  print "Punto publicado: ", point
	  self.image_sub.unregister()
    

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

  
  
  def main(self):  
 
    ## Configuration parameters
    # Set the language of the speech recognition engine to Spanish:
    self.asrProxy.setLanguage("Spanish")
    # set the language of the synthesis engine to Spanish:
    self.ttsProxy.setLanguage("Spanish")

    # Adds the vocabulary
    vocabulary = ["si", "no", "hola", "Nao", "rojo", "azul", "amarillo", "violeta"]
    self.asrProxy.setVocabulary(vocabulary, False)
    # Or, if you want to enable word spotting:
    #self.asrProxy.setVocabulary(vocabulary, True)
 
    # Wake up robot
    self.motionProxy.wakeUp()

    fractionMaxSpeed = 0.5
    # Go to posture stand
    self.postureProxy.goToPosture("StandInit", fractionMaxSpeed)
      
    # Introducting the game
    self.ttsProxy.say("Hola! Soy Nao.") 
 
    # Ask the kid to choose one ball (blue, red, yellow)
    self.ttsProxy.say("Elige una de las pelotas y cuando estés listo llámame.")
    
    # Start sound tracker while the kid is not ready. When the kid says something or claps his hands, Nao turn into his location.
    targetName = "Sound"
    distance = 1
    confidence = 0.4
    parameters=[distance,confidence]
    self.trackerProxy.registerTarget(targetName, parameters)
    
    mode = "Move"
    self.trackerProxy.setMode(mode)
    # Start tracker.
    self.trackerProxy.track(targetName)
    
    while True:
	time.sleep(1)
	TargetPosition=self.trackerProxy.getTargetPosition(2) # Frame: 0-Torso, 1-World, 2-Robot
	print "Target Position: ", TargetPosition
	print len(TargetPosition)
	if len(TargetPosition)>0:
	  if TargetPosition[0]>0 and abs(TargetPosition[1])<0.1: # Exit from the loop when the kid voice is opposite the robot
	    break
    
    self.trackerProxy.stopTracker()
    self.trackerProxy .unregisterAllTargets()
    self.ttsProxy.say("¡Al fin te encuentro!")
   
    # Go to posture stand
    self.postureProxy.goToPosture("StandInit", fractionMaxSpeed)
   
    self.ttsProxy.say("¿Me dejas ver la pelota?")
    
    repeate_recognition=1
    count=0
    while (repeate_recognition==1):
      self.postureProxy.goToPosture("StandInit", fractionMaxSpeed)
      # Robot posture with hands opened to catch the ball
      #joint_pub = rospy.Publisher('/joint_angles', JointAnglesWithSpeed, queue_size=10)
      rospy.loginfo("Poniendo manos en posición de reconocimiento")
      joint = JointAnglesWithSpeed()
      joint.joint_names.append("LShoulderRoll")
      joint.joint_angles.append(-0.35) 
      joint.joint_names.append("LShoulderPitch")
      joint.joint_angles.append(-0.30) 
      joint.joint_names.append("LElbowYaw")
      joint.joint_angles.append(-1.00) 
      joint.joint_names.append("LElbowRoll")
      joint.joint_angles.append(0.00) 
      joint.joint_names.append("LWristYaw")
      joint.joint_angles.append(-1.50) 
      joint.joint_names.append("LHand")
      joint.joint_angles.append(1)
          
      joint.joint_names.append("RShoulderRoll")
      joint.joint_angles.append(0.35) 
      joint.joint_names.append("RShoulderPitch")
      joint.joint_angles.append(-0.30) 
      joint.joint_names.append("RElbowYaw")
      joint.joint_angles.append(1.00) 
      joint.joint_names.append("RElbowRoll")
      joint.joint_angles.append(0.00) 
      joint.joint_names.append("RWristYaw")
      joint.joint_angles.append(1.50) 
      joint.joint_names.append("RHand")
      joint.joint_angles.append(1)
      
      joint.joint_names.append("HeadYaw")
      joint.joint_angles.append(0.00)
      joint.joint_names.append("HeadPitch")
      joint.joint_angles.append(0.00)

      joint.speed = 0.1
      joint.relative = False

      self.joint_pub.publish(joint)
	
      # Wait some time for the kid to put the ball in Nao's hands
      time.sleep(10) # 10 seconds
      
      # Take a frame with the ball to recognize the colour 
      ball_detected = 0
      while (ball_detected == 0):
	output = self.cv_image.copy() # Draw the detected circles without destroying the original image.
	gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY) # The cv2.HoughCircles function requires an 8-bit, single channel image, so we'll go ahead and convert from the RGB color space to grayscale

	# Detect circles in the image
	circles = cv2.HoughCircles(gray, cv2.cv.CV_HOUGH_GRADIENT, 1.2, 100, param2=80,minRadius=50,maxRadius=200)
      
	if circles == None:
	  print 'No se ha detectado la pelota'
	  self.ttsProxy.say("No soy capaz de reconocer la pelota")
	  time.sleep(5)
	else:
	  print 'Se ha detectado la pelota: ', circles
	  ball_detected = 1
	  # Ensure at least some circles were found
	
	if circles is not None:
	# Convert the (x, y) coordinates and radius of the circles to integers allowing us to draw them on our output image
	  circles = np.round(circles[0, :]).astype("int")
 
	  # Loop over the (x, y) coordinates and radius of the circles
	  for (x, y, r) in circles:
	    # Draw the circle in the output image, then draw a rectangle
	    # Corresponding to the center of the circle
	    cv2.circle(output, (x, y), r, (0, 255, 0), 4)
	    cv2.rectangle(output, (x - 45, y - 45), (x + 45, y + 45), (0, 128, 255), -1)
	    ###
	
	  hsv=cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
	
	  # Calculate the mean of the HSV values ​​within a rectangle (side 45) close to the center of the detected ball
	  s=[0,0,0]
	  for i in range (y-45,y+46):
	    for j in range (x-45,x+46):
	      s=s+hsv[i,j]
	      #print j,i, hsv[i,j]
	  s=s/((45+46)*(45+46))
	  print s
	
	  # I calculate the upper and lower limits from the mean HSV
	  if s[1] < 100: # Saturation < 100
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
	
	  print "Rango de colores. Limites superior e inferior: ", self.colourUpper, self.colourLower
	  # Compare the colour range with the ones in the data base and say to the kid what colour is it
	  if s[0]<20: # Red
	    colour="rojo"
	  elif s[0]>90 and s[0]<110: # Blue
	    colour="azul"
	  elif s[0]>=20 and s[0]<=40: # Yellow
	    colour="amarillo"
	  elif s[0]>=130 and s[0]<=150:# Violet
	    colour="violeta"
	  else:	# The colour is not  in the data base
	    colour="no"
      
	  self.ttsProxy.say("Ya la puedes coger")

	  # Wait 
	  time.sleep(5) # 5 seconds
      
	  # Go to posture stand
	  self.postureProxy.goToPosture("StandInit", fractionMaxSpeed)

	  if colour=="no":
	    self.ttsProxy.say("No he visto bien el color")
	    self.ttsProxy.say("¿Me dejas ver de nuevo la pelota?")
	    repeate_recognition=1
	  else:
	
	    self.ttsProxy.say("¿Quieres que siga una pelota de color" + colour + "¿Es eso cierto?")
	  	
	    respuesta=0
	    while(respuesta==0):
	      # Start the speech recognition engine with user Test_ASR
	      self.asrProxy.subscribe("Test_ASR")
	      self.subscribed = True
	      print 'Speech recognition engine started'
	      time.sleep(5)
	      WordHeard = self.Memory.getData("WordRecognized")[0]
	      # Stop the speech recognition engine with user Test_ASR
	      self.asrProxy.unsubscribe("Test_ASR")
	      self.subscribed = False
	      print "contador: " ,count
	      if WordHeard == "si":
		respuesta = 1
		self.ttsProxy.say("¡Genial! Comencemos a jugar")
		repeate_recognition=0
	      elif WordHeard == "no":
		respuesta = 1
	      
		count += 1
		if count > 2:
		  self.ttsProxy.say("¡Ai ai ai! ¿De qué color es entonces?")
		  repeate_recognition=0
		
		  respuesta2=0
		  while(respuesta2==0):
		    repeate_recognition=0
		    # Start the speech recognition engine with user Test_ASR
		    self.asrProxy.subscribe("Test_ASR")
		    self.subscribed = True
		    print 'Speech recognition engine started'
		    time.sleep(5)
		    WordHeard = self.Memory.getData("WordRecognized")[0]
		    # Stop the speech recognition engine with user Test_ASR
		    self.asrProxy.unsubscribe("Test_ASR")
		    self.subscribed = False
		    if WordHeard == "rojo":
		      respuesta2 = 1
		      colour="rojo"
		      self.colourLower=(0, 100, 100)
		      self.colourUpper=(20, 255, 255)
		      self.ttsProxy.say("Muy bien, seguiré una pelota de color" + colour)
		    elif WordHeard == "azul":
		      respuesta2 = 1
		      colour="azul"
		      self.colourLower=(90, 100, 100)
		      self.colourUpper=(110, 255, 255)
		      self.ttsProxy.say("Muy bien, seguiré una pelota de color" + colour)
		    elif WordHeard == "violeta":
		      respuesta2 = 1
		      colour="violeta"
		      self.colourLower=(135, 40, 40)
		      self.colourUpper=(155, 255, 255)
		      self.ttsProxy.say("Muy bien, seguiré una pelota de color" + colour)
		    elif WordHeard == "amarillo":
		      respuesta2 = 1
		      colour="amarillo"
		      self.colourLower=(20, 100, 100)
		      self.colourUpper=(40, 255, 255)
		      self.ttsProxy.say("Muy bien, seguiré una pelota de color" + colour)
		    else:
		      self.ttsProxy.say("Disculpa, tendrás que volver a repetírmelo")
		
		else:
		  self.ttsProxy.say("¡Ups! Entonces tendré que volver a ver la pelota")
		  repeate_recognition=1
			
	      else:
		self.ttsProxy.say("Perdona, no te he entendido bien. ¿Puedes repetirme tu respuesta?")	    
	  
    
    self.postureProxy.goToPosture("StandInit", fractionMaxSpeed)
    
    print "Tracker activado"
    self.ttsProxy.say("Ahora muévete y enséñame la pelota, que ya voy a por ella")
    
    self.ttsProxy.say("Cuando quieras que la coja párate")
    self.tracking=1
	
    while self.end is not True:
      time.sleep(1)
	
    self.postureProxy.goToPosture("Stand", fractionMaxSpeed)
    self.ttsProxy.say("¿Quieres volver a jugar?")
    
    
    respuesta3 = 0
    while (respuesta3 == 0):
      # Start the speech recognition engine with user Test_ASR
      self.asrProxy.subscribe("Test_ASR")
      self.subscribed = True
      print 'Speech recognition engine started'
      time.sleep(5)
      WordHeard = self.Memory.getData("WordRecognized")[0]
      # Stop the speech recognition engine with user Test_ASR
      self.asrProxy.unsubscribe("Test_ASR")
      self.subscribed = False
      if WordHeard == "si":
	respuesta3 = 1
	self.ttsProxy.say("¡Genial! Volvamos a jugar con una pelota de otro color")
	self.play_again = True
      elif WordHeard == "no":
	respuesta3 = 1
	self.ttsProxy.say("Joo, ¡Qué pena!")
	self.play_again = False
	raise KeyboardInterrupt
      else:
	self.ttsProxy.say("Perdona, no te he entendido bien. ¿Puedes repetirme tu respuesta?")	
	
   
  def salir(self):
    
    self.trackerProxy.stopTracker()
    self.trackerProxy .unregisterAllTargets()
    print "ALTracker stopped."
    # Go to rest position
    self.motionProxy.rest()
    if self.subscribed:
      self.asrProxy.unsubscribe("Test_ASR")
	
if __name__ == "__main__":
      
  try:
    while True:
      ic = tracker()
      rospy.init_node('ejemplo_final', anonymous=True)
      ic.main()
	  
  except KeyboardInterrupt:
    print "Interrupted by user"
    print "Stopping..."
    ic.salir()