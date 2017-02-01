# -*- encoding: UTF-8 -*-

import time

from naoqi import ALProxy

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
    
    self.subscribed = False
  
  def main(self):

      ## Configuration parameters
      option=0
      modo=0
      # Set the language of the speech recognition engine to Spanish:
      self.asrProxy.setLanguage("Spanish")
      # set the language of the synthesis engine to Spanish:
      self.ttsProxy.setLanguage("Spanish")
      
      
      # Adds the vocabulary
      vocabulary = ["si", "no", "hola", "Nao", "pelota", "cara", "sonido", "cabeza", "cuerpo"]
      self.asrProxy.setVocabulary(vocabulary, False)
      # Or, if you want to enable word spotting:
      #self.asrProxy.setVocabulary(vocabulary, True)
  
      # Wake up robot
      self.motionProxy.wakeUp()

      fractionMaxSpeed = 0.5
      # Go to posture stand
      self.postureProxy.goToPosture("StandInit", fractionMaxSpeed)
      
      # Introducting the game
      self.ttsProxy.say("Hola!")
            
      # Ask/Choose tracking target
    
      while (option == 0):
	self.ttsProxy.say("Dime qué quieres que siga: una pelota, una cara o un sonido")
      
	# Start the speech recognition engine with user Test_ASR
	self.asrProxy.subscribe("Test_ASR")
	print 'Speech recognition engine started'
	time.sleep(5)
	self.subscribed = True
	WordHeard = self.Memory.getData("WordRecognized")[0]
	# Stop the speech recognition engine with user Test_ASR
	self.asrProxy.unsubscribe("Test_ASR")
	self.subscribed = False
      
	if WordHeard == "sonido":
	  option = 1
	  self.ttsProxy.say("Vale, seguiré un sonido")
	elif WordHeard == "pelota":
	  option=2
	  self.ttsProxy.say("Vale, seguiré una pelota roja. ¡Es mi color favorito!")
	elif WordHeard == "cara":
	  option=3
	  self.ttsProxy.say("Vale, seguiré una cara")
	else:
	  self.ttsProxy.say("Perdona, no te he entendido bien")

      # Ask/choose tracking mode
      while (modo == 0):
	self.ttsProxy.say("Ahora dime cómo quieres que juegue: con la cabeza o con el cuerpo")

	# Start the speech recognition engine with user Test_ASR
	self.asrProxy.subscribe("Test_ASR")
	self.subscribed = True
	print 'Speech recognition engine started'
	time.sleep(5)
	WordHeard = self.Memory.getData("WordRecognized")[0]
	# Stop the speech recognition engine with user Test_ASR
	self.asrProxy.unsubscribe("Test_ASR")
	self.subscribed = False
      
	if WordHeard == "cabeza":
	  modo = 2
	  self.ttsProxy.say("Moveré mi cabeza para jugar")
	elif WordHeard == "cuerpo":
	  modo = 1
	  self.ttsProxy.say("Moveré mi cuerpo para jugar")
	else:
	  self.ttsProxy.say("Perdona, no te he entendido bien")
	
	
      # Tracker configuration

      # Add target to track. 				
      #	Target Name		|	Parameters
      #	------------------------|---------------------------------
      #1.	Sound		|	[distance,confidence]
      #2.	RedBall		|	Diameter of ball (meters)
      #3.	Face		|	Width of face (meters)
      
      print 'Tracking configuration'
      if option == 1:
	targetName = "Sound"
	distance = 1
	confidence = 0.4
	parameters=[distance,confidence]
      elif option == 2:
	targetName= "RedBall"
	diameter = 0.08 # 8cm
	parameters=diameter
      else: #option==3
	targetName="Face"
	Width = 0.18 #15cm
	parameters=Width
	
      self.trackerProxy.registerTarget(targetName, parameters)
      
      # set mode
      if modo == 1:
	mode = "Move" # WholeBody, Head, Move
      else: #modo==2
	mode = "Head"
      
      self.trackerProxy.setMode(mode)

      # Go to posture stand
      self.postureProxy.goToPosture("StandInit", fractionMaxSpeed)
      # Then, start tracker.
      self.trackerProxy.track(targetName)
      
      self.ttsProxy.say("¡Allá vamos!")
      print "ALTracker successfully started"
      print "Use Ctrl+c to stop this script."

      while True:
	time.sleep(1)
    
  def salir(self):
    
    self.trackerProxy.stopTracker()
    self.trackerProxy .unregisterAllTargets()
    print "ALTracker stopped."
    # Go to rest position
    self.motionProxy.rest()
    if self.subscribed:
      self.asrProxy.unsubscribe("Test_ASR")
	
if __name__ == "__main__":
    
    ic = tracker()
    try:
        ic.main()
    except KeyboardInterrupt:
        print "Interrupted by user"
        print "Stopping..."
        ic.salir()
