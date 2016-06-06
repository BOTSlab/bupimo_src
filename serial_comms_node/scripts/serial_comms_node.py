#!/usr/bin/env python
'''
Created January, 2011

@author: Dr. Rainer Hessmer

  arduino.py - gateway to Arduino based differential drive base
  Copyright (c) 2011 Dr. Rainer Hessmer.  All right reserved.
  Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# AV: Not needed
# import roslib
import rospy
import sys

from geometry_msgs.msg import Twist
from bupimo_msgs.msg import ZumoProximities
from std_msgs.msg import String

from SerialDataGateway import SerialDataGateway

class serial_comms_node(object):
	'''
	Helper class for communicating with an Arduino board over serial port
	'''

        # Half the baseline distance between the Zumo's wheels (in metres)
        HALF_BASELINE = 0.085

        # Radius of the Zumo's wheels (including treads --- in metres)
        WHEEL_RADIUS = 0.019

        # Arbitrarily set conversion factor from wheel speeds in rads/sec to
        # the Zumo library's input speeds in the range of [-400, 400].
        ZUMO_SPEED_CONVERSION = 7.6

	def ReceivedLine(self,  line):
            #print("LINE: " + line)
            words = line.split(" ")
            #print("words: ")
            #print(words)
            if len(line) > 0:
                # Its a proximities message.
                prox_msg = ZumoProximities()
                try:
                    prox_msg.front = int(words[0])
                    prox_msg.left = int(words[1])
                    prox_msg.right = int(words[2])
                except: # Catch all 
                    exception = sys.exc_info()[0]
                    print("Exception in serial_comms_node: " + str(exception))

                self.proxPublisher.publish(prox_msg)
		
        # AV: Switching to controlling the robot via a Twist message
	#def TransmitWheelSpeeds(self, wheelSpeeds):
	#	#message = "w " + str(wheelSpeeds) +"\n"
	#	message = str(wheelSpeeds.data)
	#	self.SerialDataGateway.Write(message)

        # AV: The following callback will use the inverse kinematics for a
        # differential-drive robot (i.e. the Zumo) to determine the desired
        # wheel speeds which are sent to the Zumo as a string in the form
        # "LEFT_SPEED RIGHT_SPEED".  Note that both speeds should be in the
        # range [-400, 400].  So we will apply those caps here as well.
        def TwistCallback(self, twistMessage):
            v = twistMessage.linear.x   # Forward speed
            w = twistMessage.angular.z  # Angular speed

            # The following should be in rads / sec
            leftSpeedRads = (v - self.HALF_BASELINE * w) / self.WHEEL_RADIUS
            rightSpeedRads = (v + self.HALF_BASELINE * w) / self.WHEEL_RADIUS

            # Convert to commanded speeds
            leftSpeed = int(self.ZUMO_SPEED_CONVERSION * leftSpeedRads)
            rightSpeed = int(self.ZUMO_SPEED_CONVERSION * rightSpeedRads)

            #print('leftSpeed: ' + str(leftSpeed))
            #print('rightSpeed: ' + str(rightSpeed))
            
            # Cap at +/-400
            if leftSpeed > 400: leftSpeed = 400
            if leftSpeed < -400: leftSpeed = -400
            if rightSpeed > 400: rightSpeed = 400
            if rightSpeed < -400: rightSpeed = -400

            # Send to Zumo
            message = str(leftSpeed) + ' ' + str(rightSpeed)
	    self.SerialDataGateway.Write(message)
		

        # AV: Corrected baudrate, although it shouldn't matter
	# def __init__(self, port="/dev/serial0", baudrate=56700):
	def __init__(self, port="/dev/serial0", baudrate=57600):
		'''
		Initializes the receiver class. 
		port: The serial port to listen to.
		baudrate: Baud rate for the serial communication
		'''

		rospy.init_node('serial_comms_node')

		port = rospy.get_param("~port", "/dev/serial0")
		baudRate = int(rospy.get_param("~baudRate", 57600))

		rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baudRate))

		# subscriptions
		# rospy.Subscriber("wheelSpeeds", String, self.TransmitWheelSpeeds)
		rospy.Subscriber("cmd_vel", Twist, self.TwistCallback)
    
		self.proxPublisher = rospy.Publisher('zumo_prox', ZumoProximities, queue_size = 1)

		# CREATE A SERIAL_DATA_GATEWAY OBJECT 
		# pass it a function pointer to _HandleReceivedLine
		self.SerialDataGateway = SerialDataGateway(port, baudRate,  self.ReceivedLine)

	def Start(self):
		#rospy.logdebug("Starting")
		self.SerialDataGateway.Start()

	def Stop(self):
		#rospy.logdebug("Stopping")
		self.SerialDataGateway.Stop()
		

if __name__ == '__main__':
	serial_comms_node = serial_comms_node()
	try:
		serial_comms_node.Start()
		rospy.spin()

	except rospy.ROSInterruptException:
		serial_comms_node.Stop()

