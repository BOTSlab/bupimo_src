#!/usr/bin/env python
'''
Adapted for the Zumo 32U4 robot by Nicholi Shiell and Andrew Vardy.
Memorial University

---

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

import rospy
import sys
import tf

from geometry_msgs.msg import Twist
from bupimo_msgs.msg import ZumoData
from std_msgs.msg import String
from nav_msgs.msg import Odometry


from SerialDataGateway import SerialDataGateway

class zumo_comms_node(object):
	'''
	Helper class for communicating with the Zumo 32U4 over serial
	'''

	def ReceivedLine(self,  line):
            #print("LINE: " + line)
            words = line.split(" ")
            #print("words: ")
            #print(words)
            if len(line) > 0:
                zumo_msg = ZumoData()
                try:
                    zumo_msg.x = float(words[0])
                    zumo_msg.y = float(words[1])
                    zumo_msg.theta = float(words[2])
                    zumo_msg.linearSpeed = float(words[3])
                    zumo_msg.angularSpeed = float(words[4])
                    zumo_msg.compassHeading = float(words[5])
                    zumo_msg.frontProximity = int(words[6])
                except: # Catch all 
                    exception = sys.exc_info()[0]
                    print("Exception in zumo_comms_node: " + str(exception))

                self.zumoPublisher.publish(zumo_msg)

                self.tf_and_odom(zumo_msg)

        def tf_and_odom(zumo_msg):
            """Broadcast tf transform and publish odometry.  It seems a little
            redundant to do both, but both are needed if we want to use the ROS
            navigation stack."""
    
            trans = (zumo_msg.x, zumo_msg.y, 0)
            rot = tf.transformations.quaternion_from_euler(0, 0, zumo_msg.theta)

            self.broadcaster.sendTransform(trans, rot, \
               rospy.Time.now(), \
               "base_link", \
               "odom")

            odom = Odometry()
            odom.header.frame_id = 'odom'
            odom.header.stamp = rospy.Time.now()
            odom.pose.pose.position = trans
            odom.pose.pose.orientation = rot
            odom.child_frame_id = 'base_link'
            odom.twist.twist = stored_twist
		
        def TwistCallback(self, twist):
            v = twistMessage.linear.x   # Forward speed
            w = twistMessage.angular.z  # Angular speed

            # Send to Zumo
            message = str(v) + ':' + str(w) + ':0'
	    self.SerialDataGateway.Write(message)

            self.stored_twist = twist
		

	def __init__(self, port="/dev/serial0", baudrate=57600):
		'''
		Initializes the receiver class. 
		port: The serial port to listen to.
		baudrate: Baud rate for the serial communication
		'''

		rospy.init_node('zumo_comms_node')

		port = rospy.get_param("~port", "/dev/serial0")
		baudRate = int(rospy.get_param("~baudRate", 57600))

		rospy.loginfo("Starting with serial port: " + port + \
                            ", baud rate: " + str(baudRate))

		# Subscriptions
		rospy.Subscriber("cmd_vel", Twist, self.TwistCallback)
    
                # Publishers
		self.zumoPublisher = rospy.Publisher('zumo_data', ZumoData, \
                                                    queue_size=1)
                self.odomPublisher = rospy.Publisher('odom', Odometry, \
                                                     queue_size=1)

                # Creat TF broadcaster
                self.broadcaster = tf.TransformBroadcaster()

		# CREATE A SERIAL_DATA_GATEWAY OBJECT 
		# pass it a function pointer to _HandleReceivedLine
		self.SerialDataGateway = SerialDataGateway(port, baudRate, \
                                                            self.ReceivedLine)

                self.stored_twist = Twist()

	def Start(self):
		#rospy.logdebug("Starting")
		self.SerialDataGateway.Start()

	def Stop(self):
		#rospy.logdebug("Stopping")
		self.SerialDataGateway.Stop()
		

if __name__ == '__main__':
	zumo_comms_node = zumo_comms_node()
	try:
		zumo_comms_node.Start()
		rospy.spin()

	except rospy.ROSInterruptException:
		zumo_comms_node.Stop()

