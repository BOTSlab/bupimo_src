#!/usr/bin/env python
"""
"""

from __future__ import division
import rospy
from std_msgs.msg import Bool

import time

from Adafruit_PWM_Servo_Driver import PWM

# Initialise with the default address (0x40).
pwm = PWM(0x40)

# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096

# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.setPWM(channel, 0, pulse)

def callback(data):
    if data.data == 0:
        pwm.setPWM(0, 0, servo_min+50)
    if data.data == 1:
        pwm.setPWM(0, 0, servo_max)

def main():
    # Set frequency to 60hz.
    pwm.setPWMFreq(60)

    rospy.init_node('servo_control')
    rospy.Subscriber("servo_control", Bool, callback);
    rospy.spin()

if __name__ == '__main__':
    main()
