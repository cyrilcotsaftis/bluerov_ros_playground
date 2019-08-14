#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from bluerov_ros_playground.msg import Attitude
from bluerov_ros_playground.msg import Set_heading
from bluerov_ros_playground.msg import Set_target

PI = np.pi

class Heading_Controller():
    def __init__(self, heading_desired=0, KP=35, KD=25, pwm_max=1550, pwm_neutral=1500,rosrate=4):
        #pub_rc4 = rospy.Publisher('/BlueRov2/rc_channel4/set_pwm', UInt16, queue_size=10)
        #pub_arm = rospy.Publisher('/BlueRov2/arm', Bool, queue_size=10)
        self.pub_pwm = rospy.Publisher('/Command/heading', UInt16, queue_size=10)

        rospy.Subscriber('/BlueRov2/imu/attitude', Attitude, self._callback_att)

        self.rate = rospy.Rate(rosrate)
        self.attitude = [0, 0, 0, 0, 0, 0] #[ROLL, PITCH, YAW, ROLLSPEED, PITCHSPEED, YAWSPEED]
        self.pwm_max = pwm_max
        self.pwm_neutral = pwm_neutral
        self.heading_desired = heading_desired
        self.KP = KP
        self.KD = KD

        rospy.Subscriber('/Settings/set_heading', Set_heading, self._callback_set_heading)
        rospy.Subscriber('/Settings/set_target', Set_target, self._callback_set_target)
        
    def sawtooth (self, x):
        """to deal with 2*PI modulo"""
        return (x+PI)%(2*PI)-PI         

    def _callback_att(self, msg):
        self.attitude = [msg.roll,
                         msg.pitch,
                         msg.yaw,
                         msg.rollspeed,
                         msg.pitchspeed,
                         msg.yawspeed]
                   
    def _callback_set_heading(self, msg):
        if msg.pwm_max < 1500:
            self.pwm_max = 1500
        else:
            self.pwm_max = msg.pwm_max
        self.KP = msg.KP 
        self.KD = msg.KD 

    def _callback_set_target(self, msg):
        self.heading_desired = msg.heading_desired

    def control(self, yaw, yawspeed):
        return self.KP*self.sawtooth(yaw-self.heading_desired) + self.KD*yawspeed
    
    def saturation(self, pwm):
        pwm_min = self.pwm_neutral - (self.pwm_max - self.pwm_neutral)
        if pwm > self.pwm_max :
            pwm = self.pwm_max
        if pwm < pwm_min:
            pwm = pwm_min
        return int(pwm)

    def main(self):
        #pub_arm.publish(1)
        yaw = self.attitude[2]
        yawspeed = self.attitude[5]
        u = self.control(yaw, yawspeed)
        pwm = self.pwm_neutral - u
        pwm = self.saturation(pwm)
        print("HEADING_DESIRED : {}, YAW_MESURED : {}, PWM : {}".format(self.heading_desired, yaw, pwm))
        #pub_rc4.publish(pwm)
        self.pub_pwm.publish(pwm)


if __name__ == "__main__":
    rospy.init_node('heading_controller', anonymous=True)
    heading_controller = Heading_Controller()
    while not rospy.is_shutdown():
        heading_controller.main()
        heading_controller.rate.sleep()
