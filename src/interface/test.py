#!/usr/bin/env python
import sys
from PyQt5 import QtGui, QtCore, QtWidgets, uic
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Joy
from interface import MyWindow

PATH = "/home/nathan/ROS_bluerov2_ws/src/bluerov_ros_playground/bluerov_ros/src/interface/"

class Display():

    def __init__(self):
        
        rospy.Subscriber('/BlueRov2/arm', Bool, self._arm_callback) 
        rospy.Subscriber('/BlueRov2/battery', BatteryState, self._battery_callback)

        rospy.Subscriber('/Command/depth', UInt16, self._callback_depth)
        rospy.Subscriber('/Command/heading', UInt16, self._callback_heading)
        rospy.Subscriber('/Command/velocity', UInt16, self._callback_velocity)
        rospy.Subscriber('/Command/joy', Joy, self._callback_joy)
        self.arm = False    
        self.battery = None

        self.pwm_depth = 1200
        self.pwm_forward = None
        self.pwm_heading = None

        self.gamepad_axes = None
        self.gamepad_buttons = None
        self.override_controller = None

    def _arm_callback(self,msg):
        self.arm = msg.data
    def _battery_callback(self, msg):
        self.battery = msg.voltage
     
    def _callback_depth(self,msg):
        self.pwm_depth = msg.data
        print(self.pwm_depth)
    def _callback_heading(self,msg):
        self.pwm_heading = msg.data

    def _callback_velocity(self,msg):
        self.pwm_forward = msg.data

    def _callback_joy(self,msg):
        self.gamepad_axes = msg.axes
        self.gamepad_buttons = msg.buttons
        self.override_controller = self.gamepad_buttons[1]
           
    
    def display(self):
        print("display")
        #STATUS section :
        if self.arm:
            self.arm_disarm_display.setText('ARM')
        else:
            self.arm_disarm_display.setText('DISARM')

        if self.override_controller:
            self.automatic_manual_ctrl_display.setText('MANUAL')
        else:
            self.automatic_manual_ctrl_display.setText('AUTOMATIC')

        self.battery_level_display.setText('{} V'.format(self.battery))

	#self.light_level_display 
        
        #CONTROLLER OVERVIEW :
        self.depth_measured_display.setText('TEST')
        self.depth_controller_pwm_display.setText('{}'.format(self.pwm_depth))
        self.depth_controller_KI_display
        self.depth_controller_KP_display
        self.depth_controller_KD_display

        self.heading_measured_display
        self.heading_controller_pwm_display.setText('{}'.format(self.pwm_heading))
        self.heading_controller_KI_display
        self.heading_controller_KP_display
        self.heading_controller_KD_display

        self.velocity_measured_display
        self.velocity_controller_pwm_display.setText('{}'.format(self.pwm_forward))
        self.velocity_controller_KI_display
        self.velocity_controller_KP_display
        self.velocity_controller_KD_display


if __name__ == "__main__":
    window = Display()

