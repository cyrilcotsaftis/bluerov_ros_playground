#!/usr/bin/env python
import sys
from PyQt5 import QtGui, QtCore, QtWidgets, uic
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Joy
from bluerov_ros_playground.msg import Set_velocity 
from bluerov_ros_playground.msg import Set_heading 
from bluerov_ros_playground.msg import Set_depth
from bluerov_ros_playground.msg import Set_attitude

PATH = "/home/nathan/ROS_bluerov2_ws/src/bluerov_ros_playground/bluerov_ros/src/interface/"

class Display(QtWidgets.QMainWindow):

    def _depth_param_clicked(self):
        self.depth_ctrl_msgToSend.pwm_max = self.spinBox_pwm_max.value()
        self.depth_ctrl_msgToSend.KI = self.spinBox_KI_depth.value()
        self.depth_ctrl_msgToSend.KP = self.spinBox_KP_depth.value()
        self.depth_ctrl_msgToSend.KD = self.spinBox_KD_depth.value()
        self.pub_set_depth.publish(self.depth_ctrl_msgToSend)

        
    def _heading_param_clicked(self):
        self.heading_ctrl_msgToSend.pwm_max = self.spinBox_pwm_max.value()
        #self.heading_ctrl_msgToSend.KI = self.spinBox_KI_heading.value()
        self.heading_ctrl_msgToSend.KP = self.spinBox_KP_heading.value()
        self.heading_ctrl_msgToSend.KD = self.spinBox_KD_heading.value()
        self.pub_set_heading.publish(self.heading_ctrl_msgToSend)


    def _velocity_param_clicked(self):
        self.velocity_ctrl_msgToSend.pwm_max = self.spinBox_pwm_max.value()
        #self.velocity_ctrl_msgToSend.KI = self.spinBox_KI_velocity.value()
        self.velocity_ctrl_msgToSend.KP = self.spinBox__KP_velocity.value()
        self.velocity_ctrl_msgToSend.KD = self.spinBox_KD_velocity.value()
        self.pub_set_velocity.publish(self.velocity_ctrl_msgToSend)

    def _attitude_param_clicked(self):
        self.attitude_ctrl_msgToSend.depth_desired = self.doubleSpinBox_depth.value()
        self.attitude_ctrl_msgToSend.heading_desired = self.doubleSpinBox_heading.value()
        self.attitude_ctrl_msgToSend.velocity_desired = self.doubleSpinBox_velocity.value()
        self.pub_set_attitude.publish(self.attitude_ctrl_msgToSend)


    def __init__(self):
        super(Display, self).__init__()
        
        uic.loadUi(PATH + "BlueRov2.ui", self)
        
        self.pub_set_heading = rospy.Publisher('/Settings/set_heading', Set_heading, queue_size = 10)
        self.pub_set_depth = rospy.Publisher('/Settings/set_depth', Set_depth, queue_size=10)
        self.pub_set_velocity = rospy.Publisher('/Settings/set_velocity', Set_velocity, queue_size=10)
        self.pub_set_attitude = rospy.Publisher('/Settings/set_attitude', Set_attitude, queue_size=10)
        rospy.Subscriber('/BlueRov2/arm', Bool, self._arm_callback) 
        rospy.Subscriber('/BlueRov2/battery', BatteryState, self._battery_callback)

        rospy.Subscriber('/Command/depth', UInt16, self._callback_depth)
        rospy.Subscriber('/Command/heading', UInt16, self._callback_heading)
        rospy.Subscriber('/Command/velocity', UInt16, self._callback_velocity)
        rospy.Subscriber('/Command/joy', Joy, self._callback_joy)
        
        rospy.Subscriber('/Settings/set_depth', Set_depth, self._settings_depth_ctrl_callback)
        rospy.Subscriber('/Settings/set_heading', Set_heading, self._settings_heading_ctrl_callback)
        rospy.Subscriber('/Settings/set_velocity', Set_velocity, self._settings_velocity_ctrl_callback)
        
        self.pushButton_send_parameters_heading.clicked.connect(self._heading_param_clicked)
        self.pushButton_send_parameters_velocity.clicked.connect(self._velocity_param_clicked)
        self.pushButton_send_parameters_depth.clicked.connect(self._depth_param_clicked)
        self.pushButton_send_parameters_attitude.clicked.connect(self._attitude_param_clicked)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.display)
        self.timer.start(500)
        
        self.arm = False    
        self.battery = None

        self.pwm_depth = None
        self.pwm_forward = None
        self.pwm_heading = None

        self.gamepad_axes = None
        self.gamepad_buttons = None
        self.override_controller = None
    
        self.heading_ctrl_param_rcv = Set_heading() #depth_desired, pwm_max, pwm_neutral, K, KI, KP, KD, rosrate
        self.depth_ctrl_param_rcv = Set_depth() #heading_desired, KP, KD, pwm_max, pwm_neutral, rosrate
        self.velocity_ctrl_param_rcv = Set_velocity()#velocity_desired, pwm_max, pwm_neutral, KP, KD, rosrate

        self.heading_ctrl_msgToSend = Set_heading() 
        self.depth_ctrl_msgToSend = Set_depth()
        self.velocity_ctrl_msgToSend = Set_velocity()
        self.attitude_ctrl_msgToSend = Set_attitude()
    def _arm_callback(self,msg):
        self.arm = msg.data

    def _battery_callback(self, msg):
        self.battery = msg.voltage
     
    def _callback_depth(self,msg):
        self.pwm_depth = msg.data

    def _callback_heading(self,msg):
        self.pwm_heading = msg.data

    def _callback_velocity(self,msg):
        self.pwm_forward = msg.data

    def _callback_joy(self,msg):
        self.gamepad_axes = msg.axes
        self.gamepad_buttons = msg.buttons
        self.override_controller = self.gamepad_buttons[1]
        
    def _settings_depth_ctrl_callback(self,msg):
        #depth_desired, pwm_max, pwm_neutral, K, KI, KP, KD, rosrate
        self.depth_ctrl_param_rcv = msg

    def _settings_heading_ctrl_callback(self,msg):
        #heading_desired, KP, KD, pwm_max, pwm_neutral, rosrate
        self.heading_ctrl_param_rcv = msg

    def _settings_velocity_ctrl_callback(self,msg):
        #velocity_desired, pwm_max, pwm_neutral, KP, KD, rosrate
        self.velocity_ctrl_param_rcv = msg

    
    def display(self):
        #STATUS section :
        if self.arm:
            self.arm_disarm_display.setText('ARM')
        else:
            self.arm_disarm_display.setText('DISARM')

        if self.override_controller == 0:
            self.automatic_manual_ctrl_display.setText('AUTOMATIC')
        else:
            self.automatic_manual_ctrl_display.setText('MANUAL')

        self.battery_level_display.display(self.battery)

	#self.light_level_display 
        
                #CONTROLLER OVERVIEW :
        self.depth_measured_display.display(-1)
        self.depth_controller_pwm_display.display(self.pwm_depth)
        self.depth_controller_KI_display.display(self.depth_ctrl_param_rcv.KI)
        self.depth_controller_KP_display.display(self.depth_ctrl_param_rcv.KP)
        self.depth_controller_KD_display.display(self.depth_ctrl_param_rcv.KD)
        

        self.heading_measured_display
        self.heading_controller_pwm_display.display(self.pwm_heading)
        self.heading_controller_KI_display.display(self.heading_ctrl_param_rcv.KP)
        self.heading_controller_KP_display
        self.heading_controller_KD_display.display(self.heading_ctrl_param_rcv.KD)

        self.velocity_measured_display
        self.velocity_controller_pwm_display.display('{}'.format(self.pwm_forward))
        self.velocity_controller_KI_display
        self.velocity_controller_KP_display.display(self.velocity_ctrl_param_rcv.KP)
        self.velocity_controller_KD_display.display(self.velocity_ctrl_param_rcv.KD)
    

if __name__ == "__main__":
    rospy.init_node('GUI', anonymous =True)
    app = QtWidgets.QApplication(sys.argv)
    window = Display()
    window.show()
    app.exec_()


