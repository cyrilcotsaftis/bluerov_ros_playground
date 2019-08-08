#!/usr/bin/env python
import sys
from PyQt5 import QtGui, QtCore, QtWidgets, uic
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Joy
from bluerov_ros_playground.msg import Attitude, Bar30, Set_depth, Set_heading, Set_target, Set_velocity, State
from recordThread import RecordThread

PATH = "/home/nathan/ROS_bluerov2_ws/src/bluerov_ros_playground/bluerov_ros/src/interface/"
g = 9.81  # m.s^-2 gravitationnal acceleration  
p0 = 990*100 #Pa surface pressure NEED to be cheked    
rho = 1000 # kg.m^3  water density

class Display(QtWidgets.QMainWindow):

    def _depth_param_clicked(self):
        self.depth_ctrl_msgToSend.KI = self.spinBox_KI_depth.value()
        self.depth_ctrl_msgToSend.KP = self.spinBox_KP_depth.value()
        self.depth_ctrl_msgToSend.KD = self.spinBox_KD_depth.value()

    def _heading_param_clicked(self):
        #self.heading_ctrl_msgToSend.KI = self.spinBox_KI_heading.value()
        self.heading_ctrl_msgToSend.KP = self.spinBox_KP_heading.value()
        self.heading_ctrl_msgToSend.KD = self.spinBox_KD_heading.value()

    def _velocity_param_clicked(self):
        #self.velocity_ctrl_msgToSend.KI = self.spinBox_KI_velocity.value()
        self.velocity_ctrl_msgToSend.KP = self.spinBox_KP_velocity.value()
        self.velocity_ctrl_msgToSend.KD = self.spinBox_KD_velocity.value()

    def _target_param_clicked(self):
        self.target_ctrl_msgToSend.depth_desired = - self.doubleSpinBox_depth.value()#because in depht_controller, depth desired = altitude ( so -XX to go under the surface)
        self.target_ctrl_msgToSend.heading_desired = self.doubleSpinBox_heading.value()
        self.target_ctrl_msgToSend.velocity_desired = self.doubleSpinBox_velocity.value()

    def _pwm_max_clicked(self):
        self.depth_ctrl_msgToSend.pwm_max = self.spinBox_pwm_max.value() 
        self.heading_ctrl_msgToSend.pwm_max = self.spinBox_pwm_max.value()
        self.velocity_ctrl_msgToSend.pwm_max = self.spinBox_pwm_max.value()

    def _activate_depth_ctrl_checked(self):
        self.depth_ctrl_msgToSend.enable_depth_ctrl = self.checkBox_activate_depth_controller.isChecked()
    def _activate_headind_ctrl_checked(self):
        self.heading_ctrl_msgToSend.enable_heading_ctrl = self.checkBox_activate_heading_controller.isChecked()
    def _activate_velocity_ctrl_checked(self):
        self.velocity_ctrl_msgToSend.enable_velocity_ctrl = self.checkBox_activate_velocity_controller.isChecked()

    def _record_depth_clicked(self):
        filename = "test.csv"
        recordtime = self.spinBox_depth_record_time.value()
        argtorecord = ["bar30_pressure_measured", ""]
        record = RecordThread(filename, recordtime, self, argtorecord)
        record.start()

    def _record_heading_clicked(self):
        pass
    def _record_velocity_clicked(self):
        pass

    def __init__(self):
        super(Display, self).__init__() 
        
        uic.loadUi(PATH + "BlueRov2.ui", self)
        
        self.pub_set_heading = rospy.Publisher('/Settings/set_heading', Set_heading, queue_size = 10)
        self.pub_set_depth = rospy.Publisher('/Settings/set_depth', Set_depth, queue_size=10)
        self.pub_set_velocity = rospy.Publisher('/Settings/set_velocity', Set_velocity, queue_size=10)
        self.pub_set_target = rospy.Publisher('/Settings/set_target', Set_target, queue_size=10)

        rospy.Subscriber('/BlueRov2/State', State, self._state_callback) 
        rospy.Subscriber('/BlueRov2/battery', BatteryState, self._battery_callback)
        rospy.Subscriber('/BlueRov2/bar30', Bar30, self._bar30_callback)
        rospy.Subscriber('/BlueRov2/imu/attitude', Attitude, self._callback_attitude)  

        rospy.Subscriber('/Command/depth', UInt16, self._callback_depth)
        rospy.Subscriber('/Command/heading', UInt16, self._callback_heading)
        rospy.Subscriber('/Command/velocity', UInt16, self._callback_velocity)
        rospy.Subscriber('/Command/joy', Joy, self._callback_joy)
        
        rospy.Subscriber('/Settings/set_depth', Set_depth, self._settings_depth_ctrl_callback)
        rospy.Subscriber('/Settings/set_heading', Set_heading, self._settings_heading_ctrl_callback)
        rospy.Subscriber('/Settings/set_velocity', Set_velocity, self._settings_velocity_ctrl_callback)
        rospy.Subscriber('/Settings/set_target', Set_target, self._settings_target_callback)

        self.pushButton_send_parameters_heading.clicked.connect(self._heading_param_clicked)
        self.pushButton_send_parameters_velocity.clicked.connect(self._velocity_param_clicked)
        self.pushButton_send_parameters_depth.clicked.connect(self._depth_param_clicked)
        self.pushButton_send_parameters_target.clicked.connect(self._target_param_clicked)
        self.pushButton_send_pwm_max.clicked.connect(self._pwm_max_clicked)
        self.pushButton_record_depth.clicked.connect(self._record_depth_clicked)
        self.pushButton_record_heading.clicked.connect(self._record_heading_clicked)
        self.pushButton_record_velocity.clicked.connect(self._record_velocity_clicked)
        self.checkBox_activate_depth_controller.clicked.connect(self._activate_depth_ctrl_checked)
        self.checkBox_activate_heading_controller.clicked.connect(self._activate_headind_ctrl_checked)
        self.checkBox_activate_velocity_controller.clicked.connect(self._activate_velocity_ctrl_checked)


        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.display)
        self.timer.start(250)
        
        self.battery = None

        self.pwm_depth = None
        self.pwm_forward = None
        self.pwm_heading = None

        self.gamepad_axes = None
        self.gamepad_buttons = None
        self.override_controller = None
        self.bar30_pressure_measured = 1015 
        self.heading_measured = None
        
        self.state = State()
        self.heading_ctrl_param_rcv = Set_heading() #depth_desired, pwm_max, pwm_neutral, K, KI, KP, KD, rosrate
        self.depth_ctrl_param_rcv = Set_depth() #heading_desired, KP, KD, pwm_max, pwm_neutral, rosrate
        self.velocity_ctrl_param_rcv = Set_velocity()#velocity_desired, pwm_max, pwm_neutral, KP, KD, rosrate
        self.target_rcv = Set_target()

        self.heading_ctrl_msgToSend = Set_heading() 
        self.depth_ctrl_msgToSend = Set_depth()
        self.velocity_ctrl_msgToSend = Set_velocity()
        self.target_ctrl_msgToSend = Set_target()
        

        #INITIALISATION OF THE VIEW
        self.init()

    def init(self):
        self._depth_param_clicked()
        self._heading_param_clicked()
        self._velocity_param_clicked()
        self._target_param_clicked()

    def _state_callback(self,msg):
        self.state = msg

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
    
    def _settings_target_callback(self, msg):
        self.target_rcv = msg

    def _bar30_callback(self,msg):
        self.bar30_pressure_measured = msg.press_abs

    def _callback_attitude(self, msg):
        self.heading_measured = msg.yaw

    def display(self):
        #STATUS section :
        if self.state.arm: 
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
        depth = -(self.bar30_pressure_measured*100-p0)/(rho*g) #pressure measured in hPa => *100 to have Pa
        self.depth_measured_display.display(depth)
        self.depth_controller_pwm_display.display(self.pwm_depth)
        self.depth_target_display.display(self.target_rcv.depth_desired)
        self.depth_controller_KI_display.display(self.depth_ctrl_param_rcv.KI)
        self.depth_controller_KP_display.display(self.depth_ctrl_param_rcv.KP)
        self.depth_controller_KD_display.display(self.depth_ctrl_param_rcv.KD)
        
        self.heading_measured_display.display(self.heading_measured)
        self.heading_target_display.display(self.target_rcv.heading_desired)
        self.heading_controller_pwm_display.display(self.pwm_heading)
        self.heading_controller_KI_display
        self.heading_controller_KP_display.display(self.heading_ctrl_param_rcv.KP)
        self.heading_controller_KD_display.display(self.heading_ctrl_param_rcv.KD)

        self.velocity_measured_display.display(-1)
        self.velocity_target_display.display(self.target_rcv.velocity_desired)
        self.velocity_controller_pwm_display.display(self.pwm_forward)
        self.velocity_controller_KI_display
        self.velocity_controller_KP_display.display(self.velocity_ctrl_param_rcv.KP)
        self.velocity_controller_KD_display.display(self.velocity_ctrl_param_rcv.KD)
        
        self.pub_set_velocity.publish(self.velocity_ctrl_msgToSend)
        self.pub_set_heading.publish(self.heading_ctrl_msgToSend)
        self.pub_set_target.publish(self.target_ctrl_msgToSend)
        self.pub_set_depth.publish(self.depth_ctrl_msgToSend)

if __name__ == "__main__":
    rospy.init_node('GUI', anonymous =True)
    app = QtWidgets.QApplication(sys.argv)
    window = Display()
    window.show()
    app.exec_()


