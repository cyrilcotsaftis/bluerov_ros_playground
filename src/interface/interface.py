#!/usr/bin/env python
from PyQt5 import QtCore, QtGui, QtWidgets, uic
import sys
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Joy
PATH = "/home/nathan/ROS_bluerov2_ws/src/bluerov_ros_playground/bluerov_ros/src/interface/"


class MyWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MyWindow,self).__init__()
        uic.loadUi(PATH + "TestBlueRov2.ui", self)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())
        
