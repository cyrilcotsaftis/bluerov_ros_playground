#!/usr/bin/env python
#coding:utf-8

import socket
import time
import rospy
import json
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
import numpy as np
import os
import inspect
"""
Data received :
Axes are those defined by the physical sensor on the robot
{IMU1:{"time": t,
"accel_x": accel_x,
"accel_y": accel_y,
"accel_z": accel_z,
"mag_x": mag_x,
"mag_y": mag_y,
"mag_z": mag_z,
"gyro_x": gyro_x,
"gyro_y": gyro_y,
"gyro_z": gyro_z,
"temperature": temp},  
IMU2:{"time": t,
"accel_x": accel_x,
"accel_y": accel_y,
"accel_z": accel_z,
"mag_x": mag_x,
"mag_y": mag_y,
"mag_z": mag_z,
"gyro_x": gyro_x,
"gyro_y": gyro_y,
"gyro_z": gyro_z,
"temperature": temp}}
"""

FOLDERPATH = os.path.split(inspect.getfile(inspect.currentframe()))[0]



class Imu_bridge:
    """Class Imu_bridge : Client to the imu server on Raspberry Pi. Collect data from Imus, calibrate and filter them befor publishing them on th etopic /imu/data_raw
    
    """
    
    def __init__(self, host="192.168.2.2",port=14600):
        self.host = host 
        self.port = port 
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connection()
        #----Uncomment to publish IMUs data on two different topics (4 more lines to uncomment in publish method)----
#        self.pub_imu1 = rospy.Publisher('/BlueRov2/imu/imu1', Imu, queue_size=10)
#        self.pub_imu2 = rospy.Publisher('/BlueRov2/imu/imu2', Imu, queue_size=10)
#        self.pub_imu1_mag = rospy.Publisher('/BlueRov2/imu/mag1', MagneticField, queue_size=10)
#        self.pub_imu2_mag = rospy.Publisher('/BlueRov2/imu/mag2', MagneticField, queue_size=10)
        self.pub_imu = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
        self.pub_mag = rospy.Publisher('/imu/mag', MagneticField, queue_size=10)

        #Calibration file in ./calibration/
        with open('{}/calibration/calibrationIMU1.json'.format(FOLDERPATH)) as f:
            calibrationFileIMU1 = json.load(f)
            self.IMU1_offsetVector = np.array([[calibrationFileIMU1["acc_off_x"]],
                                               [calibrationFileIMU1["acc_off_y"]],
                                               [calibrationFileIMU1["acc_off_z"]],
                                               [calibrationFileIMU1["gyr_off_x"]],
                                               [calibrationFileIMU1["gyr_off_y"]],
                                               [calibrationFileIMU1["gyr_off_z"]],
                                               [calibrationFileIMU1["magn_off_x"]],
                                               [calibrationFileIMU1["magn_off_y"]],
                                               [calibrationFileIMU1["magn_off_z"]]])
            self.IMU1_scaleVector = np.array([[calibrationFileIMU1["acc_scale_x"]],
                                              [calibrationFileIMU1["acc_scale_y"]],
                                              [calibrationFileIMU1["acc_scale_z"]],
                                              [calibrationFileIMU1["gyr_scale_x"]],
                                              [calibrationFileIMU1["gyr_scale_y"]],
                                              [calibrationFileIMU1["gyr_scale_z"]],
                                              [calibrationFileIMU1["magn_scale_x"]],
                                              [calibrationFileIMU1["magn_scale_y"]],
                                              [calibrationFileIMU1["magn_scale_z"]]])
        with open('{}/calibration/calibrationIMU2.json'.format(FOLDERPATH)) as f:
            calibrationFileIMU2 = json.load(f)
            self.IMU2_offsetVector = np.array([[calibrationFileIMU2["acc_off_x"]],
                                               [calibrationFileIMU2["acc_off_y"]],
                                               [calibrationFileIMU2["acc_off_z"]],
                                               [calibrationFileIMU2["gyr_off_x"]],
                                               [calibrationFileIMU2["gyr_off_y"]],
                                               [calibrationFileIMU2["gyr_off_z"]],
                                               [calibrationFileIMU2["magn_off_x"]],
                                               [calibrationFileIMU2["magn_off_y"]],
                                               [calibrationFileIMU2["magn_off_z"]]])
            self.IMU2_scaleVector = np.array([[calibrationFileIMU2["acc_scale_x"]],
                                              [calibrationFileIMU2["acc_scale_y"]],
                                              [calibrationFileIMU2["acc_scale_z"]],
                                              [calibrationFileIMU2["gyr_scale_x"]],
                                              [calibrationFileIMU2["gyr_scale_y"]],
                                              [calibrationFileIMU2["gyr_scale_z"]],
                                              [calibrationFileIMU2["magn_scale_x"]],
                                              [calibrationFileIMU2["magn_scale_y"]],
                                              [calibrationFileIMU2["magn_scale_z"]]])
        self.IMU1_data_flt = np.zeros((9,1)) #for filtered (flt) value
        self.IMU2_data_flt = np.zeros((9,1))
        self.w1 = np.array([[0.3], # coefficient of the IIR low pass filter for IMU 1
                            [0.3],
                            [0.3],
                            [0.3],
                            [0.3],
                            [0.3],
                            [0.3],
                            [0.3],
                            [0.3]])
        self.w2 = np.array([[0.3], #the same for IMU2
                            [0.3],
                            [0.3],
                            [0.3],
                            [0.3],
                            [0.3],
                            [0.3],
                            [0.3],
                            [0.3]])
        self.bound1 = np.array([[0.1], #to bound the value to try to get 0 when ROV motionless : not working in the lake
                                [0.1],
                                [0.1],
                                [0],
                                [0],
                                [0],
                                [0],
                                [0],
                                [0]])
        self.bound2 = np.array([[0.1], #same for IMU 2
                                [0.1],
                                [0.1],
                                [0],
                                [0],
                                [0],
                                [0],
                                [0],
                                [0]])
       

        self.IMU1_accX = 0        
        self.IMU1_accY = 0
        self.IMU1_accZ = 0
        self.IMU2_accX = 0
        self.IMU2_accY = 0
        self.IMU2_accZ = 0

    #connection to the serveur
    def connection(self):
        try:
            print("Connecting ...")
            self.socket.connect((self.host, self.port))
            print("CONNECTED to BlueRov2")
#        except ConnectionRefusedError: #only in python3 but ROS still use python2 in august 2019
#            print("Server not started")
        except:
            print("CONNECTION FAILED")

    def send(self, data):
        data = data.encode("utf8")
        self.socket.send(data)

    def recv(self):
        b_data =  self.socket.recv(1024) #1024 characters
        s_data = b_data.decode("utf8")
        return json.loads(s_data)      

    def publish(self):
        """Publish average of the 2 IMUs
        You can publish each imu if you uncommant the ligne for plublishing them : 4 lines in __init__, 4 in publish
        """
        msg_imu1, msg_mag1, msg_imu2, msg_mag2, msg_imu, msg_mag= self._create_msg()
        self.pub_imu.publish(msg_imu)
        self.pub_mag.publish(msg_mag)
        #------Uncomment to publish IMUs data separately------
#        self.pub_imu1.publish(msg_imu1)
#        self.pub_imu2.publish(msg_imu2)
#        self.pub_imu1_mag.publish(msg_mag1)
#        self.pub_imu2_mag.publish(msg_mag2)

    def _create_msg(self):
        """
        Messages published with ROS have axis from the robot frame : x forward, y left, z up 
        """
    #----IMU 1----
        msg1 = Imu()
        msg1_magfield = MagneticField()   
        msg1.header.stamp = rospy.Time.now()
        msg1.header.frame_id = '/base_link'
        msg1_magfield.header.stamp = rospy.Time.now()
        msg1_magfield.header.frame_id = '/base_link'

        #For calibration and normalisation of linear_acceleration and magnetometer
 
        msg1.linear_acceleration.x = self.IMU1_data_flt[0]
        msg1.linear_acceleration.y = self.IMU1_data_flt[1]
        msg1.linear_acceleration.z = self.IMU1_data_flt[2]

        msg1.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        msg1.angular_velocity.x = self.IMU1_data_flt[3]
        msg1.angular_velocity.y = self.IMU1_data_flt[4]
        msg1.angular_velocity.z = self.IMU1_data_flt[5]
        msg1.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        msg1.orientation.w = 0
        msg1.orientation.x = 0
        msg1.orientation.y = 0
        msg1.orientation.z = 0
        msg1.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
       
        msg1_magfield.magnetic_field.x = self.IMU1_data_flt[6]
        msg1_magfield.magnetic_field.y = self.IMU1_data_flt[7]
        msg1_magfield.magnetic_field.z = self.IMU1_data_flt[8]
        

    #----IMU 2----
        msg2 = Imu()
        msg2_magfield = MagneticField()   
     
        msg2.header.stamp = rospy.Time.now()
        msg2.header.frame_id = '/base_link'
        msg2_magfield.header.stamp = rospy.Time.now()
        msg2_magfield.header.frame_id = '/base_link'

        msg2.linear_acceleration.x = self.IMU2_data_flt[0]
        msg2.linear_acceleration.y = self.IMU2_data_flt[1]
        msg2.linear_acceleration.z = self.IMU2_data_flt[2]
        msg2.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        msg2.angular_velocity.x = self.IMU2_data_flt[3]
        msg2.angular_velocity.y = self.IMU2_data_flt[4]
        msg2.angular_velocity.z = self.IMU2_data_flt[5]
        msg2.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        msg2.orientation.w = 0
        msg2.orientation.x = 0
        msg2.orientation.y = 0
        msg2.orientation.z = 0
        msg2.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        msg2_magfield.magnetic_field.x = self.IMU2_data_flt[6]
        msg2_magfield.magnetic_field.y = self.IMU2_data_flt[7]
        msg2_magfield.magnetic_field.z = self.IMU2_data_flt[8]

    #----AVERAGE----
        msgA = Imu()
        msgA_magfield = MagneticField()   
     
        msgA.header.stamp = rospy.Time.now()
        msgA.header.frame_id = '/base_link'
        msgA_magfield.header.stamp = rospy.Time.now()
        msgA_magfield.header.frame_id = '/base_link'

        msgA.linear_acceleration.x = (self.IMU1_data_flt[0] + self.IMU2_data_flt[0])/2
        msgA.linear_acceleration.y = (self.IMU1_data_flt[1] + self.IMU2_data_flt[1])/2
        msgA.linear_acceleration.z = (self.IMU1_data_flt[2] + self.IMU2_data_flt[2])/2
        msgA.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        msgA.angular_velocity.x = (self.IMU1_data_flt[3] + self.IMU2_data_flt[3])/2
        msgA.angular_velocity.y = (self.IMU1_data_flt[4] + self.IMU2_data_flt[4])/2
        msgA.angular_velocity.z = (self.IMU1_data_flt[5] + self.IMU2_data_flt[5])/2
        msgA.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        msgA.orientation.w = 0
        msgA.orientation.x = 0
        msgA.orientation.y = 0
        msgA.orientation.z = 0
        msgA.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        msgA_magfield.magnetic_field.x = (self.IMU1_data_flt[6] + self.IMU2_data_flt[6])/2
        msgA_magfield.magnetic_field.y = (self.IMU1_data_flt[7] + self.IMU2_data_flt[7])/2
        msgA_magfield.magnetic_field.z = (self.IMU1_data_flt[8] + self.IMU2_data_flt[8])/2

        return msg1, msg1_magfield, msg2, msg2_magfield, msgA, msgA_magfield 
    
    def process_data(self, data):
        """Process IMU data : calibration and IIR lowpass filter
        """
        IMU1_data = np.array([[-data["IMU1"]["accel_y"]],
                              [ data["IMU1"]["accel_z"]],
                              [ data["IMU1"]["accel_x"]], 
                              [-data["IMU1"]["gyro_y"]],
                              [ data["IMU1"]["gyro_z"]],
                              [ data["IMU1"]["gyro_x"]],
                              [-data["IMU1"]["mag_y"]], 
                              [-data["IMU1"]["mag_z"]],
                              [ data["IMU1"]["mag_x"]]])
        IMU2_data = np.array([[data["IMU2"]["accel_y"]],
                              [-data["IMU2"]["accel_z"]],
                              [data["IMU2"]["accel_x"]],
                              [data["IMU2"]["gyro_y"]],
                              [-data["IMU2"]["gyro_z"]],
                              [data["IMU2"]["gyro_x"]],
                              [data["IMU2"]["mag_y"]],
                              [data["IMU2"]["mag_z"]],
                              [data["IMU2"]["mag_x"]]])
        IMU1_data_cor = self.IMU1_scaleVector*(IMU1_data-self.IMU1_offsetVector) #Calibration IMU1
        IMU2_data_cor = self.IMU2_scaleVector*(IMU2_data-self.IMU2_offsetVector) #Calibration IMU2
        self.IMU1_data_flt = self.w1*IMU1_data_cor + (1-self.w1)*self.IMU1_data_flt #IIR lowpass filter IMU1
        self.IMU2_data_flt = self.w2*IMU2_data_cor + (1-self.w2)*self.IMU2_data_flt #IIR lowpass filter IMU2
        
        #To bound the acceleration to try to get 0 when ROV motionless, not working in lake
#        for i in range(len(self.IMU1_data_flt)):
#            if self.bound1[i]>self.IMU1_data_flt[i] and self.IMU1_data_flt[i]>-self.bound1[i]:
#                self.IMU1_data_flt[i] = 0
#            if self.bound2[i]>self.IMU2_data_flt[i] and self.IMU2_data_flt[i]>-self.bound2[i]:
#                self.IMU2_data_flt[i] = 0
            

    def main(self):
        while True:
            self.send("?")
            data_rcv = self.recv()
            self.process_data(data_rcv)
            self.publish()
            time.sleep(0.01)

if __name__=="__main__":
    rospy.init_node('Imu_bridge', anonymous=True)
    bridge = Imu_bridge()
    bridge.main()
    bridge.socket.close()
