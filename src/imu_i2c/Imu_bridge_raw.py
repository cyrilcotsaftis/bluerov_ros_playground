#!/usr/bin/env python
#coding:utf-8

import socket
import time
import rospy
import json
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

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

class Imu_bridge_raw:
    def __init__(self, host="192.168.2.2",port=14600):
        self.host = host 
        self.port = port 
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connection()

        self.pub_imu1_raw = rospy.Publisher('/BlueRov2/imu/imu1_raw', Imu, queue_size=10)
        self.pub_imu1_mag_raw = rospy.Publisher('/BlueRov2/imu/mag1_raw', MagneticField, queue_size=10)
        self.pub_imu2_raw = rospy.Publisher('/BlueRov2/imu/imu2_raw', Imu, queue_size=10)
        self.pub_imu2_mag_raw = rospy.Publisher('/BlueRov2/imu/mag2_raw', MagneticField, queue_size=10)       
        
    #connection to the serveur
    def connection(self):
        try:
            self.socket.connect((self.host, self.port))
        except ConnectionRefusedError:
            print("Server not started")
        except:
            print("CONNECTION FAILED")

    def send(self, data):
        data = data.encode("utf8")
        self.socket.send(data)

    def recv(self):
        b_data =  self.socket.recv(1024) #1024 characters
        s_data = b_data.decode("utf8")
        return json.loads(s_data)      

    def publish(self,data):
        msg_imu1_raw, msg_mag1_raw, msg_imu2_raw, msg_mag2_raw = self._create_msg(data)

        self.pub_imu1_raw.publish(msg_imu1_raw)
        self.pub_imu2_raw.publish(msg_imu2_raw)
        self.pub_imu1_mag_raw.publish(msg_mag1_raw)
        self.pub_imu2_mag_raw.publish(msg_mag2_raw)
        

    def _create_msg(self,data):
        """
        Messages published with ROS have axis from the robot frame : x forward, y ?, z up or down
        """ 
        
        #----IMU 1 raw --------
        msg1_raw = Imu()
        msg1_magfield_raw = MagneticField()   
        msg1_raw.header.stamp = rospy.Time.now()
        msg1_raw.header.frame_id = '/base_link'
        msg1_magfield_raw.header.stamp = rospy.Time.now()
        msg1_magfield_raw.header.frame_id = '/base_link'

        msg1_raw.linear_acceleration.x = -data["IMU1"]["accel_y"]
        msg1_raw.linear_acceleration.y = data["IMU1"]["accel_z"]
        msg1_raw.linear_acceleration.z =  data["IMU1"]["accel_x"]
        msg1_raw.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        msg1_raw.angular_velocity.x = -data["IMU1"]["gyro_y"]
        msg1_raw.angular_velocity.y = -data["IMU1"]["gyro_z"]
        msg1_raw.angular_velocity.z =  data["IMU1"]["gyro_x"]
        msg1_raw.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        msg1_raw.orientation.w = 0
        msg1_raw.orientation.x = 0
        msg1_raw.orientation.y = 0
        msg1_raw.orientation.z = 0
        msg1_raw.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        msg1_magfield_raw.magnetic_field.x = -data["IMU1"]["mag_y"]  
        msg1_magfield_raw.magnetic_field.y = data["IMU1"]["mag_z"]
        msg1_magfield_raw.magnetic_field.z =  data["IMU1"]["mag_x"]
        
        
        #----IMU 2 raw --------
        msg2_raw = Imu()
        msg2_magfield_raw = MagneticField()   

        msg2_raw.header.stamp = rospy.Time.now()
        msg2_raw.header.frame_id = '/base_link'
        msg2_magfield_raw.header.stamp = rospy.Time.now()
        msg2_magfield_raw.header.frame_id = '/base_link'

        msg2_raw.linear_acceleration.x = data["IMU2"]["accel_y"]
        msg2_raw.linear_acceleration.y = -data["IMU2"]["accel_z"]
        msg2_raw.linear_acceleration.z = data["IMU2"]["accel_x"]
        msg2_raw.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        msg2_raw.angular_velocity.x = data["IMU2"]["gyro_y"]
        msg2_raw.angular_velocity.y = -data["IMU2"]["gyro_z"]
        msg2_raw.angular_velocity.z = data["IMU2"]["gyro_x"]
        msg2_raw.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        msg2_raw.orientation.w = 0
        msg2_raw.orientation.x = 0
        msg2_raw.orientation.y = 0
        msg2_raw.orientation.z = 0
        msg2_raw.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        msg2_magfield_raw.magnetic_field.x = data["IMU2"]["mag_y"]  
        msg2_magfield_raw.magnetic_field.y = data["IMU2"]["mag_z"]
        msg2_magfield_raw.magnetic_field.z = data["IMU2"]["mag_x"]
        
        return  msg1_raw, msg1_magfield_raw, msg2_raw, msg2_magfield_raw 
    
    def main(self):
        while True:
            self.send("?")
            data_rcv = self.recv()
            self.publish(data_rcv)
            time.sleep(0.01)

if __name__=="__main__":
    rospy.init_node('Imu_bridge_raw', anonymous=True)
    bridge = Imu_bridge_raw()
    bridge.main()
    bridge.socket.close()

