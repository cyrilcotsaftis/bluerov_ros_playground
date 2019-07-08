#!/usr/bin/env python3
#coding:utf-8

import socket
import time
import rospy
import json
from sensor_msgs.msg import Imu

class Imu_bridge:
    def __init__(self, host="192.168.2.2",port=14600):
        self.host = host 
        self.port = port 
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connection()
        
        self.pub_imu1 = rospy.Publisher('/BlueRov2/imu/imu1', Imu, queue_size=10)
        self.pub_imu2 = rospy.Publisher('/BlueRov2/imu/imu2', Imu, queue_size=10)

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
        print(data)
        msg_imu1 = self._create_msg(data,"IMU1")
        msg_imu2 = self._create_msg(data,"IMU2")
        self.pub_imu1.publish(msg_imu1)
        self.pub_imu2.publish(msg_imu2)
        
    def _create_msg(self,data,imu):
        #IMU1, 2
        
        #accel_x ,y, z
        #gyro_x ,y, z
        #mag_x, y, z
        msg = Imu()
        
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/base_link'
        
        msg.linear_acceleration.x = data[imu]["accel_x"]
        msg.linear_acceleration.y = data[imu]["accel_y"]
        msg.linear_acceleration.z = data[imu]["accel_z"]
        msg.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        msg.angular_velocity.x = data[imu]["gyro_x"]
        msg.angular_velocity.y = data[imu]["gyro_y"]
        msg.angular_velocity.z = data[imu]["gyro_z"]
        msg.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        msg.orientation.w = 0
        msg.orientation.x = 0
        msg.orientation.y = 0
        msg.orientation.z = 0
        msg.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        #data[imu][mag_x]
        #data[imu][mag_y]
        #data[imu][mag_z]
        
        return msg
   

    
    def main(self):
        while True:
            self.send("?")
            data_rcv = self.recv()
            self.publish(data_rcv)
            time.sleep(0.01)

if __name__=="__main__":
    rospy.init_node('imu__', anonymous=True)
    bridge = Imu_bridge()
    bridge.main()
    bridge.socket.close()

#    while not rospy.is_shutdown():
#        bridge.main()
#        time.sleep(0.01)

