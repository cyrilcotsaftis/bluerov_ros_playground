#!/usr/bin/env python
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
#        print(data)
        msg_imu1, msg_imu2 = self._create_msg(data)
        self.pub_imu1.publish(msg_imu1)
        self.pub_imu2.publish(msg_imu2)
        
    def _create_msg(self,data):

        msg1 = Imu()
     
        msg1.header.stamp = rospy.Time.now()
        msg1.header.frame_id = '/base_link1'
        
        msg1.linear_acceleration.x = -data["IMU1"]["accel_y"]
        msg1.linear_acceleration.y = -data["IMU1"]["accel_z"]
        msg1.linear_acceleration.z = data["IMU1"]["accel_x"]
        msg1.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        msg1.angular_velocity.x = -data["IMU1"]["gyro_y"]
        msg1.angular_velocity.y = -data["IMU1"]["gyro_z"]
        msg1.angular_velocity.z = data["IMU1"]["gyro_x"]
        msg1.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        msg1.orientation.w = 0
        msg1.orientation.x = 0
        msg1.orientation.y = 0
        msg1.orientation.z = 0
        msg1.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        
        msg2 = Imu()
     
        msg2.header.stamp = rospy.Time.now()
        msg2.header.frame_id = '/base_link2'
        
        msg2.linear_acceleration.x = data["IMU2"]["accel_y"]
        msg2.linear_acceleration.y = data["IMU2"]["accel_z"]
        msg2.linear_acceleration.z = data["IMU2"]["accel_x"]
        msg2.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        msg2.angular_velocity.x = data["IMU2"]["gyro_y"]
        msg2.angular_velocity.y = data["IMU2"]["gyro_z"]
        msg2.angular_velocity.z = data["IMU2"]["gyro_x"]
        msg2.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        msg2.orientation.w = 0
        msg2.orientation.x = 0
        msg2.orientation.y = 0
        msg2.orientation.z = 0
        msg2.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        #data[imu][mag_x]
        #data[imu][mag_y]
        #data[imu][mag_z]
        
        return msg1, msg2
   

    
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

