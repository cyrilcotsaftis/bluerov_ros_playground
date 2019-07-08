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
        return b_data.decode("utf8")
   
    def publish(self,data):
        print(data)
        msg_imu1 = Imu()
        msg_imu2 = Imu()
         #TODO : traitement des donnees recus pour les publier ROS
        msg_imu1.test = 0
        print(data)
        self.pub_imu1.publish(msg_imu1)
        self.pub_imu2.publish(msg_imu2)
    
    def main(self):
        while True:
            self.send("?")
            data_rcv = self.recv()
            self.publish(data_rcv)
            time.sleep(0.5)

if __name__=="__main__":
    rospy.init_node('imu__', anonymous=True)
    bridge = Imu_bridge()
    bridge.main()
    bridge.socket.close()

#    while not rospy.is_shutdown():
#        bridge.main()
#        time.sleep(0.01)

