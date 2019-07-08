#!/usr/bin/env python3

import socket
import threading
import board
import busio
import adafruit_lsm9ds1
import json
import time

class Imu_reader:
    def __init__(self, host="192.168.2.2",port=14600):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.imu1 = adafruit_lsm9ds1.LSM9DS1_I2C(self.i2c, adafruit_lsm9ds1._LSM9DS1_ADDRESS_MAG, adafruit_lsm9ds1._LSM9DS1_ADDRESS_ACCELGYRO) 
        self.imu2 = adafruit_lsm9ds1.LSM9DS1_I2C(self.i2c, adafruit_lsm9ds1._LSM9DS1_ADDRESS_MAG2, adafruit_lsm9ds1._LSM9DS1_ADDRESS_ACCELGYRO2) #with SDO_M, SDO_AG connected to the ground

        self.data_imu1 = {} 
        self.data_imu2 = {} 
        self.str_imu1_2 = ""

        self.host = host 
        self.port = port 
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((host, port))
        print('SERVER ON')
        self.conn = None
        self.address = None
        self.lock = threading.Lock()
        self.connect()

    def connect(self):
        self.socket.listen(5)
        self.conn, self.address = self.socket.accept()
        print("une Connexion {}".format( self.address ))
        
        
    def get_data(self):
        while True:
            self.lock.acquire()
            print('acquisition')
            t = time.time()  
            accel_x, accel_y, accel_z = self.imu1.acceleration
            mag_x, mag_y, mag_z = self.imu1.magnetic
            gyro_x, gyro_y, gyro_z = self.imu1.gyro
            temp = self.imu1.temperature
            self.data_imu1 = {
                    "time": t,
                    "accel_x": accel_x,
                    "accel_y": accel_y,
                    "accel_z": accel_z,
                    "mag_x": mag_x,
                    "mag_y": mag_y,
                    "mag_z": mag_z,
                    "gyro_x": gyro_x,
                    "gyro_y": gyro_y,
                    "gyro_z": gyro_z,
                    "temperature": temp
                    }

            t = time.time()  
            accel_x, accel_y, accel_z = self.imu2.acceleration
            mag_x, mag_y, mag_z = self.imu2.magnetic
            gyro_x, gyro_y, gyro_z = self.imu2.gyro
            temp = self.imu2.temperature
            self.data_imu2 = {
                    "time": t,
                    "accel_x": accel_x,
                    "accel_y": accel_y,
                    "accel_z": accel_z,
                    "mag_x": mag_x,
                    "mag_y": mag_y,
                    "mag_z": mag_z,
                    "gyro_x": gyro_x,
                    "gyro_y": gyro_y,
                    "gyro_z": gyro_z,
                    "temperature": temp
                    }
            self.lock.release()
            time.sleep(0.01)
    
    def send(self,data):
        data = data.encode("utf8")
        self.conn.send(data)

    def recv(self):
        while True:
            print("receiving...")
            data_rcv = self.conn.recv(1024)
            print("recv : ", data_rcv.decode("utf8"))
            if data_rcv.decode("utf8") == "?":
                self.build_data()
                self.send(self.str_imu1_2)

    def build_data(self):
        self.lock.acquire()
        data = {"IMU1": self.data_imu1, "IMU2": self.data_imu2}
        self.str_imu1_2 = json.dumps(data)
        self.lock.release()

    def main(self):
        recv_thread = threading.Thread(target=self.recv)
        get_data_thread = threading.Thread(target=self.get_data)
        recv_thread.start()
        get_data_thread.start()

if __name__ == "__main__":
    imu_reader = Imu_reader(host = '')
    imu_reader.main()

