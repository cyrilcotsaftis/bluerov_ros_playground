#!/usr/bin/env python3

import socket
import threading
import board
import busio
import adafruit_lsm9ds1
import json
import time

HOST = '' 
PORT = 14600

socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
socket.bind((HOST, PORT))
print('SERVER ON')
LOCK = threading.Lock()

i2c = busio.I2C(board.SCL, board.SDA)

imu1 = adafruit_lsm9ds1.LSM9DS1_I2C(i2c, adafruit_lsm9ds1._LSM9DS1_ADDRESS_MAG, adafruit_lsm9ds1._LSM9DS1_ADDRESS_ACCELGYRO) 

imu2 = adafruit_lsm9ds1.LSM9DS1_I2C(i2c, adafruit_lsm9ds1._LSM9DS1_ADDRESS_MAG2, adafruit_lsm9ds1._LSM9DS1_ADDRESS_ACCELGYRO2) #with SDO_M, SDO_AG connected to the ground

data_imu1 = {} 
data_imu2 = {} 

class ClientThread(threading.Thread):
    """Thread to communicate with the Imu_bridge.py on the topside computer

    When the client sends a "?", the thread sends back the IMU data dictionnary
    """
    def __init__(self, conn, address):
        threading.Thread.__init__(self)
        self.conn = conn
        self.address = address
        self.running = True

    def run(self):
        t0 = time.time()
        while self.running:
            if time.time() - t0 > 10:
                self.running = False
            data_rcv = self.conn.recv(1024)
            #print("recv : ", data_rcv.decode("utf8"))
            if data_rcv.decode("utf8") == "?":
                t0 = time.time()
                str_to_send = build_data()
                self.send(str_to_send)

    def send(self,data):
        """Encode data with utf8 standart and send them to the client."""
        data = data.encode("utf8")
        self.conn.send(data)

def build_data():
    """Build daa structure to send"""
    LOCK.acquire()
    data = {"IMU1": data_imu1, "IMU2": data_imu2}
    LOCK.release()
    return json.dumps(data)

def get_data():
    """Read data from IMU
    IMU1 is the one without SDOM, SDAG connected to ground
    IMU2 is the one with SDOM, SDAG, connected to ground
    """
    while True:
        global data_imu1, data_imu2
        LOCK.acquire()
        t = time.time()  
        accel_x, accel_y, accel_z = imu1.acceleration
        mag_x, mag_y, mag_z = imu1.magnetic
        gyro_x, gyro_y, gyro_z = imu1.gyro
        temp = imu1.temperature
        data_imu1 = {
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
        accel_x, accel_y, accel_z = imu2.acceleration
        mag_x, mag_y, mag_z = imu2.magnetic
        gyro_x, gyro_y, gyro_z = imu2.gyro
        temp = imu2.temperature
        data_imu2 = {
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
        LOCK.release()
        time.sleep(0.01)


get_data_thread = threading.Thread(target=get_data)
get_data_thread.start()
threads = []

while True:
    socket.listen(5)
    conn, address = socket.accept()
    print("une Connexion {}".format( address ))
    newthread = ClientThread(conn,address) 
    newthread.start()
    threads.append(newthread)
    
    #threads manager
    id_threadTerminated = []
    print(len(threads))
    for i in range(len(threads)): #find thread stopped
        if not threads[i].running:
            id_threadTerminated.append(i)
    for i in id_threadTerminated: #remove thread stopped from the list threads
        del(threads[i])
            
                    



