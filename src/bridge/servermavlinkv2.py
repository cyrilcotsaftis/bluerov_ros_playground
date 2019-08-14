#!/usr/bin/env python3

import socket
import threading
import time

HOST = '' 
PORT = 14550

socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
socket.bind((HOST, PORT))

while True:
    socket.listen(5)
    conn, address = socket.acc
    print("une Connexion {}".f
    newthread.start()
    threads.append(newthread)
