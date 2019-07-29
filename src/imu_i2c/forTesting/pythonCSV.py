import csv
import sys
import matplotlib.pyplot as plt
import numpy as np

FILE = sys.argv[1]
f = open(FILE, "r")
print(FILE)
fichierCSV = csv.reader(f)

IMU1_X = []
IMU1_Y = []
IMU1_Z = []

IMU1_X_raw = []
IMU1_Y_raw = []
IMU1_Z_raw = []

IMU2_X = []
IMU2_Y = []
IMU2_Z = []

IMU2_X_raw = []
IMU2_Y_raw = []
IMU2_Z_raw = []

time = []
for i in fichierCSV:
    try:
        line = [float(j) for j in i] 
        time.append(line[0])
        IMU1_X.append(line[1])
        IMU1_Y.append(line[2])
        IMU1_Z.append(line[3])
        IMU1_X_raw.append(line[4])
        IMU1_Y_raw.append(line[5])
        IMU1_Z_raw.append(line[6])
        IMU2_X.append(line[7])
        IMU2_Y.append(line[8])
        IMU2_Z.append(line[9])
        IMU2_X_raw.append(line[10])
        IMU2_Y_raw.append(line[11])
        IMU2_Z_raw.append(line[12])
    except :
        pass
    

time = np.array(time)
IMU1_X = np.array(IMU1_X) 
IMU1_Y = np.array(IMU1_Y)
IMU1_Z = np.array(IMU1_Z)

IMU1_X_raw = np.array(IMU1_X_raw)
IMU1_Y_raw = np.array(IMU1_Y_raw)
IMU1_Z_raw = np.array(IMU1_Z_raw)

IMU2_X = np.array(IMU1_X) 
IMU2_Y = np.array(IMU1_Y)
IMU2_Z = np.array(IMU1_Z)

IMU2_X_raw = np.array(IMU1_X_raw)
IMU2_Y_raw = np.array(IMU1_Y_raw)
IMU2_Z_raw = np.array(IMU1_Z_raw)

plt.figure()

plt.subplot(3,1,1)
plt.plot(time, IMU1_X,'+r', label="Normalize")
plt.plot(time, IMU1_X_raw,'+b', label="Raw")
plt.title("X acceleration IMU1")
plt.ylabel("Acceleration (m.s^-2)")
plt.legend()

plt.subplot(3,1,2)
plt.plot(time, IMU1_Y,'+r', label="Normalize")
plt.plot(time, IMU1_Y_raw,'+b', label="Raw")
plt.title("Y acceleration IMU1")
plt.ylabel("Acceleration (m.s^-2)")
plt.legend()

plt.subplot(3,1,3)
plt.plot(time, IMU1_Z,'+r', label="Normalize")
plt.plot(time, IMU1_Z_raw,'+b', label="Raw")
plt.title("Z acceleration IMU1")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (m.s^-2)")
plt.legend()

plt.figure(2)

plt.subplot(3,1,1)
plt.plot(time, IMU2_X,'+r', label="Normalize")
plt.plot(time, IMU2_X_raw,'+b', label="Raw")
plt.title("X acceleration IMU2")
plt.ylabel("Acceleration (m.s^-2)")
plt.legend()

plt.subplot(3,1,2)
plt.plot(time, IMU2_Y,'+r', label="Normalize")
plt.plot(time, IMU2_Y_raw,'+b', label="Raw")
plt.title("Y acceleration IMU2")
plt.ylabel("Acceleration (m.s^-2)")
plt.legend()

plt.subplot(3,1,3)
plt.plot(time, IMU2_Z,'+r', label="Normalize")
plt.plot(time, IMU2_Z_raw,'+b', label="Raw")
plt.title("Z acceleration IMU2")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (m.s^-2)")
plt.legend()


plt.show()


