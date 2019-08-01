import csv
import sys
import matplotlib.pyplot as plt
import numpy as np

FILE = sys.argv[1]
f = open(FILE, "r")
print(FILE)
fichierCSV = csv.reader(f)

if FILE[:3] == 'acc':
    sensor = "Linear Acceleration"
    unit = "Acceleration (m.s^-2)"
elif FILE[:3]=='gyr':
    sensor = "Angular Acceleration"
    unit = "Acceleration (dps)"
elif FILE[:3] == 'mag':
    sensor = "Magnetic Field"
    unit = "Magnitude (g)"

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

IMU2_X = np.array(IMU2_X) 
IMU2_Y = np.array(IMU2_Y)
IMU2_Z = np.array(IMU2_Z)

IMU2_X_raw = np.array(IMU2_X_raw)
IMU2_Y_raw = np.array(IMU2_Y_raw)
IMU2_Z_raw = np.array(IMU2_Z_raw)


IMU1_X_raw_mean = [IMU1_X_raw.mean() for i in range(len(time))]
IMU1_Y_raw_mean = [IMU1_Y_raw.mean() for i in range(len(time))]
IMU1_Z_raw_mean = [IMU1_Z_raw.mean() for i in range(len(time))]

IMU2_X_raw_mean = [IMU2_X_raw.mean() for i in range(len(time))]
IMU2_Y_raw_mean = [IMU2_Y_raw.mean() for i in range(len(time))]
IMU2_Z_raw_mean = [IMU2_Z_raw.mean() for i in range(len(time))]

IMU1_X_raw_shift = IMU1_X_raw-IMU1_X_raw_mean
IMU1_Y_raw_shift = IMU1_Y_raw-IMU1_Y_raw_mean
IMU1_Z_raw_shift = IMU1_Z_raw-IMU1_Z_raw_mean

IMU2_X_raw_shift = IMU2_X_raw-IMU2_X_raw_mean
IMU2_Y_raw_shift = IMU2_Y_raw-IMU2_Y_raw_mean
IMU2_Z_raw_shift = IMU2_Z_raw-IMU2_Z_raw_mean

IMU1_X_raw_shift_mean = [IMU1_X_raw_shift.mean() for i in range(len(time))]
IMU1_Y_raw_shift_mean = [IMU1_Y_raw_shift.mean() for i in range(len(time))]
IMU1_Z_raw_shift_mean = [IMU1_Z_raw_shift.mean() for i in range(len(time))]

IMU2_X_raw_shift_mean = [IMU2_X_raw_shift.mean() for i in range(len(time))]
IMU2_Y_raw_shift_mean = [IMU2_Y_raw_shift.mean() for i in range(len(time))]
IMU2_Z_raw_shift_mean = [IMU2_Z_raw_shift.mean() for i in range(len(time))]


def graph():
    plt.figure()

    plt.subplot(3,1,1)
    plt.plot(time, IMU1_X,'+r', label="Normed")
    plt.plot(time, IMU1_X_raw,'+b', label="Raw")
    plt.plot(time, IMU1_X_raw_shift,'+g', label="Raw shift")
    plt.plot(time, IMU1_X_raw_mean,'k', label=" avg = {}".format(IMU1_X_raw_mean[0]))
    plt.plot(time, IMU1_X_raw_shift_mean,'k', label=" avg = {}".format(IMU1_X_raw_shift_mean[0]))

    plt.title("{} X  IMU1".format(sensor))
    plt.ylabel("{}".format(unit))
    plt.legend()

    plt.subplot(3,1,2)
    plt.plot(time, IMU1_Y,'+r', label="Normed")
    plt.plot(time, IMU1_Y_raw,'+b', label="Raw")
    plt.plot(time, IMU1_Y_raw_shift,'+g', label="Raw shift")
    plt.plot(time, IMU1_Y_raw_mean,'k', label=" avg = {}".format(IMU1_Y_raw_mean[0]))
    plt.plot(time, IMU1_Y_raw_shift_mean,'k', label=" avg = {}".format(IMU1_Y_raw_shift_mean[0]))
    #plt.plot(time, IMU1_Y_raw_flt, "m", label="flt")
    plt.title("{} Y IMU1".format(sensor))
    plt.ylabel("Acceleration (m.s^-2)")
    plt.legend()

    plt.subplot(3,1,3)
    plt.plot(time, IMU1_Z,'+r', label="Normed")
    plt.plot(time, IMU1_Z_raw,'+b', label="Raw")
    plt.plot(time, IMU1_Z_raw_shift,'+g', label="Raw shift")
    plt.plot(time, IMU1_Z_raw_mean,'k', label=" avg = {}".format(IMU1_Z_raw_mean[0]))
    plt.plot(time, IMU1_Z_raw_shift_mean,'k', label=" avg = {}".format(IMU1_Z_raw_shift_mean[0]))
    plt.title("{} Z IMU1".format(sensor))
    plt.xlabel("Time (s)")
    plt.ylabel("{}".format(unit))
    plt.legend()

    plt.figure(2)

    plt.subplot(3,1,1)
    plt.plot(time, IMU2_X,'+r', label="Normed")
    plt.plot(time, IMU2_X_raw,'+b', label="Raw")
    plt.plot(time, IMU2_X_raw_shift,'+g', label="Raw shift")
    plt.plot(time, IMU2_X_raw_mean,'k', label=" avg = {}".format(IMU2_X_raw_mean[0]))
    plt.plot(time, IMU2_X_raw_shift_mean,'k', label=" avg = {}".format(IMU2_X_raw_shift_mean[0]))
    plt.title("{} X IMU2".format(sensor))
    plt.ylabel("{}".format(unit))
    plt.legend()

    plt.subplot(3,1,2)
    plt.plot(time, IMU2_Y,'+r', label="Normed")
    plt.plot(time, IMU2_Y_raw,'+b', label="Raw")
    plt.plot(time, IMU2_Y_raw_shift,'+g', label="Raw shift")
    plt.plot(time, IMU2_Y_raw_mean,'k', label=" avg = {}".format(IMU2_Y_raw_mean[0]))
    plt.plot(time, IMU2_Y_raw_shift_mean,'k', label=" avg = {}".format(IMU2_Y_raw_shift_mean[0]))
    plt.title("{} Y IMU2".format(sensor))
    plt.ylabel("{}".format(unit))
    plt.legend()

    plt.subplot(3,1,3)
    plt.plot(time, IMU2_Z,'+r', label="Normed")
    plt.plot(time, IMU2_Z_raw,'+b', label="Raw")
    plt.plot(time, IMU2_Z_raw_shift,'+g', label="Raw shift")
    plt.plot(time, IMU2_Z_raw_mean,'k', label=" avg = {}".format(IMU2_Z_raw_mean[0]))
    plt.plot(time, IMU2_Z_raw_shift_mean,'k', label=" avg = {}".format(IMU2_Z_raw_shift_mean[0]))
    plt.title("{} Z IMU2".format(sensor))
    plt.xlabel("Time (s)")
    plt.ylabel("{}".format(unit))
    plt.legend()
    
    if sensor == "Linear Acceleration":
        plt.figure(3)
        plt.subplot(3,1,1)
        plt.plot(time, IMU1_X, '+r', label="Normalize")
        plt.plot(time, IMU1_X_raw, '+b', label="Raw")
        plt.ylabel("{}".format(unit))
        plt.title("{} X IMU1".format(sensor))
        plt.legend()
        plt.subplot(3,1,2)
        plt.plot(time, speedX_IMU1_raw_euler, '+b', label="Euler")
        plt.title("X Speed Raw")
        plt.ylabel("Speed (m.s^-1)")
        plt.legend()

        plt.subplot(3,1,3)
        plt.plot(time, speedX_IMU1_euler,'+r', label="Euler")
        plt.title("X Speed")
        plt.xlabel("Time (s)")
        plt.ylabel("Speed (m.s^-1)")
        plt.legend()
    plt.show()

#-------EULER INTGR-------
speedX_IMU1_euler = [0]
speedX_IMU1_raw_euler = [0]
speedX_IMU2_euler = [0]
speedX_MEAN_euler = [0]
speedX_IMU1_simpson = [IMU1_X[0]+IMU1_X[0]+IMU1_X[1]]
speedX_IMU2_simpson = [IMU2_X[0]+IMU2_X[0]+IMU2_X[1]]
dt = 0.01
for i in range(0,len(IMU1_X)-1):
    speedX_IMU1_raw_euler.append(speedX_IMU1_raw_euler[i-1] + dt * IMU1_X_raw[i])
    speedX_IMU1_euler.append(speedX_IMU1_euler[i-1] + dt * IMU1_X[i])

    speedX_MEAN_euler.append(speedX_MEAN_euler[i-1] + dt * ((IMU1_X[i]+IMU2_X[i])/2.))
    speedX_IMU1_simpson.append(speedX_IMU1_simpson[i-1] + IMU1_X[i]+IMU1_X[i]+IMU1_X[i+1])
    speedX_IMU2_simpson.append(speedX_IMU2_simpson[i-1] + IMU2_X[i]+IMU2_X[i]+IMU2_X[i+1])

speedX_IMU1_euler = np.array(speedX_IMU1_euler)
print(speedX_IMU1_euler.shape)
speedX_IMU1_raw_euler = np.array(speedX_IMU1_raw_euler)
speedX_IMU2_euler = np.array(speedX_IMU2_euler)
speedX_MEAN_euler = np.array(speedX_MEAN_euler)
speedX_IMU1_simpson =  np.array(speedX_IMU1_simpson)*(dt/3)
speedX_IMU2_simpson = np.array(speedX_IMU2_simpson) *(dt/3)

graph()


