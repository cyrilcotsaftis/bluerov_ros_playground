import csv
import sys
import matplotlib.pyplot as plt
import numpy as np
from numpy.fft import fft, fftfreq
import scipy.integrate

FILE = sys.argv[1]
f = open(FILE, "r")
print(FILE)
fichierCSV = csv.reader(f)

if FILE[:3] == 'acc':
    sensor = "Linear Acceleration"
    unit = "Acceleration (m.s^-2)"
elif FILE[:3]=='gyr':
    sensor = "Angular Acceleration"
    unit = "Speed (dps)"
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

IMU2_X = np.array(IMU2_X) 
IMU2_Y = np.array(IMU2_Y)
IMU2_Z = np.array(IMU2_Z)

IMU1_X_raw = np.array(IMU1_X_raw)
IMU1_Y_raw = np.array(IMU1_Y_raw)
IMU1_Z_raw = np.array(IMU1_Z_raw)

IMU2_X_raw = np.array(IMU2_X_raw)
IMU2_Y_raw = np.array(IMU2_Y_raw)
IMU2_Z_raw = np.array(IMU2_Z_raw)


def graph():
    
    if sensor == "Linear Acceleration":
        plt.figure()
        plt.subplot(3,2,1)
        plt.plot(time, IMU1_X_gt, 'r', label="IMU1")
        plt.plot(time, IMU2_X_gt, 'b', label="IMU2")
        plt.ylabel("{}".format(unit))
        plt.title("{} X".format(sensor))
        plt.legend()
        plt.subplot(3,2,3)
        plt.plot(time, speedX_mean_simpson,'k', label="IMU1+IMU2")
        plt.plot(time, speedTEST_mean_simpson,'g', label="try")
        plt.title("X Speed")
        plt.xlabel("Time (s)")
        plt.ylabel("Speed (m.s^-1)")
        plt.legend()
        plt.subplot(3,2,2)
        plt.plot(time, IMU1_Y_gt, 'r', label="IMU1")
        plt.plot(time, IMU2_Y_gt, 'b', label="IMU2")
        plt.title("X Speed")
        plt.ylabel("{}".format(unit))
        plt.title("{} Y".format(sensor))
        plt.legend()
        plt.subplot(3,2,4)
        plt.plot(time, speedY_mean_simpson,'k', label="IMU1+IMU2")
        plt.title("Y Speed")
        plt.xlabel("Time (s)")
        plt.ylabel("Speed (m.s^-1)")
        plt.legend()
        plt.subplot(3,2,5)
        plt.plot(time, posX_mean_simpson)
#        plt.plot(posX_mean_simpson, posY_mean_simpson,'g', label="")
        plt.title("X Position")
        plt.xlabel("Pos (m)")
        plt.ylabel("Pos (m)")
        plt.legend()
        plt.subplot(3,2,6)
        plt.plot(time, posY_mean_simpson)
#        plt.plot(posX_mean_simpson, posY_mean_simpson,'g', label="")
        plt.title("Y Position")
        plt.xlabel("Pos (m)")
        plt.ylabel("Pos (m)")
        plt.legend()
    plt.show()


#-------To zeros--------
shp = IMU1_X.shape
IMU1_X_gt = IMU1_X.copy()
IMU1_Y_gt = IMU1_Y.copy()
IMU1_Z_gt = IMU1_Z.copy()
IMU2_X_gt = IMU1_X.copy()
IMU2_Y_gt = IMU1_Y.copy()
IMU2_Z_gt = IMU1_Z.copy()

#thresh = 0.25
#for i in range(len(IMU1_X)):
#    if IMU1_X[i]>-thresh and IMU1_X[i]<thresh:
#        IMU1_X_gt[i] = 0
#    if IMU1_Y[i]>-thresh and IMU1_Y[i]<thresh:
#        IMU1_Y_gt[i] = 0
#    if IMU1_Z[i]>-thresh and IMU1_Z[i]<thresh:
#        IMU1_Z_gt[i] = 0
#    if IMU2_X[i]>-thresh and IMU2_X[i]<thresh:
#        IMU2_X_gt[i] = 0
#    if IMU2_Y[i]>-thresh and IMU2_Y[i]<thresh:
#        IMU2_Y_gt[i] = 0
#    if IMU2_Z[i]>-thresh and IMU2_Z[i]<thresh:
#        IMU2_Z_gt[i] = 0
#
#IMU1_X_gt = np.array(IMU1_X_gt) 
#IMU1_Y_gt = np.array(IMU1_Y_gt) 
#IMU1_Z_gt = np.array(IMU1_Z_gt) 
#IMU2_X_gt = np.array(IMU2_X_gt) 
#IMU2_Y_gt = np.array(IMU2_Y_gt) 
#IMU2_Z_gt = np.array(IMU2_Z_gt) 



#-------INTGR => SPEED-------
meanX = (IMU1_X_gt + IMU2_X_gt)/2
meanY = (IMU1_Y_gt + IMU2_Y_gt)/2
speedX_mean_simpson = [0]
speedY_mean_simpson = [0]
speedTEST_mean_simpson = [0]
dt = 0.01
for i in range(0,len(meanX)-1):
    speedX_mean_simpson.append(scipy.integrate.simps(meanX[:i+1], dx=dt))
    speedY_mean_simpson.append(scipy.integrate.simps(meanY[:i+1], dx=dt))
    speedTEST_mean_simpson.append(speedTEST_mean_simpson[i] + scipy.integrate.simps(meanX[i:i+2], dx=dt))
speedX_mean_simpson= np.array(speedX_mean_simpson)
speedY_mean_simpson= np.array(speedY_mean_simpson)


a, b = 0, len(speedX_IMU1_simpson)
coeff = np.polyfit(time[a:b], speedX_IMU1_simpson[a,b])
lin_reg = coeff[0]*time+coeff[1]
speedX_IMU1_simpson_corr = speedX_IMU1_simpson - (coeff[0]*time[a:b]+coeff[1])
#-------INTGR => SPEED-------
posX_mean_simpson = [0]
posY_mean_simpson = [0]
for i in range(0,len(meanX)-1):
    posX_mean_simpson.append(scipy.integrate.simps(speedX_mean_simpson[:i+3]))
    posY_mean_simpson.append(scipy.integrate.simps(speedY_mean_simpson[:i+3]))


graph()


