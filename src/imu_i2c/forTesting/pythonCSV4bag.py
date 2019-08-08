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
    sensor = "Linear Acceleration {}".format(FILE)
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
    
    plt.figure()
    plt.subplot(3,2,1)
    plt.plot(time, IMU1_X, 'r', label="IMU1+IMU2")
    plt.ylabel("{}".format(unit))
    plt.title("{} X".format(sensor))
    plt.legend()
    plt.subplot(3,2,3)
    plt.plot(time, lin_reg,'k', label="lin_reg")
    plt.plot(time, speedX_IMU1_simpson_corr,'b', label="IMU1+IMU2 corr")
    plt.plot(time[a:b], speedX_IMU1_simpson[a:b],'m', label="IMU1+IMU2")
    plt.plot(time[0:a], speedX_IMU1_simpson[0:a],'r', label="IMU1+IMU2")
    plt.plot(time[b::], speedX_IMU1_simpson[b::],'r', label="IMU1+IMU2")
    plt.plot(time_shrink, speedX_IMU1_simpson_shrink,'g', label="shrink")
    plt.title("X Speed")
    plt.xlabel("Time (s)")
    plt.ylabel("Speed (m.s^-1)")
    plt.legend()
    plt.subplot(3,2,2)
    plt.plot(time, IMU1_Y, 'r', label="IMU1+IMU2")
    plt.title("X Speed")
    plt.ylabel("{}".format(unit))
    plt.title("{} Y".format(sensor))
    plt.legend()
    plt.subplot(3,2,4)
    plt.plot(time, speedY_IMU1_simpson,'k', label="IMU1+IMU2")
    plt.title("Y Speed")
    plt.xlabel("Time (s)")
    plt.ylabel("Speed (m.s^-1)")
    plt.legend()
    plt.subplot(3,2,5)
    plt.plot(time, posX_mean_simpson)
#   plt.plot(posX_mean_simpson, posY_mean_simpson,'g', label="")
    plt.title("X Position")
    plt.xlabel("Pos (m)")
    plt.ylabel("Pos (m)")
    plt.legend()
    plt.subplot(3,2,6)
    plt.plot(time, posY_mean_simpson)
#   plt.plot(posX_mean_simpson, posY_mean_simpson,'g', label="")
    plt.title("Y Position")
    plt.xlabel("Pos (m)")
    plt.ylabel("Pos (m)")
    plt.legend()

    plt.figure(2)

    plt.subplot(3,1,1)
    plt.plot(time, IMU1_X,'r', label="IMU1+IMU2")
    plt.plot(time, IMU1_X_raw,',b', label="Raw")
    plt.title("{} X  IMU1".format(sensor))
    plt.ylabel("{}".format(unit))
    plt.legend()

    plt.subplot(3,1,2)
    plt.plot(time, IMU1_Y,'r', label="IMU1+IMU2")
    plt.plot(time, IMU1_Y_raw,',b', label="Raw")
    plt.title("{} Y IMU1".format(sensor))
    plt.ylabel("{}".format(unit))
    plt.legend()

    plt.subplot(3,1,3)
    plt.plot(time, IMU1_Z,'r', label="IMU1+IMU2")
    plt.plot(time, IMU1_Z_raw,',b', label="Raw")
    plt.title("{} Z IMU1".format(sensor))
    plt.xlabel("Time (s)")
    plt.ylabel("{}".format(unit))
    plt.legend()

    plt.figure(3)

    plt.subplot(3,1,1)
    plt.plot(time, IMU1_X,'r', label="IMU1+IMU2")
    plt.plot(time, IMU2_X_raw,',b', label="Raw")
    plt.title("{} X IMU2".format(sensor))
    plt.ylabel("{}".format(unit))
    plt.legend()

    plt.subplot(3,1,2)
    plt.plot(time, IMU1_Y,'r', label="IMU1+IMU2")
    plt.plot(time, IMU2_Y_raw,',b', label="Raw")
    plt.title("{} Y IMU2".format(sensor))
    plt.ylabel("{}".format(unit))
    plt.legend()

    plt.subplot(3,1,3)
    plt.plot(time, IMU1_Z,'r', label="IMU1+IMU2")
    plt.plot(time, IMU2_Z_raw,',b', label="Raw")
    plt.title("{} Z IMU2".format(sensor))
    plt.xlabel("Time (s)")
    plt.ylabel("{}".format(unit))
    plt.legend()
 
    plt.show()

#-------To zeros--------
shp = IMU1_X.shape
IMU1_X_gt = IMU1_X_raw.copy()
IMU1_Y_gt = IMU1_Y_raw.copy()
IMU1_Z_gt = IMU1_Z_raw.copy()
IMU2_X_gt = IMU1_X_raw.copy()
IMU2_Y_gt = IMU1_Y_raw.copy()
IMU2_Z_gt = IMU1_Z_raw.copy()

thresh = 0.25
for i in range(len(IMU1_X)):
    if IMU1_X_raw[i]>-thresh and IMU1_X_raw[i]<thresh:
        IMU1_X_gt[i] = 0
    if IMU1_Y[i]>-thresh and IMU1_Y_raw[i]<thresh:
        IMU1_Y_gt[i] = 0
    if IMU1_Z_raw[i]>-thresh and IMU1_Z_raw[i]<thresh:
        IMU1_Z_gt[i] = 0
    if IMU2_X_raw[i]>-thresh and IMU2_X_raw[i]<thresh:
        IMU2_X_gt[i] = 0
    if IMU2_Y_raw[i]>-thresh and IMU2_Y_raw[i]<thresh:
        IMU2_Y_gt[i] = 0
    if IMU2_Z_raw[i]>-thresh and IMU2_Z_raw[i]<thresh:
        IMU2_Z_gt[i] = 0

IMU1_X_gt = np.array(IMU1_X_gt) 
IMU1_Y_gt = np.array(IMU1_Y_gt) 
IMU1_Z_gt = np.array(IMU1_Z_gt) 
IMU2_X_gt = np.array(IMU2_X_gt) 
IMU2_Y_gt = np.array(IMU2_Y_gt) 
IMU2_Z_gt = np.array(IMU2_Z_gt) 

#-------INTGR => SPEED-------
meanX = (IMU1_X_gt + IMU2_X_gt)/2
meanY = (IMU1_Y_gt + IMU2_Y_gt)/2
speedX_mean_simpson = [0]
speedY_mean_simpson = [0]

speedX_IMU1_simpson = [0]
speedY_IMU1_simpson = [0]

dt = 0.02
for i in range(0,len(meanX)-1):
    speedX_mean_simpson.append(scipy.integrate.simps(meanX[:i+1], dx=dt))
    speedY_mean_simpson.append(scipy.integrate.simps(meanY[:i+1], dx=dt))
    speedX_IMU1_simpson.append(scipy.integrate.simps(IMU1_X[:i+1], dx=dt))
    speedY_IMU1_simpson.append(scipy.integrate.simps(IMU1_Y[:i+1], dx=dt))

speedX_mean_simpson = np.array(speedX_mean_simpson)
speedY_mean_simpson = np.array(speedY_mean_simpson)
speedX_IMU1_simpson = np.array(speedX_IMU1_simpson)
speedY_IMU1_simpson = np.array(speedY_IMU1_simpson)

print(len(IMU1_X))
a, b = 4350, len(speedX_IMU1_simpson)

time_shrink = np.hstack((time[200:1300],time[4350::]))
speedX_IMU1_simpson_shrink = np.hstack((speedX_IMU1_simpson[200:1300],speedX_IMU1_simpson[4350::]))
coeff = np.polyfit(time_shrink, speedX_IMU1_simpson_shrink,1)
print(coeff)
if len(coeff)==2:
    lin_reg = coeff[0]*time+coeff[1]
elif len(coeff)==3:
    lin_reg = coeff[0]*time**2+coeff[1]*time+coeff[2]
speedX_IMU1_simpson_corr = speedX_IMU1_simpson - lin_reg

#-------INTGR => SPEED-------
posX_mean_simpson = [0]
posY_mean_simpson = [0]
for i in range(0,len(meanX)-1):
    posX_mean_simpson.append(scipy.integrate.simps(speedX_mean_simpson[:i+3]))
    posY_mean_simpson.append(scipy.integrate.simps(speedY_mean_simpson[:i+3]))


graph()


