import csv
import sys
import matplotlib.pyplot as plt
import numpy as np
from numpy.fft import fft, fftfreq
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
    plt.plot(time, IMU1_X,'r', label="Normed")
    plt.plot(time, IMU1_X_gt,'g', label="Bounded")
    plt.plot(time, IMU1_X_raw,',b', label="Raw")
    #plt.plot(time, IMU1_X_raw_shift,'+g', label="Raw shift")
    #plt.plot(time, IMU1_X_raw_mean,'k', label=" avg = {}".format(IMU1_X_raw_mean[0]))
    #plt.plot(time, IMU1_X_raw_shift_mean,'k', label=" avg = {}".format(IMU1_X_raw_shift_mean[0]))

    plt.title("{} X  IMU1".format(sensor))
    plt.ylabel("{}".format(unit))
    plt.legend()

    plt.subplot(3,1,2)
    plt.plot(time, IMU1_Y,'r', label="Normed")
    plt.plot(time, IMU1_Y_gt,'g', label="Bounded")
    plt.plot(time, IMU1_Y_raw,',b', label="Raw")
    #plt.plot(time, IMU1_Y_raw_shift,'+g', label="Raw shift")
    #plt.plot(time, IMU1_Y_raw_mean,'k', label=" avg = {}".format(IMU1_Y_raw_mean[0]))
    #plt.plot(time, IMU1_Y_raw_shift_mean,'k', label=" avg = {}".format(IMU1_Y_raw_shift_mean[0]))
    #plt.plot(time, IMU1_Y_raw_flt, "m", label="flt")
    plt.title("{} Y IMU1".format(sensor))
    plt.ylabel("{}".format(unit))
    plt.legend()

    plt.subplot(3,1,3)
    plt.plot(time, IMU1_Z,'r', label="Normed")
    plt.plot(time, IMU1_Z_gt,'g', label="Bounded")
    plt.plot(time, IMU1_Z_raw,',b', label="Raw")
    #plt.plot(time, IMU1_Z_raw_shift,'+g', label="Raw shift")
    #plt.plot(time, IMU1_Z_raw_mean,'k', label=" avg = {}".format(IMU1_Z_raw_mean[0]))
    #plt.plot(time, IMU1_Z_raw_shift_mean,'k', label=" avg = {}".format(IMU1_Z_raw_shift_mean[0]))
    plt.title("{} Z IMU1".format(sensor))
    plt.xlabel("Time (s)")
    plt.ylabel("{}".format(unit))
    plt.legend()

    plt.figure(2)

    plt.subplot(3,1,1)
    plt.plot(time, IMU2_X,'r', label="Normed")
    plt.plot(time, IMU2_X_gt,'g', label="Bounded")
    plt.plot(time, IMU2_X_raw,',b', label="Raw")
    #plt.plot(time, IMU2_X_raw_shift,'+g', label="Raw shift")
    #plt.plot(time, IMU2_X_raw_mean,'k', label=" avg = {}".format(IMU2_X_raw_mean[0]))
    #plt.plot(time, IMU2_X_raw_shift_mean,'k', label=" avg = {}".format(IMU2_X_raw_shift_mean[0]))
    plt.title("{} X IMU2".format(sensor))
    plt.ylabel("{}".format(unit))
    plt.legend()

    plt.subplot(3,1,2)
    plt.plot(time, IMU2_Y,'r', label="Normed")
    plt.plot(time, IMU2_Y_gt,'g', label="Bounded")
    plt.plot(time, IMU2_Y_raw,',b', label="Raw")
    #plt.plot(time, IMU2_Y_raw_shift,'+g', label="Raw shift")
    #plt.plot(time, IMU2_Y_raw_mean,'k', label=" avg = {}".format(IMU2_Y_raw_mean[0]))
    #plt.plot(time, IMU2_Y_raw_shift_mean,'k', label=" avg = {}".format(IMU2_Y_raw_shift_mean[0]))
    plt.title("{} Y IMU2".format(sensor))
    plt.ylabel("{}".format(unit))
    plt.legend()

    plt.subplot(3,1,3)
    plt.plot(time, IMU2_Z,'r', label="Normed")
    plt.plot(time, IMU2_Z_gt,'g', label="Bounded")
    plt.plot(time, IMU2_Z_raw,',b', label="Raw")
    #plt.plot(time, IMU2_Z_raw_shift,'+g', label="Raw shift")
    #plt.plot(time, IMU2_Z_raw_mean,'k', label=" avg = {}".format(IMU2_Z_raw_mean[0]))
    #plt.plot(time, IMU2_Z_raw_shift_mean,'k', label=" avg = {}".format(IMU2_Z_raw_shift_mean[0]))
    plt.title("{} Z IMU2".format(sensor))
    plt.xlabel("Time (s)")
    plt.ylabel("{}".format(unit))
    plt.legend()
    
    if sensor == "Linear Acceleration":
        plt.figure(3)
        plt.subplot(3,2,1)
        plt.plot(time, IMU1_X_gt, 'r', label="IMU1")
        plt.plot(time, IMU2_X_gt, 'b', label="IMU2")
        plt.ylabel("{}".format(unit))
        plt.title("{} X".format(sensor))
        plt.legend()
        plt.subplot(3,2,3)
        plt.plot(time, speedX_IMU1_simpson,'r', label="IMU1")
        plt.plot(time, speedX_IMU2_simpson,'b', label="IMU2")
        plt.plot(time, speedX_mean_simpson,'k', label="IMU1+IMU2")
        plt.plot(time, (speedX_IMU1_simpson + speedX_IMU2_simpson)/2,'g', label="IMU1+IMU2")
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
        plt.plot(time, speedY_IMU1_simpson,'r', label="IMU1")
        plt.plot(time, speedY_IMU2_simpson,'b', label="IMU2")
        plt.plot(time, speedY_mean_simpson,'k', label="IMU1+IMU2")
        plt.plot(time, (speedY_IMU1_simpson + speedY_IMU2_simpson)/2,'g', label="IMU1+IMU2")
        plt.title("Y Speed")
        plt.xlabel("Time (s)")
        plt.ylabel("Speed (m.s^-1)")
        plt.legend()
        plt.subplot(3,2,5)
        plt.plot(posX_IMU1_simpson, posY_IMU1_simpson, 'g', label="")
        plt.title("X Position")
        plt.xlabel("Pos (m)")
        plt.ylabel("Pos (m)")
        plt.legend()
        plt.subplot(3,2,6)
        plt.plot(posX_IMU2_simpson, posY_IMU2_simpson,'g', label="")
        plt.title("Y Position")
        plt.xlabel("Pos (m)")
        plt.ylabel("Pos (m)")
        plt.legend()
    plt.figure(4)
    plt.subplot(311)
    plt.plot(freqs[mask], fft_th_IMU1_X_raw[mask])
    plt.title("{} X IMU1 FFT".format(sensor))
    plt.xlabel("Freqs (Hz)")
    plt.ylabel("abs")
    plt.subplot(312)
    plt.plot(freqs[mask], fft_th_IMU1_Y_raw[mask])
    plt.title("{} Y IMU1 FFT".format(sensor))
    plt.xlabel("Freqs (Hz)")
    plt.ylabel("abs")
    plt.subplot(313)
    plt.plot(freqs[mask], fft_th_IMU1_Z_raw[mask])
    plt.title("{} Z IMU1 FFT".format(sensor))
    plt.xlabel("Freqs (Hz)")
    plt.ylabel("abs")

    plt.figure(5)
    plt.subplot(311)
    plt.plot(freqs[mask], fft_th_IMU2_X_raw[mask])
    plt.title("{} X IMU2 FFT".format(sensor))
    plt.xlabel("Freqs (Hz)")
    plt.ylabel("abs")
    plt.subplot(312)
    plt.plot(freqs[mask], fft_th_IMU2_Y_raw[mask])
    plt.title("{} Y IMU2 FFT".format(sensor))
    plt.xlabel("Freqs (Hz)")
    plt.ylabel("abs")
    plt.subplot(313)
    plt.plot(freqs[mask], fft_th_IMU2_Z_raw[mask])
    plt.title("{} Z IMU2 FFT".format(sensor))
    plt.xlabel("Freqs (Hz)")
    plt.ylabel("abs")
    plt.show()


#-------To zeros--------
shp = IMU1_X.shape
IMU1_X_gt = IMU1_X.copy()
IMU1_Y_gt = IMU1_Y.copy()
IMU1_Z_gt = IMU1_Z.copy()
IMU2_X_gt = IMU1_X.copy()
IMU2_Y_gt = IMU1_Y.copy()
IMU2_Z_gt = IMU1_Z.copy()

thresh = 0.25
for i in range(len(IMU1_X)):
    if IMU1_X[i]>-thresh and IMU1_X[i]<thresh:
        IMU1_X_gt[i] = 0
    if IMU1_Y[i]>-thresh and IMU1_Y[i]<thresh:
        IMU1_Y_gt[i] = 0
    if IMU1_Z[i]>-thresh and IMU1_Z[i]<thresh:
        IMU1_Z_gt[i] = 0
    if IMU2_X[i]>-thresh and IMU2_X[i]<thresh:
        IMU2_X_gt[i] = 0
    if IMU2_Y[i]>-thresh and IMU2_Y[i]<thresh:
        IMU2_Y_gt[i] = 0
    if IMU2_Z[i]>-thresh and IMU2_Z[i]<thresh:
        IMU2_Z_gt[i] = 0

IMU1_X_gt = np.array(IMU1_X_gt) 
IMU1_Y_gt = np.array(IMU1_Y_gt) 
IMU1_Z_gt = np.array(IMU1_Z_gt) 
IMU2_X_gt = np.array(IMU2_X_gt) 
IMU2_Y_gt = np.array(IMU2_Y_gt) 
IMU2_Z_gt = np.array(IMU2_Z_gt) 

#-------FFT -------------
n =len(IMU1_X_raw)
freqs = fftfreq(n)
idx = np.argsort(freqs)
mask = freqs > 0
#fft values
fft_IMU1_X_raw = fft(IMU1_X_raw)
fft_IMU1_Y_raw = fft(IMU1_Y_raw)
fft_IMU1_Z_raw = fft(IMU1_Z_raw)
fft_IMU2_X_raw = fft(IMU2_X_raw)
fft_IMU2_Y_raw = fft(IMU2_Y_raw)
fft_IMU2_Z_raw = fft(IMU2_Z_raw)
# true theorical fft
fft_th_IMU1_X_raw = 2.0*np.abs(fft_IMU1_X_raw)
fft_th_IMU1_Y_raw = 2.0*np.abs(fft_IMU1_Y_raw)
fft_th_IMU1_Z_raw = 2.0*np.abs(fft_IMU1_Z_raw)
fft_th_IMU2_X_raw = 2.0*np.abs(fft_IMU2_X_raw)
fft_th_IMU2_Y_raw = 2.0*np.abs(fft_IMU2_Y_raw)
fft_th_IMU2_Z_raw = 2.0*np.abs(fft_IMU2_Z_raw)


#-------INTGR => SPEED-------
meanX = (IMU1_X_gt + IMU2_X_gt)/2
meanY = (IMU1_Y_gt + IMU2_Y_gt)/2
speedX_mean_simpson = [0]
speedY_mean_simpson = [0]
speedX_IMU1_simpson = [IMU1_X_gt[0]+IMU1_X_gt[0]+IMU1_X_gt[1]]
speedX_IMU2_simpson = [IMU2_X_gt[0]+IMU2_X_gt[0]+IMU2_X_gt[1]]
speedY_IMU1_simpson = [IMU1_Y_gt[0]+IMU1_Y_gt[0]+IMU1_Y_gt[1]]
speedY_IMU2_simpson = [IMU2_Y_gt[0]+IMU2_Y_gt[0]+IMU2_Y_gt[1]]
dt = 0.02
for i in range(0,len(meanX)-1):
    speedX_mean_simpson.append(speedX_mean_simpson[i] + (meanX[i]+meanX[i+1])/2 + meanX[i]+meanX[i+1]) 
    speedY_mean_simpson.append(speedY_mean_simpson[i] + (meanY[i]+meanY[i+1])/2 + meanY[i]+meanY[i+1]) 
    speedX_IMU1_simpson.append(speedX_IMU1_simpson[i-1] + IMU1_X_gt[i]+IMU1_X_gt[i]+IMU1_X_gt[i+1])
    speedX_IMU2_simpson.append(speedX_IMU2_simpson[i-1] + IMU2_X_gt[i]+IMU2_X_gt[i]+IMU2_X_gt[i+1])
    speedY_IMU1_simpson.append(speedY_IMU1_simpson[i-1] + IMU1_Y_gt[i]+IMU1_Y_gt[i]+IMU1_Y_gt[i+1])
    speedY_IMU2_simpson.append(speedY_IMU2_simpson[i-1] + IMU2_Y_gt[i]+IMU2_Y_gt[i]+IMU2_Y_gt[i+1])
speedX_mean_simpson = np.array(speedX_mean_simpson)*(dt/3)
speedY_mean_simpson = np.array(speedY_mean_simpson)*(dt/3)
speedX_IMU1_simpson = np.array(speedX_IMU1_simpson)*(dt/3)
speedX_IMU2_simpson = np.array(speedX_IMU2_simpson)*(dt/3)
speedY_IMU1_simpson = np.array(speedY_IMU1_simpson)*(dt/3)
speedY_IMU2_simpson = np.array(speedY_IMU2_simpson)*(dt/3)

#-------INTGR => SPEED-------
posX_IMU1_simpson = [speedX_IMU1_simpson[0]+speedX_IMU1_simpson[0]+speedX_IMU1_simpson[1]]
posX_IMU2_simpson = [speedX_IMU2_simpson[0]+speedX_IMU2_simpson[0]+speedX_IMU2_simpson[1]]
posY_IMU1_simpson = [speedY_IMU1_simpson[0]+speedY_IMU1_simpson[0]+speedY_IMU1_simpson[1]]
posY_IMU2_simpson = [speedY_IMU2_simpson[0]+speedY_IMU2_simpson[0]+speedY_IMU2_simpson[1]]
dt = 0.01
for i in range(0,len(IMU1_X)-1):
    posX_IMU1_simpson.append(posX_IMU1_simpson[i-1] + speedX_IMU1_simpson[i]+speedX_IMU1_simpson[i]+speedX_IMU1_simpson[i+1])
    posX_IMU2_simpson.append(posX_IMU2_simpson[i-1] + speedX_IMU2_simpson[i]+speedX_IMU2_simpson[i]+speedX_IMU2_simpson[i+1])
    posY_IMU1_simpson.append(posY_IMU1_simpson[i-1] + speedY_IMU1_simpson[i]+speedY_IMU1_simpson[i]+speedY_IMU1_simpson[i+1])
    posY_IMU2_simpson.append(posY_IMU2_simpson[i-1] + speedY_IMU2_simpson[i]+speedY_IMU2_simpson[i]+speedY_IMU2_simpson[i+1])
posX_IMU1_simpson = np.array(posX_IMU1_simpson)*(dt/3)
posX_IMU2_simpson = np.array(posX_IMU2_simpson)*(dt/3)
posY_IMU1_simpson = np.array(posY_IMU1_simpson)*(dt/3)

graph()


