import sys
import time
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

class IMU_listener():
    def __init__(self):
        rospy.Subscriber('/BlueRov2/imu/imu1', Imu, self._callback_imu_1)
        rospy.Subscriber('/BlueRov2/imu/imu2', Imu, self._callback_imu_2)
        rospy.Subscriber('/BlueRov2/imu/mag1', MagneticField, self._callback_mag_1)
        rospy.Subscriber('/BlueRov2/imu/mag2', MagneticField, self._callback_mag_2)
        rospy.Subscriber('/BlueRov2/imu/imu1_raw', Imu, self._callback_imu_1_raw)
        rospy.Subscriber('/BlueRov2/imu/imu2_raw', Imu, self._callback_imu_2_raw)
        rospy.Subscriber('/BlueRov2/imu/mag1_raw', MagneticField, self._callback_mag_1_raw)
        rospy.Subscriber('/BlueRov2/imu/mag2_raw', MagneticField, self._callback_mag_2_raw)

        self.rosrate = 100
        self.rate = rospy.Rate(self.rosrate)
        self.acc_imu1 = Imu()
        self.acc_imu2 = Imu()
        self.mag_imu1 = MagneticField()
        self.mag_imu2 = MagneticField()
        self.acc_imu1_raw = Imu()
        self.acc_imu2_raw = Imu()
        self.mag_imu1_raw = MagneticField()
        self.mag_imu2_raw = MagneticField()


    def _callback_imu_1(self, msg):
        self.acc_imu1 = msg

    def _callback_imu_2(self, msg):
        self.acc_imu2 = msg 

    def _callback_mag_1(self, msg):
        self.mag_imu1 = msg

    def _callback_mag_2(self, msg):
        self.mag_imu2 = msg 

    def _callback_imu_1_raw(self, msg):
        self.acc_imu1_raw = msg

    def _callback_imu_2_raw(self, msg):
        self.acc_imu2_raw= msg 

    def _callback_mag_1_raw(self, msg):
        self.mag_imu1_raw = msg

    def _callback_mag_2_raw(self, msg):
        self.mag_imu2_raw = msg

    def record_data(self, t):
        mag_file = open('mag.csv', 'w')
        acc_file = open('acc.csv', 'w')
        mag_file.write('TIME, IMU1_mag_x, IMU1_mag_y,IMU1_mag_z, IMU1_mag_raw_x, IMU1_mag_raw_y, IMU1_mag_raw_z,IMU2_mag_x, IMU2_mag_y,IMU2_mag_z, IMU2_mag_raw_x, IMU2_mag_raw_y, IMU2_mag_raw_z')
        acc_file.write('TIME, IMU1_acc_x, IMU1_acc_y,IMU1_acc_z, IMU1_acc_raw_x, IMU1_acc_raw_y, IMU1_acc_raw_z,IMU2_acc_x, IMU2_acc_y,IMU2_acc_z, IMU2_acc_raw_x, IMU2_acc_raw_y, IMU2_acc_raw_z')

        t0 = time.time()
        #NOT WORKING I DON'T KNOW WHY !!!!!!
        while (time.time()-t0)<t:
            recordTime = time.time()-t0
            print("RECORD TIME : {} /{}".format(recordTime, t))
            mag_buf = '{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}\n'.format(recordTime,
                                                                                self.mag_imu1.magnetic_field.x,
                                                                                self.mag_imu1.magnetic_field.y,
                                                                                self.mag_imu1.magnetic_field.z,
                                                                                self.mag_imu1_raw.magnetic_field.x,
                                                                                self.mag_imu1_raw.magnetic_field.y,
                                                                                self.mag_imu1_raw.magnetic_field.z,
                                                                                self.mag_imu2.magnetic_field.x,
                                                                                self.mag_imu2.magnetic_field.y,
                                                                                self.mag_imu2.magnetic_field.z,
                                                                                self.mag_imu2_raw.magnetic_field.x,
                                                                                self.mag_imu2_raw.magnetic_field.y,
                                                                                self.mag_imu2_raw.magnetic_field.z)

            acc_buf = '{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}\n'.format(recordTime,
                                                                                self.acc_imu1.linear_acceleration.x,
                                                                                self.acc_imu1.linear_acceleration.y,
                                                                                self.acc_imu1.linear_acceleration.z,
                                                                                self.acc_imu1_raw.linear_acceleration.x,
                                                                                self.acc_imu1_raw.linear_acceleration.y,
                                                                                self.acc_imu1_raw.linear_acceleration.z,
                                                                                self.acc_imu2.linear_acceleration.x,
                                                                                self.acc_imu2.linear_acceleration.y,
                                                                                self.acc_imu2.linear_acceleration.z,
                                                                                self.acc_imu2_raw.linear_acceleration.x,
                                                                                self.acc_imu2_raw.linear_acceleration.y,
                                                                                self.acc_imu2_raw.linear_acceleration.z)
            mag_file.write(mag_buf)
            acc_file.write(acc_buf)
            time.sleep(0.01)
        mag_file.close()
        acc_file.close()

if __name__ == "__main__":
    rospy.init_node('imu_listener', anonymous=True)
    imu_listener = IMU_listener()
    if sys.argv[1] != None:
        print("Need record time input")
    imu_listener.record_data(int(sys.argv[1]))



