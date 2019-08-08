To deal with the Two IMUs adafruit LSM9DS1 with a Raspberry Pi 3 (Debian Jessi):
    
    connect the imus on the I2C bus, don't forget to connect the SDOM and SDOAG pin of one IMU to the ground
    
    Prerequisite :
        sudo apt-get update
        sudo apt-get upgrade
        sudo apt-get python3-pip #if pip is not already installed
        
        sudo apt-get python-smbus
        sudo apt-get i2c-tools
        
        pip3 install RPi.GPIO #a sudo may be necessary
        pip3 install --upgrade setuptools #a sudo may be necessary
        pip3 install adafruit-circuitpython-lsm9ds1 #a sudo may be necessary
     
     Now we are ready to deal with one IMU (the one with has not his SDOM and SDOAG pin connected to the ground) => see exemple to use the lib
     
     In order to deal with the two IMUs on the same I2C bus replace the adafruit_lsm9ds1.py file in /usr/local/lib/python3.4/dist-packages/ by the file adafruit_lsm9ds1.py from the folder tools/lib2imus/:
        
            sudo cp ~/imu_i2c/lib2imus/adafruit_lsm9ds1.py /usr/local/lib/python3.4/dist-packages/

    There is an example reading and displaying IMUs data in the folder tools/
    
To publish IMUs values on ROS topics : 
    start serverIMU.py on Raspberry Pi 3 : python3 server.py
    start Imu_bridge.py on topside computer : python Imu_bridge.py


