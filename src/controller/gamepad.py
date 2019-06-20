#!/usr/bin/env python

from __future__ import print_function
import rospy
from inputs import get_gamepad
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

class Gamepad():
    def __init__(self, pwm_max=1550, pwm_neutral=1500, gain_pwm_cam = 400, gain_light_inc=50, gain_light_max=1900, rosrate=4):
        self.pub = rospy.Publisher('/Command/joy', Joy, queue_size=10)
        self.sub = rospy.Subscriber('/BlueRov2/arm', Bool, self._arm_callback) 
        self.rate = rospy.Rate(rosrate)
        self.model_base_link = '/base_link'

        self.pwm_max = pwm_max
        self.pwm_neutral = pwm_neutral
        self.gain_pwm_cam = gain_pwm_cam  

        self.override_controller = 0 # 1 to override all pwm sended by controllers
        self.armed = False # False if BlueRov2 disarmed, True if armed 
        self.gain_light = 1100
        self.gain_light_inc = gain_light_inc
        self.gain_light_max = gain_light_max
        #sensor_msgs/Joy :
        #  std_msgs/Header header
        #    uint32 seq
        #    time stamp
        #    string frame_id
        #  float32[] axes -> [THROTTLE, YAW, FORWARD, LATERAL]
        #  int32[] buttons ->[ARM, OVERRIDE_CONTROLLER, PWM_CAM]
        self.msg = Joy()
        self.msg.axes = [self.pwm_neutral, 
                self.pwm_neutral, 
                self.pwm_neutral,
                self.pwm_neutral]

        self.msg.buttons = [self.armed, 
                self.override_controller, 
                self.pwm_neutral, 
                self.gain_light]
        
        #device : logitech gamepad F310
        self.input = {'ABS_Y': self._throttle, # LEFT soick vertical [0-255], 128 = neutral
            'ABS_X': self._lateral, # LEFT stick horizontal [0-255], 128 = neutral
            
            'ABS_RZ': self._forward, # RIGHT stick vertical [0-255], 128 = neutral
            'ABS_Z': self._yaw,  # RIGHT stick horizontal [0-255], 128 = neutral
            
            'ABS_HAT0Y': self._NDEF, # LEFT cross vertical -1 left, up ; +1 right, down
            'ABS_HAT0X': self._set_gain_light, # LEFT cross horizontal [-1;0;1]
            
            'BTN_TRIGGER': self._override_controller, # X [0;1]
            'BTN_TOP1': self._NDEF, # Y [0;1]
            'BTN_THUMB': self._NDEF, # A [0;1]
            'BTN_THUMB2': self._NDEF, # A [0,1]

            'BTN_BASE': self._NDEF, # LT [0;1]
            'BTN_BASE2': self._NDEF, # RT [0;1]
            'BTN_TOP2': self._cam_down, # LB [0;1]
            'BTN_PINKIE': self._cam_up, # RB [0;1]
            
            'BTN_BASE3': self._disarm, # BACK [0;1]
            'BTN_BASE4': self._arm, # START [0;1]
            }

    def _arm_callback(self, msg):
        self.armed = msg.data
        print('ARRMM ', self.armed)    

    def msg_header(self):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = self.model_base_link

    def _NDEF(self, key, state):
        print('{} not BIND, state : {}'.format(key, state))

    def _arm(self, key, state):
        if not self.armed :
            self.armed = True
        self.msg.buttons[0] = self.armed
        print("ARM, key : {}, state : {}, arm : {}".format(key, state, self.armed))

    def _disarm(self, key, state):
        if self.armed:
            self.armed = False 
        self.msg.buttons[0] = self.armed
        print("DISARM, key : {}, state : {}, arm : {}".format(key, state, self.armed))

    def _override_controller(self, key, state):
        if state == 1 and self.override_controller == 0:
            self.override_controller = 1
        elif state == 1 and self.override_controller == 1 :
            self.override_controller = 0
        self.msg.buttons[1] = self.override_controller
        print("OVERRIDE_CONTROLLER, key : {}, state, {}, override_controller : {}".format(key, state, self.override_controller))

    def _throttle(self, key, state):
        state = 255-state #to fix 255 top, 0 down (default : 0 up, 255 down)
        pwm_min = self.pwm_neutral - (self.pwm_max-self.pwm_neutral)
        pwm = pwm_min + state*((self.pwm_max-pwm_min)/255.)   #255. is the maximum of the stick need to be a float
        self.msg.axes[0] = self.pwm_set_neutral(pwm)
        print("THROTTLE, key : {}, state : {}, pwm : {}".format(key, state, self.msg.axes[0]))


    def _yaw(self, key,state):
        pwm_min = self.pwm_neutral - (self.pwm_max-self.pwm_neutral)
        pwm = pwm_min + state*((self.pwm_max-pwm_min)/255.)   #255. is the maximum of the stick need to be a float
        self.msg.axes[1] = self.pwm_set_neutral(pwm)
        print("YAW, key : {}, state : {}, pwm : {}".format(key, state, self.msg.axes[1]))

    
    def _forward(self, key, state):
        state = 255-state #to fix 255 top, 0 down (default : 0 up, 255 down)
        pwm_min = self.pwm_neutral - (self.pwm_max-self.pwm_neutral)
        pwm = pwm_min + state*((self.pwm_max-pwm_min)/255.)   #255. is the maximum of the stick need to be a float
        self.msg.axes[2] = self.pwm_set_neutral(pwm)
        print("FORWARD, key : {}, state : {}, pwm : {}".format(key, state, self.msg.axes[2]))

    def _lateral(self, key,state):
        pwm_min = self.pwm_neutral - (self.pwm_max-self.pwm_neutral)
        pwm = pwm_min + state*((self.pwm_max-pwm_min)/255.)   #255. is the maximum of the stick need to be a float
        self.msg.axes[3] = self.pwm_set_neutral(pwm)
        print("LATERAL, key : {}, state : {}, pwm : {}".format(key, state, self.msg.axes[3]))


    def _cam_up(self, key, state):
        if state == 1:
            pwm = self.pwm_neutral + self.gain_pwm_cam 
        else :
            pwm = self.pwm_neutral
        self.msg.buttons[2] = pwm
        print("CAM_UP, key : {}, state : {}, pwm : {}".format(key, state, self.msg.buttons[2]))

    def _cam_down(self, key, state):
        if state == 1:
            pwm = self.pwm_neutral - self.gain_pwm_cam 
        else :
            pwm = self.pwm_neutral
        self.msg.buttons[2] = pwm
        print("CAM_DOWN, key : {}, state : {}, pwm : {}".format(key, state, self.msg.buttons[2]))
    
    def _set_gain_light(self, key, state):
        if state == 1:
            self.gain_light += self.gain_light_inc 
            if self.gain_light > self.gain_light_max:
                self.gain_light = self.gain_light_max
        elif state == -1:
            self.gain_light -= self.gain_light_inc 
            if self.gain_light < 0:
                self.gain_light = 0
        else:
            pass
        self.msg.buttons[3] = self.gain_light
        print("SET_GAIN_LIGHT, key : {}, state : {}, gain_light : {}".format(key, state, self.msg.buttons[3]))

    def pwm_set_neutral(self,pwm):
        if pwm >= 1495 and pwm <= 1505: #to ensure that when stick are in the center neutral pwm i send
            pwm = self.pwm_neutral
	return int(pwm)

    def publish(self):
        events = get_gamepad()
        for event in events:
            #print(event.ev_type, event.code, event.state)
            if event.code in self.input:
                self.input[event.code](event.code,event.state) #launch the method that correspond to the input 
        self.msg_header()
        self.pub.publish(self.msg)

if __name__ == "__main__":
    rospy.init_node('Gamepad', anonymous=True)

    gamepad = Gamepad()

    while not rospy.is_shutdown():
        gamepad.publish()

