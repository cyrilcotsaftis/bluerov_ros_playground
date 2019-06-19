#!/usr/bin/env python

from __future__ import print_function
import rospy
from inputs import get_gamepad
from sensor_msgs.msg import Joy

class Gamepad():
    def __init__(self, pwmmax=1550, pwmneutral=1500,rosrate = 10):
        self.pub = rospy.Publisher('/Command/joy', Joy, queue_size=10)
        self.rate = rospy.Rate(rosrate)
        self.model_base_link = '/base_link'

        self.pwm_max = pwmmax
        self.pwm_neutral = pwmneutral
        self.override_controller = 0 # 1 to override all pwm sended by controllers
        self.armed = 0 # 0 if BlueRov2 disarmed, 1 if armed TODO: subscribe to the channel armed

        #sensor_msgs/Joy :
        #  std_msgs/Header header
        #    uint32 seq
        #    time stamp
        #    string frame_id
        #  float32[] axes -> [THROTTLE, YAW, FORWARD]
        #  int32[] buttons ->[ARM, OVERRIDE_CONTROLLER, ]
        self.msg = Joy()
        self.msg.axes = [self.pwm_neutral, self.pwm_neutral, self.pwm_neutral]
        self.msg.buttons = [self.armed, self.override_controller]
        
        #device : logitech gamepad F310
        self.input = {'ABS_Y': self._throttle, # LEFT soick vertical [0-255], 128 = neutral
            'ABS_X': self._NDEF, # LEFT stick horizontal [0-255], 128 = neutral
            
            'ABS_RZ': self._forward, # RIGHT stick vertical [0-255], 128 = neutral
            'ABS_Z': self._yaw,  # RIGHT stick horizontal [0-255], 128 = neutral
            
            'ABS_HATOY': self._NDEF, # LEFT cross vertical -1 left, up ; +1 right, down
            'ABS_HATOX': self._NDEF, # LEFT cross horizontal [-1;0;1] 
            
            'BTN_TRIGGER': self._override_controller, # X [0;1]
            'BTN_TOP1': self._NDEF, # Y [0;1]
            'BTN_THUMB': self._NDEF, # A [0;1]
            'BTN_THUMB2': self._NDEF, # A [0,1]

            'BTN_BASE': self._NDEF, # LT [0;1]
            'BTN_BASE2': self._NDEF, # RT [0;1]
            'BTN_TOP2': self._NDEF, # LB [0;1]
            'BTN_PINKIE': self._NDEF, # RB [0;1]
            
            'BTN_BASE3': self._disarm, # BACK [0;1]
            'BTN_BASE4': self._arm, # START [0;1]
            }

    def update():
        #TODO: goal to subscribe to the order topic to set pwm_max and to get the arm state of blue rov
        pass

    
    def msg_header(self):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = self.model_base_link

    def _NDEF(self, key, state):
        print('{} not BIND, state : {}'.format(key, state))

    def _arm(self, key, state):
        if self.armed == 0:
            self.armed = 1
        self.msg.buttons[0] = self.armed
        print("ARM, key : {}, state : {}, arm : {}".format(key, state, self.armed))

    def _disarm(self, key, state):
        if self.armed == 1:
            self.armed = 0
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

    def pwm_set_neutral(self,pwm):
        if pwm >= 1495 and pwm <= 1505:
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
        #gamepad.rate.sleep()

