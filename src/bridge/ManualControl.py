#!/usr/bin/env python
"""Simple example showing how to get gamepad events."""

from __future__ import print_function

from inputs import get_gamepad


def _NDEF(key, state):
    print('{} not BIND, state : {}'.format(key, state))

def _arm(key, state):
    arm = 1
    print("ARM, key : {}, state : {}, arm : {}".format(key, state, arm))

def _disarm(key, state):
    disarm = 1
    print("DISARM, key : {}, state : {}, arm : {}".format_map(key, state, disarm))

def _forward(key, state):
    pwm = -1
    print("FORWARD, key : {}, state : {}, pwm : {}".format(key, state, pwm))

def _yaw(key,state):
    pwm = -1
    print("YAW, key : {}, state : {}, pwm : {}".format(key, state, pwm))

def _throttle(key, state):
    pwm = -1
    print("THROTTLE, key : {}, state : {}, pwm : {}".format(key, state, pwm))



def main():
    """Just print out some event infomation when the gamepad is used."""
    #device : logitech gamepad F310
    INPUT_TYPE ={'ABS_Y': _NDEF, # LEFT stick vertical [0-255], 128 = neutral
        'ABS_X': _NDEF, # LEFT stick horizontal [0-255], 128 = neutral
        
        'ABS_RZ': _forward, # RIGHT stick vertical [0-255], 128 = neutral
        'ABS_Z': _yaw,  # RIGHT stick horizontal [0-255], 128 = neutral
        
        'ABS_HATOY': _NDEF, # LEFT cross vertical -1 left, up ; +1 right, down
        'ABS_HATOX': _NDEF, # LEFT cross horizontal [-1;0;1] 
        
        'BTN_TRIGGER': _NDEF, # X [0;1]
        'BTN_TOP1': _NDEF, # Y [0;1]
        'BTN_THUMB': _NDEF, # A [0;1]
        'BTN_THUMB2': _NDEF, # A [0,1]

        'BTN_BASE': _NDEF, # LT [0;1]
        'BTN_BASE2': _NDEF, # RT [0;1]
        'BTN_TOP2': _NDEF, # LB [0;1]
        'BTN_PINKIE': _NDEF, # RB [0;1]
        
        'BTN_BASE3': _disarm, # BACK [0;1]
        'BTN_BASE4': _arm, # START [0;1]
        }

    while 1:
        events = get_gamepad()
        for event in events:
            print(event.ev_type, event.code, event.state)
            if event.code in INPUT_TYPE:
                INPUT_TYPE[event.code](event.code,event.state)  

if __name__ == "__main__":
    main()

