# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('udpin:192.168.2.1:14560')
# Wait a heartbeat before sending commands

master.wait_heartbeat()

for i in range(20):
    print(master.recv_match().to_dict())
# http://mavlink.org/messages/common#MAV_CMD_COMPONENT_ARM_DISARM

# Arm
# master.arducopter_arm() or:
master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)

# Disarm
# master.arducopter_disarm() or:
master.mav.command_long_send(
           master.target_system,
           master.target_component,
           mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
           0,
           0, 0, 0, 0, 0, 0, 0)
