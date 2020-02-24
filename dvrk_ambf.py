# !/usr/env/bin/python
# import ambf_client and connect the controller
from ambf_client import Client
from ambf_msgs.msg import ObjectCmd
from sensor_msgs.msg import JointState
import time
import rospy

global psm_handle
global position0
global _client
global joint_states
global joints

joints = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw']
position0 = None
psm_handle = None
_client = None
joint_states = None

def dvrkCb(data):
    global joint_states
    global joints
    global psm_handle
    for joint_i, joint in enumerate(joints):
        index = data.name.index(joint)
        msg = [data.position[index], data.velocity[index], data.effort[index]]
        joint_states[joint] = msg

    psm_handle.set_joint_pos(0,joint_states[joints[0]][0])
    psm_handle.set_joint_pos(1,joint_states[joints[1]][0])
    psm_handle.set_joint_pos(2,joint_states[joints[2]][0])
    psm_handle.set_joint_pos(3,joint_states[joints[3]][0])
    psm_handle.set_joint_pos(4,joint_states[joints[4]][0])

    # rospy.loginfo(joint_states[joints[0]][0])


def callback1(data):
    rospy.loginfo(data.position[1])

def init_client():
    global psm_handle
    global _client
    global joint_states
    global joints
    _client = Client()
    _client.connect()
    psm_handle = _client.get_obj_handle('psm/baselink')
    rospy.loginfo(psm_handle.get_children_names())
    # psm_handle.set_joint_effort(0,0.5)
    # joint_states = {joint: [1., 2., 3.] for joint in joints}


def main():
    global position0
    global _client
    global joint_states
    global joints
    global psm_handle

    _client = Client()
    _client.connect()
    psm_handle = _client.get_obj_handle('psm/baselink')
    time.sleep(0.1)
    # rospy.loginfo(psm_handle.get_children_names())
    print(psm_handle.get_children_names())

    # psm_handle.set_joint_pos(0, 0)
    # psm_handle.set_joint_pos(1, 0)
    # psm_handle.set_joint_pos(2, 0)
    # psm_handle.set_joint_pos(3, 0)
    # psm_handle.set_joint_pos(4, 0)

    joint_states = {joint: [1., 2., 3.] for joint in joints}

    a = None
    while a == None:
        rospy.Subscriber("/dvrk/PSM1/state_joint_current",JointState,dvrkCb)

        # print("joint1: ", joint_states[joints[0]][0])
        # print("joint2: ", joint_states[joints[1]][0])
        # print("joint3: ", joint_states[joints[2]][0])
        # print("joint4: ", joint_states[joints[3]][0])
        # print("joint5: ", joint_states[joints[4]][0])
        a = raw_input()
        _client.clean_up()

if __name__ == '__main__':
    main()
