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
global jp

joints = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw']
position0 = None
psm_handle = None
_client = None
joint_states = None
jp = None

def callback0(data):
    global joint_states
    global joints
    psm_handle_cmd_type = ObjectCmd()
    psm_handle_cmd_type.enable_position_controller = False
    for joint_i, joint in enumerate(joints):
        index = data.name.index(joint)
        msg = [data.position[index], data.velocity[index], data.effort[index]]
        joint_states[joint] = msg

    # for joint_i, joint in enumerate(joints):
    #     psm_handle_cmd_type.joint_cmds.append(joint_states[joint][0])
    #     psm_handle_cmd_type.position_controller_mask.append(False)
    #
    # psm_handle_cmd_type.header.stamp = rospy.Time()
    # psm_handle_cmd_type.publish_children_names = True
    # psm_handle_cmd_type.publish_joint_names = True
    # psm_handle_cmd_type.publish_joint_positions = True

    # jp.publish(psm_handle_cmd_type)
    psm_handle.set_joint_pos(0,joint_states[joints[0]][0])
    psm_handle.set_joint_pos(1,joint_states[joints[1]][0])
    psm_handle.set_joint_pos(2,joint_states[joints[2]][0])
    psm_handle.set_joint_pos(3,joint_states[joints[3]][0])
    psm_handle.set_joint_pos(4,joint_states[joints[4]][0])
    psm_handle.publish_joint_positions = True


    # psm_handle = _client.get_obj_handle('psm/baselink')
    # Header header
    # bool enable_position_controller
    # geometry_msgs/Pose pose
    # geometry_msgs/Wrench wrench
    # float32[] joint_cmds
    # bool[] position_controller_mask
    # bool publish_children_names
    # bool publish_joint_names
    # bool publish_joint_positions




    # psm_handle.set_joint_pos(0,-0.4821533815364588)
    # psm_handle.set_joint_pos(1,-0.27399483964461135)
    # psm_handle.set_joint_pos(2,0.12995825766000002)
    # psm_handle.set_joint_pos(3,0.10556933567190421)
    # psm_handle.set_joint_pos(4,-0.6088681253046674)



    # position0 = data.position[0]

    # return position0

def callback1(data):
    rospy.loginfo(data.position[1])

def init_client():
    global psm_handle
    global _client
    global joint_states
    global joints
    _client = Client()
    _client.connect()
    psm_handle = _client.get_obj_handle('psm/maininsertionlink')
    rospy.loginfo(psm_handle.get_children_names())
    # psm_handle.set_joint_effort(0,0.5)
    joint_states = {joint: [1., 2., 3.] for joint in joints}

def init_ros():
    rospy.init_node('joint_state_listener')

def main():
    global position0
    global _client
    global joint_states
    global joints
    global psm_handle
    global jp
    init_client()
    # _client = Client()
    # _client.connect()
    # psm_handle = _client.get_obj_handle('psm/baselink')
    # psm_handle.set_joint_effort(0,0.5)
    # rospy.Subscriber("/dvrk/PSM1/state_joint_current",JointState,callback0)
    # init_ros()
    # _client = Client()
    # _client.connect()
    # connect to psm
    # rospy.init_node('joint_state_listener')

    # joint positions test
    # psm_handle.set_joint_effort(0,0.5)
    # a = None
    # while a is None:
        # try:
        # pos1_sub = rospy.Subscriber("/dvrk/PSM1/state_joint_current",JointState,callback1)
    # print('ambf set')

    # rospy.spin()
        # a = None
    raw_input()
    _client.clean_up()

        # except KeyboardInterrupt:
        #     print("keyboard interrupt")
        #     _client.clean_up()
        #     exit(0)
        #     a = False

    # raw_input("press anything to clean up...")
    # _client.clean_up()

if __name__ == '__main__':
    main()
