# !/usr/bin/env Python

from ambf_client import Client

_client = Client()
_client.connect()

psm_handle = _client.get_obj_handle('psm/baselink')

psm_handle.set_joint_pos(0,-0.4821533815364588)
psm_handle.set_joint_pos(1,-0.27399483964461135)
psm_handle.set_joint_pos(2,0.12995825766000002)
psm_handle.set_joint_pos(3,0.10556933567190421)
psm_handle.set_joint_pos(4,-0.6088681253046674)

print(psm_handle.get_children_names())

raw_input("press any key to continue...")
_client.clean_up()
