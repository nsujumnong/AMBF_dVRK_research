#!/usr/bin/env python

from numpy import genfromtxt
import rospy
import numpy as np
import pandas as pd
#import dvrk
import geometry_msgs.msg as gm
import csv
import sys


#filename = sys.argv[1]
#print(filename)

#time = pd.read_csv(filename,usecols=[1],skiprows=7)
##rigid body
#rig1_rotx = pd.read_csv(filename,usecols=[2],skiprows=7)
#rig1_roty = pd.read_csv(filename,usecols=[3],skiprows=7)
#rig1_rotz = pd.read_csv(filename,usecols=[4],skiprows=7)
#rig1_rotw = pd.read_csv(filename,usecols=[5],skiprows=7)
#rig1_posx = pd.read_csv(filename,usecols=[6],skiprows=7)
#rig1_posy = pd.read_csv(filename,usecols=[7],skiprows=7)
#rig1_posz = pd.read_csv(filename,usecols=[8],skiprows=7)

##rigid body marker
##marker1
#rig1_mark1_x = pd.read_csv(filename,usecols=[10],skiprows=7)
#rig1_mark1_y = pd.read_csv(filename,usecols=[11],skiprows=7)
#rig1_mark1_z = pd.read_csv(filename,usecols=[12],skiprows=7)
##marker2
#rig1_mark2_x = pd.read_csv(filename,usecols=[14],skiprows=7)
#rig1_mark2_y = pd.read_csv(filename,usecols=[15],skiprows=7)
#rig1_mark2_z = pd.read_csv(filename,usecols=[16],skiprows=7)

##marker3
#rig1_mark3_x = pd.read_csv(filename,usecols=[18],skiprows=7)
#rig1_mark3_y = pd.read_csv(filename,usecols=[19],skiprows=7)
#rig1_mark3_z = pd.read_csv(filename,usecols=[20],skiprows=7)
##rig1_rot = np.array(rig1_rotx_np,rig1_roty_np)

##create array of data
#rig1_rot = np.append(rig1_rotx,rig1_roty,1)
#rig1_rot = np.append(rig1_rot,rig1_rotz,1)
#rig1_rot = np.append(rig1_rot,rig1_rotw,1)

#rig1_pos = np.append(rig1_posx,rig1_posy,1)
#rig1_pos = np.append(rig1_pos,rig1_posz,1)

#print(rig1_rot)
#print(rig1_pos)
#print(rig1_rot[0])
#print(len(rig1_rot))

#for i in range(len(rig1_rot)):
#	print(rig1_rot[i])

#print(rig1_pos.shape)	

class Arms:
    def __init__(self, names):
        self.name = names
        self.pos = np.zeros(3)
        self.rot = np.zeros(4)
        self.arm_rot = np.zeros(4)
        self.marker_data_pos = np.zeros((1, 3))
        self.marker_data_rot = np.zeros((1, 4))

	filename = "Take_2020-09-28_01.52.27_PM.csv"
	rig1_rotx = pd.read_csv(filename,usecols=[2],skiprows=7)
	rig1_roty = pd.read_csv(filename,usecols=[3],skiprows=7)
	rig1_rotz = pd.read_csv(filename,usecols=[4],skiprows=7)
	rig1_rotw = pd.read_csv(filename,usecols=[5],skiprows=7)
	rig1_posx = pd.read_csv(filename,usecols=[6],skiprows=7)
	rig1_posy = pd.read_csv(filename,usecols=[7],skiprows=7)
	rig1_posz = pd.read_csv(filename,usecols=[8],skiprows=7)

	#rigid body marker
	#marker1
	rig1_mark1_x = pd.read_csv(filename,usecols=[10],skiprows=7)
	rig1_mark1_y = pd.read_csv(filename,usecols=[11],skiprows=7)
	rig1_mark1_z = pd.read_csv(filename,usecols=[12],skiprows=7)
	#marker2
	rig1_mark2_x = pd.read_csv(filename,usecols=[14],skiprows=7)
	rig1_mark2_y = pd.read_csv(filename,usecols=[15],skiprows=7)
	rig1_mark2_z = pd.read_csv(filename,usecols=[16],skiprows=7)

	#marker3
	rig1_mark3_x = pd.read_csv(filename,usecols=[18],skiprows=7)
	rig1_mark3_y = pd.read_csv(filename,usecols=[19],skiprows=7)
	rig1_mark3_z = pd.read_csv(filename,usecols=[20],skiprows=7)
	#rig1_rot = np.array(rig1_rotx_np,rig1_roty_np)

	#create array of data
	rig1_rot = np.append(rig1_rotx,rig1_roty,1)
	rig1_rot = np.append(rig1_rot,rig1_rotz,1)
	rig1_rot = np.append(rig1_rot,rig1_rotw,1)

	rig1_pos = np.append(rig1_posx,rig1_posy,1)
	rig1_pos = np.append(rig1_pos,rig1_posz,1)

	

        print(self.name)
        if self.name[0:3] == 'ECM':
            self.interface = dvrk.ecm(self.name[0])
        else:
            self.interface = dvrk.psm(self.name[0])

#        rospy.Subscriber('/vrpn_client_node/RigidBody' + str(self.name[1]) + '/pose', gm.PoseStamped, self.handle_rotation)
#        rospy.Subscriber('/vrpn_client_node/RigidBody' + str(self.name[2]) + '/pose', gm.PoseStamped, self.handle_rcm)

    def handle_rotation(self):
#        self.arm_rot = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
	for i in range(len(rig1_rot)):
		self.arm_rot = rig1_rot[i]

    def handle_rcm(self):
#        self.pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
#        self.rot = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
	for j in range(len(rig1_rot)):
		self.pos = rig1_pos[j]
		self.rot = rig1_rot[j]

    def get_marker_data(self):
        self.marker_data_pos = np.append(self.marker_data_pos, [self.pos], axis=0)
        self.marker_data_rot = np.append(self.marker_data_rot, [self.rot], axis=0)
