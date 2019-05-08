import copy
import pybullet as pb
import rospy
from omni_interface import PhantomOmni
from aml_robot import BulletPisaHand, BulletGripper, SceneObject
from aml_math import *
import numpy as np
from aml_io.tools import save_data, load_data
from kbhit import KBHit

savefile = 'test.pkl'

rospy.init_node('phantom_omni')
omni_robot = PhantomOmni(scale=1e-2)


hand = BulletPisaHand(phys_opt='gui')
hand._bullet_robot.set_default()

plane = SceneObject(phys_opt='none')
plane.set_pos_ori([0.45, 0.1, -0.33], [0.0, 0.0, 0.0, 1.0])

scene_obj = SceneObject(filename="cube_small.urdf", phys_id = hand._phys_id, phys_opt='none', fixed_base=False, scale=1.0)
object_position = [0.5,0.6,-0.29]
object_orientation = [0,0,0,1]#self.object.set_pos_ori(,[0,0,0,1])
scene_obj.set_pos_ori(object_position,object_orientation)

hand.create_constraint()
pb.setRealTimeSimulation(1)

p_init, q_init = omni_robot.get_ee_pose()
pose_init = (p_init, q_init)

start_pos = [0,0,0.5]
start_ori = q_init

hand_pos0, hand_ori0 = pb.multiplyTransforms(start_pos,[0,0,0,1],[0,0,0],[0.707,0.,0.,0.707])

hand_pose = pb.multiplyTransforms(hand_pos0,hand_ori0,[0,0,0],q_init)
hand_pose = pb.multiplyTransforms(hand_pose[0], hand_pose[1], [0,0,0], [1,0,0,0])

omni_pos0 = p_init

hand.set_base_pose(hand_pose[0],hand_pose[1], reset = True)
hand.set_joint_state(np.array([0.0]*15), reset = True)

rate = rospy.Rate(60)

hand_joint = 0.0
hand_delta = 0.01

## keyboard

def main():
	# keyboard = KBHit()
	enabled = False


	data_list = []

	hand_pose_list = {}
	joint_traj_list = {}

	traj_type = -1
	count = 0


	while not rospy.is_shutdown():

		# if keyboard.kbhit():
		enable_KEY = ord('e') 
		reset_KEY = ord('r')
		save_KEY = ord('s')
		quit_KEY = ord('q')
		keys = pb.getKeyboardEvents()
		if len(keys) > 0: 
			if enable_KEY in keys:
				enabled = True
			elif ord('1') in keys:
				traj_type = 1
				print "Part 1 of trajectory"
				# count = 0
			elif ord('2') in keys:
				traj_type = 2
				# count = 0
				print "Part 2 of trajectory"
			elif ord('3') in keys:
				traj_type = 3
				print "Part 3 of trajectory"
				# count = 0
			else:
				enabled = False
				if reset_KEY in keys:
					hand.set_base_pose(start_pos,start_ori, reset = True)
					hand.set_joint_state(np.array([0.0]*15), reset = True)
					traj_type = -1

					hand_pose_list = {}
					joint_traj_list = {}

				elif save_KEY in keys:
					traj_type = -1

					if len(hand_pose_list) > 0:
						print "Saving Trajectory Data..."
						data = {'hand_poses_trajs': hand_pose_list, 
								'joint_angles_trajs': joint_traj_list}

						data_list.append(copy.deepcopy(data))

						print "Done!"

					hand_pose_list = {}
					joint_traj_list = {}
				elif quit_KEY in keys:
					break


			if enabled:
				scene_obj.set_pos_ori(object_position,object_orientation)


		if enabled:
			p, q = omni_robot.get_ee_pose()


			hand_pose = pb.multiplyTransforms([0,0,0], hand_ori0, [0,0,0], q)
			hand_pose = pb.multiplyTransforms(hand_pose[0], hand_pose[1], [0,0,0], [1,0,0,0])

			hand.set_base_pose(p - omni_pos0 + hand_pos0, hand_pose[1], reset = False)

			if traj_type > 0:
				# count += 1
				if traj_type in hand_pose_list:
					hand_pose_list[traj_type].append(hand.get_base_pose())
					joint_traj_list[traj_type].append(hand.angles())
				else:
					hand_pose_list[traj_type] = [hand.get_base_pose()]
					joint_traj_list[traj_type]= [hand.angles()]
			# else:
			# 	count = 0

			# print count

		else:
			hand_pos0, _ = hand.get_base_pose()
			omni_pos0, _ = omni_robot.get_ee_pose()



		if omni_robot._omni_bt_state['white_bt']:
			hand_joint -= hand_delta
			hand_joint = max(hand_joint, 0.0)
			hand.set_joint_state([hand_joint], reset = False)
		elif omni_robot._omni_bt_state['grey_bt']:

			hand_joint += hand_delta
			hand_joint = min(hand_joint, 1.0)
			hand.set_joint_state([hand_joint], reset = False)



		rate.sleep()
		


	if len(data_list) > 0:
		print "Writing to file: '%s'"%savefile
		save_data(data_list, savefile)


def replay(filename = savefile):

	data = load_data(filename)
	for i in range(len(data)): # ----- number of trials, each trial is a dict with keys 'hand_poses_trajs' and 'joint_angles_trajs'
		for j in data[i]['hand_poses_trajs']: # ----- each 'hand_poses_trajs' or 'joint_angles_trajs' is a dict with integer key values (starts from 1, max 3), denoting the parts of a demonstration (1: pre-grasp, 2: grasp, 3: post-grasp)
			for k in data[i]['hand_poses_trajs'][j]: # ----- list of hand poses / joint angles in that part of the demonstration
				print k
				hand.set_base_pose(k[0], k[1], reset = False)
				rate.sleep()


if __name__ == '__main__':
	# main()
	replay()
