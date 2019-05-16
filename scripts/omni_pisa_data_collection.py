'''
	Data collection script for collecting hand motion sequence using Phantom Omni for a Pisa Hand in PyBullet.

				Key Bindings 	
	e 				: 	Enable hand tracking
	1				: 	Start recording Sequence 1 of Trajectory
	2				: 	Start recording Sequence 2 of Trajectory
	3				: 	Start recording Sequence 3 of Trajectory
	r 				: 	Clear all Sequences of currenct Trajectory and reset hand to starting pose
	s 				: 	Save all recorded Sequences as a Trajectory
	q 				: 	Quit and write all recorded Trajectories
	<any other key>	: 	Disable hand tracking

'''
import time
import copy
import pybullet as pb
import rospy
from omni_interface import PhantomOmni
from omni_msgs.msg import PisaHandExperimentData as ExpDataMsg
from aml_robot import BulletPisaHand, BulletGripper, SceneObject
from aml_math import *
import numpy as np
from aml_io.tools import save_data, load_data
from kbhit import KBHit

savefile = 'test.pkl'

hand = BulletPisaHand(phys_opt='gui')
hand._bullet_robot.set_default()

scene_obj = SceneObject(filename="cube_small.urdf", phys_id = hand._phys_id, phys_opt='none', fixed_base=False, scale=1.0)
object_position = [0.5,0.6,-0.30]
object_orientation = [-0.49081794,0.50897411,0.54080912,0.45559697] #[0,0,0,1]

def Instructions():

	return "\n\
	Key Bindings \n\
	\n\
	e 				: 	Enable hand tracking\n\n\
	1				: 	Start recording Sequence 1 of Trajectory\n\
	2				: 	Start recording Sequence 2 of Trajectory\n\
	3				: 	Start recording Sequence 3 of Trajectory\n\n\
	r 				: 	Clear all Sequences of currenct Trajectory and reset hand (and object) to starting pose\n\
	s 				: 	Save all recorded Sequences as a Trajectory\n\
	q 				: 	Quit and write all recorded Trajectories\n\n\
	<any other key>	: 	Disable hand tracking\n\n" 	
	


rospy.init_node('phantom_omni')
omni_robot = PhantomOmni(scale=1e-2)


plane = SceneObject(phys_opt='none')
plane.set_pos_ori([0.45, 0.1, -0.33], [0.0, 0.0, 0.0, 1.0])

scene_obj.set_pos_ori(object_position,object_orientation)

hand.create_constraint()
pb.setRealTimeSimulation(1)

p_init, q_init = omni_robot.get_ee_pose()
pose_init = (p_init, q_init)

start_pos = [0.08456879, 0.58092121, 0.03845082]
start_ori = q_init

hand_pos0, hand_ori0 = pb.multiplyTransforms(start_pos,[0,0,0,1],[0,0,0],[0.707,0.,0.,0.707])

hand_pose = pb.multiplyTransforms(hand_pos0,hand_ori0,[0,0,0],q_init)
hand_pose = pb.multiplyTransforms(hand_pose[0], hand_pose[1], [0,0,0], [1,0,0,0])

omni_pos0 = p_init

hand.set_base_pose(hand_pose[0],hand_pose[1], reset = True)
hand.set_joint_state(np.array([0.0]*15), reset = True)

rate = rospy.Rate(60)
rostopic = "/pisa_exp/exp_data"

def create_exp_data_msg(traj_id, pose, joint_angles):

	msg = ExpDataMsg()
	msg.header.stamp = rospy.Time.now()

	msg.traj_id = traj_id

	msg.hand_pose.position.x = pose[0][0]
	msg.hand_pose.position.y = pose[0][1]
	msg.hand_pose.position.z = pose[0][2]
	msg.hand_pose.orientation.x = pose[1][0]
	msg.hand_pose.orientation.y = pose[1][1]
	msg.hand_pose.orientation.z = pose[1][2]
	msg.hand_pose.orientation.w = pose[1][3]

	msg.finger_link_angles = joint_angles

	return msg


## keyboard

def collect_demo_to_bag():
	# keyboard = KBHit()

	pub = rospy.Publisher(rostopic, ExpDataMsg, queue_size = 1)
	enabled = False
	hand_joint = 0.0
	hand_delta = 0.01

	data_list = []

	list1 = []
	list2 = []
	hand_pose_list = {}
	joint_traj_list = {}

	traj_type = -1
	count = 0

	force = np.zeros(3)
	print Instructions()

	while not rospy.is_shutdown():

		# pb.stepSimulation()
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
				print "Sequence 1 of trajectory"

			elif ord('2') in keys:
				traj_type = 2

				print "Sequence 2 of trajectory"
			elif ord('3') in keys:
				traj_type = 3
				print "Sequence 3 of trajectory"

			else:
				enabled = False
				if reset_KEY in keys:
					hand.set_base_pose(start_pos,start_ori, reset = True)
					hand.set_joint_state(np.array([0.0]*15), reset = True)
					traj_type = -1

					hand_pose_list = {}
					joint_traj_list = {}

					scene_obj.set_pos_ori(object_position,object_orientation)

				elif quit_KEY in keys:
					break
				


		if enabled:
			p, q = omni_robot.get_ee_pose()


			hand_pose = pb.multiplyTransforms([0,0,0], hand_ori0, [0,0,0], q)
			hand_pose = pb.multiplyTransforms(hand_pose[0], hand_pose[1], [0,0,0], [1,0,0,0])

			hand.set_base_pose(p - omni_pos0 + hand_pos0, hand_pose[1], reset = False)

			_, _, total_force = hand.get_contacts()

			total_force = total_force*1e-3
			force = force*0.9 + total_force*0.1
			# print "force norm: ", np.linalg.norm(total_force)
			# print "Force: ", total_force
			# print hand.get_base_pose()
			if hand_joint <= 0.3:
				
				omni_robot.omni_force_feedback(force, gain = 1.0)
			else:
				omni_robot.omni_force_feedback(np.zeros(3), gain = 1.0)

			if traj_type > 0:

				pub.publish(create_exp_data_msg(traj_type, hand.get_base_pose(), hand.angles()))


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
		

def collect_data_to_file():
	# keyboard = KBHit()
	enabled = False
	hand_joint = 0.0
	hand_delta = 0.01

	data_list = []

	hand_pose_list = {}
	joint_traj_list = {}

	traj_type = -1
	count = 0

	force = np.zeros(3)
	print Instructions()

	while not rospy.is_shutdown():

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
				print "Sequence 1 of trajectory"
				
			elif ord('2') in keys:
				traj_type = 2
				
				print "Sequence 2 of trajectory"
			elif ord('3') in keys:
				traj_type = 3
				print "Sequence 3 of trajectory"
				
			else:
				enabled = False
				if reset_KEY in keys:
					hand.set_base_pose(start_pos,start_ori, reset = True)
					hand.set_joint_state(np.array([0.0]*15), reset = True)
					traj_type = -1

					hand_pose_list = {}
					joint_traj_list = {}

					scene_obj.set_pos_ori(object_position,object_orientation)

				elif save_KEY in keys:
					traj_type = -1

					if len(hand_pose_list) > 0:
						print "Saving Trajectory Data..."
						data = {'hand_poses_trajs': hand_pose_list, 
								'joint_angles_trajs': joint_traj_list}

						data_list.append(copy.deepcopy(data))

						print "Done!"

						count += 1

						print "\nNumber of Saved Trajectories: %d\n"%count
						time.sleep(1)

					else:
						print "No Sequences recorded to save!"

					hand_pose_list = {}
					joint_traj_list = {}
				elif quit_KEY in keys:
					break
				


		if enabled:
			p, q = omni_robot.get_ee_pose()


			hand_pose = pb.multiplyTransforms([0,0,0], hand_ori0, [0,0,0], q)
			hand_pose = pb.multiplyTransforms(hand_pose[0], hand_pose[1], [0,0,0], [1,0,0,0])

			hand.set_base_pose(p - omni_pos0 + hand_pos0, hand_pose[1], reset = False)

			# _, _, total_force = hand.get_contacts()

			# total_force = total_force*1e-3
			# force = force*0.9 + total_force*0.1
			# print "force norm: ", np.linalg.norm(total_force)
			# print "Force: ", total_force
			# print hand.get_base_pose()
			# if hand_joint <= 0.3:
				
			# 	omni_robot.omni_force_feedback(force, gain = 1.0)
			# else:
			# 	omni_robot.omni_force_feedback(np.zeros(3), gain = 1.0)

			if traj_type > 0:
				# count += 1
				if traj_type in hand_pose_list:
					hand_pose_list[traj_type].append(hand.get_base_pose())
					joint_traj_list[traj_type].append(hand.angles())
				else:
					hand_pose_list[traj_type] = [hand.get_base_pose()]
					joint_traj_list[traj_type]= [hand.angles()]

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
		print "\nWriting %d Trajectories to file: '%s'\n"%(count,savefile)
		save_data(data_list, savefile)
	else:
		print "\nNo trajectory recorded\n"


def replay_from_file(filename = savefile):

	data = load_data(filename)
	for i in range(len(data)): # ----- number of trials, each trial is a dict with keys 'hand_poses_trajs' and 'joint_angles_trajs'
		for j in data[i]['hand_poses_trajs']: # ----- each 'hand_poses_trajs' or 'joint_angles_trajs' is a dict with integer key values (starts from 1, max 3), denoting the parts of a demonstration (1: pre-grasp, 2: grasp, 3: post-grasp)
			for k, a in zip(data[i]['hand_poses_trajs'][j], data[i]['joint_angles_trajs'][j]): # ----- list of hand poses / joint angles in that part of the demonstration
				# print k
				hand.set_base_pose(k[0], k[1], reset = False)

				hand.set_joint_state(a, reset = False)
				rate.sleep()


def move_hand_from_data_msg(msg):

	hand.set_base_pose((msg.hand_pose.position.x,msg.hand_pose.position.y,msg.hand_pose.position.z), 
						(msg.hand_pose.orientation.x,msg.hand_pose.orientation.y,msg.hand_pose.orientation.z,msg.hand_pose.orientation.w))

def replay_from_rosbag():

	rospy.Subscriber(rostopic, ExpDataMsg, move_hand_from_data_msg)
	rospy.spin()



if __name__ == '__main__':
	# collect_demo_to_bag()
	replay_from_rosbag()

