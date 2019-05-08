
import pybullet as pb
import rospy
from omni_interface import PhantomOmni
from aml_robot import BulletPisaHand, BulletGripper
from aml_math import *
import numpy as np

rospy.init_node('phantom_omni')
omni_robot = PhantomOmni(scale=1e-2)


# while not rospy.is_shutdown():




hand = BulletPisaHand(phys_opt='gui')
hand._bullet_robot.set_default()

hand.create_constraint()
pb.setRealTimeSimulation(1)


# base0 = Transform3.
hand.set_base_pose([0,0,0],[0,0,0,1], reset = True)
hand.set_joint_state(np.array([0.0]*15), reset = True)

# omni_robot.calibration([0,0,0],[0,0,0,1])

p_init, q_init = omni_robot.get_ee_pose()
pose_init = (p_init, q_init)
# omni_pose0 = pb.invertTransform(p_init, q_init)
# [-0.5,-0.5,-0.5,0.5]


hp, hq = hand.get_base_pose()
hand_pose0 = (hp, q_init)
# hand_pose0 = pb.multiplyTransforms(hand_pose0[0],hand_pose0[1],[0,0,0],)

# euler2quaternion

# h = h0*o0_inv * o


rate = rospy.Rate(60)

while not rospy.is_shutdown():

	
	# p, q = omni_robot.get_ee_pose()
	# omni_pose = omni_pose0.mult(Transform3(p=p, q=q))

	# hand_pose = hand_pose0.mult(omni_pose)

	# hand.set_base_pose(hand_pose.p ,hand_pose.q, reset = False)

	if omni_robot._omni_bt_state['white_bt']:
		p, q = omni_robot.get_ee_pose()

		# p_, q_ = pb.invertTransform([0,0,0], [-0.5,0.5,0.5,0.5])
		_, q = pb.multiplyTransforms([0,0,0], [-0.707,0,0,0.707], [0,0,0], q)

		# hand_pose = pb.multiplyTransforms(hp, hq, p, q)

		# omni_pose = pb.multiplyTransforms(omni_pose0[0], omni_pose0[1], p, q)

		# hand_pose = pb.multiplyTransforms(hand_pose0[0], hand_pose0[1], omni_pose[0], omni_pose[1])
		# hand_pos, hand_ori = pb.multiplyTransforms(hand_pose0[0], [0,0,0,1], p, q)
		# _, hand_ori = pb.multiplyTransforms([0,0,0], hand_ori, [0,0,0], [-0.5,-0.5,-0.5,0.5])
		hand.set_base_pose(p, [q[0],q[1],q[2],q[3]], reset = False)
		# hand.set_joint_state([0.0]*15)

	# elif omni_robot._omni_bt_state['grey_bt']:
	# 	# hp, hq = hand.get_base_pose()
	# 	# hand_pose0 = ([0,0,0], [0,0,0,1])
	# 	# omni_pose0 = pose_init
	# 	# hand.set_base_pose(hand_pose0[0],hand_pose0[1], reset = False)
	# 	hand.set_joint_state([1.0]*15, reset = False)
	# else:
	# 	# pass
	# 	p, q = omni_robot.get_ee_pose()
	# 	omni_pose0 = pb.invertTransform(p, q)


	# 	hp, hq = hand.get_base_pose()
	# 	hand_pose0 = (hp, hq)


	# if omni_robot._omni_bt_state['white_bt']:
	# 	p, q = omni_robot.get_ee_pose()
	# 	omni_pose = omni_pose0.mult(Transform3(p=p, q=q))

	# 	hand_pose = hand_pose0.mult(omni_pose)

	# 	hand.set_base_pose(hand_pose.p ,hand_pose.q, reset = False)

	# elif omni_robot._omni_bt_state['grey_bt']:
	# 	hp, hq = hand.get_base_pose()
	# 	hand_pose0 = Transform3(p=[0,0,0], q=[0,0,0,1])
	# 	omni_pose0 = omni_robot_init_pose
	# else:
	# 	p, q = omni_robot.get_ee_pose()
	# 	omni_pose0 = Transform3(p=p, q=q)
	# 	omni_pose0.inverted()


	# 	hp, hq = hand.get_base_pose()
	# 	hand_pose0 = Transform3(p=hp, q=hq)

		




	rate.sleep()
	


