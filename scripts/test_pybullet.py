
import pybullet as pb
import rospy
from omni_interface import PhantomOmni
from aml_robot import BulletPisaHand, BulletGripper, SceneObject
from aml_math import *
import numpy as np
from kbhit import KBHit

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

hand_pos0, hand_ori0 = pb.multiplyTransforms([0,0,0],[0,0,0,1],[0,0,0],[0.707,0.,0.,0.707])

hand_pose = pb.multiplyTransforms(hand_pos0,hand_ori0,[0,0,0],q_init)
hand_pose = pb.multiplyTransforms(hand_pose[0], hand_pose[1], [0,0,0], [1,0,0,0])

omni_pos0 = p_init

hand.set_base_pose(hand_pose[0],hand_pose[1], reset = True)
hand.set_joint_state(np.array([0.0]*15), reset = True)

rate = rospy.Rate(60)

hand_joint = 0.0
hand_delta = 0.01

## keyboard
keyboard = KBHit()
enabled = False

while not rospy.is_shutdown():

	if keyboard.kbhit():
		c = keyboard.getch()
		enabled = c == 'e'
		if enabled:
			scene_obj.set_pos_ori(object_position,object_orientation)

	if enabled:
		p, q = omni_robot.get_ee_pose()


		hand_pose = pb.multiplyTransforms([0,0,0], hand_ori0, [0,0,0], q)
		hand_pose = pb.multiplyTransforms(hand_pose[0], hand_pose[1], [0,0,0], [1,0,0,0])

		hand.set_base_pose(p - omni_pos0 + hand_pos0, hand_pose[1], reset = False)
		
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
	


