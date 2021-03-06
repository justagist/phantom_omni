

import rospy
from omni_interface import PhantomOmni


rospy.init_node('phantom_omni')
omni_robot = PhantomOmni(scale=1e-2)

while not rospy.is_shutdown():
	print omni_robot.get_ee_pose()
