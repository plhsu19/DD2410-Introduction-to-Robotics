#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")
		# self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
		# rospy.wait_for_service(self.pick_srv_nm)
		# self.cube_PoseStamped.header.stamp = rospy.Time.now()
        # self.cube_PoseStamped.header.frame_id = self.robot_base_frame_nm
		
		# tuck the arm
		b0 = tuckarm()

		# lower head
		b1 = movehead("down")

		# Keep
		b2 = pt.composites.Selector(
			name="Keep",
			children=[counter(20, "Head Full down?"), go("Keeping!", 0, 0)]
		)
		# pick
		b3 = Pick()
		
		# rotate
		b4 = pt.composites.Selector(
			name="Rotate",
			children=[counter(60, "Finished?"), go("Go rotating!", 0, -0.5)]
		)

		# forward
		b5 = pt.composites.Selector(
			name="Forward",
			children=[counter(17, "Finished?"), go("Go straight ahead!", 0.5, 0)]
		)
		# place
		b6 = Place()

		# check
		b7 = Check()

		# tuck Arm
		b8 = tuckarm()
		# rotate
		b9 = pt.composites.Selector(
			name="RotateBack",
			children=[counter(58, "Finished?"), go("Go rotating!", 0, 0.5)]
		)

		# back to Table1
		b10 = pt.composites.Selector(
			name="Backward",
			children=[counter(19, "Finished?"), go("Backward!", 0.5, 0)]
		)

		# pick
		# b11 = RSequence(name="GoingBack", children=[b8, b9, b10])

		# b12 = pt.composites.Selector(
		# 	name="Detect",
		# 	children=[b7, b11]
		# )
		# up head (Unnessary)
		# bn = movehead("up")

		# become the tree
		tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
