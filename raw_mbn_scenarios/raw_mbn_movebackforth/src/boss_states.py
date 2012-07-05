#!/usr/bin/python
import roslib; roslib.load_manifest('raw_mbn_movebackforth')
import rospy

import smach
import smach_ros

#from simple_script_server import *
#sss = simple_script_server()

import actionlib
from move_base_msgs.msg import *

class move_to_area(smach.State):
    def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded'],
			input_keys=['area_to_approach', 'areas'])
		self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		self.move_base_client.wait_for_server()
		self.mbn_client = actionlib.SimpleActionClient("markerbasednav TODO", MoveBaseAction) # TODO TODO MoveBaseAction
		self.mbn_client.wait_for_server()

    def execute(self, userdata):
		targetidx = userdata.area_to_approach

		markerchain_name = userdata.areas[userdata.area_to_approach]['markerchain']

		# TODO get pose (abs location) of first marker in marker chain into variable 'premarkerpose'
		finalpose = userdata.areas[userdata.area_to_approach]['finalpose']

		rospy.loginfo("approaching area idx %d premarkerpose %s finalpose %s", targetidx, repr(premarkerpose), repr(finalpose))

		# goto location of first marker (premarkerpose)
		cmdresult = command_move_base_blocking(premarkerpose)
		if cmdresult == 'failed':
			return 'failed'

		cmdresult = command_mbn_blocking(markerchain_name)
		if cmdresult == 'failed':
			return 'failed'

		cmdresult = command_move_base_blocking(finalpose)
		if cmdresult == 'failed':
			return 'failed'

		return 'succeeded'

    def command_move_base_blocking(goal_pose):
		goal_msg = MoveBaseGoal()
		goal_msg.target_pose = goal_pose
		self.move_base_client.send_goal(goal_msg)
		goal_status = self.move_base_client.get_state()
		self.move_base_client.wait_for_result()
		print "got controller result!"

		controller_state = self.move_base_client.get_state()
		if controller_state == GoalStatus.PREEMPTED:
			 return 'failed'
		elif controller_state == GoalStatus.ABORTED:
			 print "move base controller call aborted!"
			 return 'failed'
		elif controller_state == GoalStatus.SUCCEEDED:
			 print "move base controller call succeeded!"
			 return 'succeeded'
		else:
			 print "move base controller returned %r" % (controller_state)
			 return 'failed'

    def command_mbn_blocking(markerchainname):
		goal_msg = MoveBaseGoal() #TODO
		goal_msg.target_pose = goal_pose #TODO
		self.mbn_client.send_goal(goal_msg)
		goal_status = self.mbn_client.get_state()
		self.mbn_client.wait_for_result()
		print "got mbn controller result!"

		controller_state = self.mbn_client.get_state()
		if controller_state == GoalStatus.PREEMPTED:
			 return 'failed'
		elif controller_state == GoalStatus.ABORTED:
			 print "mbn controller call aborted!"
			 return 'failed'
		elif controller_state == GoalStatus.SUCCEEDED:
			 print "mbn controller call succeeded!"
			 return 'succeeded'
		else:
			 print "mbn controller returned %r" % (controller_state)
			 return 'failed'

    def getMarkerData(self, markerid):
		rospy.wait_for_service('marker_data')

		try:
			markerdataproxy = rospy.ServiceProxy('marker_data', MarkerData)
			markerdata = markerdataproxy(markerid)
			return markerdata
		except rospy.ServiceException, e:
			print "Failed to get MarkerData for %s because of %s" % (markerid,e)

class find_new_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self,
	    outcomes=['succeeded','failed'],
            input_keys=['area_to_approach'],
            output_keys=['area_to_approach'])

    def execute(self, userdata):
	# just switch between area with idx 0 and idx 1
        userdata.area_to_approach = 1 - userdata.area_to_approach
        rospy.loginfo("next area to approach has index %d", userdata.area_to_approach)
        return 'succeeded'

class wait(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded'],
            input_keys=['waittime'])

    def execute(self, userdata):
	rospy.sleep(userdata.waittime)
        return 'succeeded'

# vim:ts=3:sw=3:
