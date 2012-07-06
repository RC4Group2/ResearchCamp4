#!/usr/bin/python
import roslib; roslib.load_manifest('raw_mbn_movebackforth')
import rospy

import smach
import smach_ros

#from simple_script_server import *
#sss = simple_script_server()

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion, PoseStamped, Point
from sensor_msgs.msg import Joy

import actionlib
from move_base_msgs.msg import *

from marker_server.srv import *

from brics_msgs.srv import markerFollower

def preparePoseQ(x,y,quart):
  goal_pose = PoseStamped()
  # TODO: this should get the frame from the input marker message...
  goal_pose.header.frame_id = "/map"
  goal_pose.header.stamp = rospy.Time.now()
  goal_pt = Point(x,y,0)
  goal_pose.pose.position = goal_pt
  goal_pose.pose.orientation = quart
  return goal_pose

class cb:
	def __init__(self):
		self.notified = False
	def reset(self):
		self.notified = False
	def call(self,message):
		#print "joy cb got notified %s" % (message,)
		if message.buttons[2] == 1:
			self.notified = True
	def notified(self):
		return self.notified


class move_to_area(smach.State):
  def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded','failed'],
			input_keys=['area_to_approach', 'areas'])
		print "init move_to_area"
		self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		self.move_base_client.wait_for_server()

		print "waiting"
		rospy.wait_for_service('/marker_server/marker_data')
		rospy.wait_for_service('/marker_server/list_marker')
		rospy.wait_for_service('/run_ar_follower')
		print "got services"
		self.markerdataproxy = rospy.ServiceProxy('/marker_server/marker_data', MarkerData)
		self.markerchainproxy = rospy.ServiceProxy('/marker_server/list_marker', MarkerList)
		self.arproxy = rospy.ServiceProxy('/run_ar_follower', markerFollower)
		#self.mbn_client = actionlib.SimpleActionClient("markerbasednav TODO", MoveBaseAction) # TODO TODO MoveBaseAction
		#self.mbn_client.wait_for_server()
		print "initialized move_to_area"

		self.cb = cb()
		self.joy = rospy.Subscriber('/joy',Joy,callback=self.cb.call)

  def execute(self, userdata):
		targetidx = userdata.area_to_approach

		markerchain_name = userdata.areas[userdata.area_to_approach]['markerchain']

		# get pose (abs location) of first marker in marker chain into variable 'premarkerpose'
		print "getting marker chain with name %s" % (markerchain_name,)
		markerchain = self.getMarkerChain(markerchain_name)
		#print "got marker chain data '%s'" % (repr(markerchain),)
		firstmarker = self.getMarkerData(markerchain.list.markersIDs[0])
		print "got marker data '%s'" % (firstmarker,)
		premarkerpose = preparePoseQ(firstmarker.pose.position.x, firstmarker.pose.position.y, firstmarker.pose.orientation)
		print "got premarkerpose '%s'" % (premarkerpose,)
		finalpose = userdata.areas[userdata.area_to_approach]['finalpose']
		
		rospy.loginfo("approaching area idx %d premarker %s finalpose %s" % (targetidx,premarkerpose,finalpose))

		# goto location of first marker (premarkerpose)
		cmdresult = self.command_move_base_blocking(premarkerpose)
		if cmdresult == 'failed':
			return 'failed'

		print "waiting for joy/starting markerfollow"
		#self.cb.reset
		#while not self.cb.notified:
		#	rospy.sleep(0.1)
		#print "joy!"
		try:
			result = self.arproxy(markerchain.list.markersIDs)
			print "got result from arproxy %s" % (result,)
		except rospy.ServiceException, e:
			print "Failed to call arproxy %s" % (e,)

		print "goto final pose"
		cmdresult = self.command_move_base_blocking(finalpose)
		if cmdresult == 'failed':
			return 'failed'

		return 'succeeded'

  def command_move_base_blocking(self,goal_pose):
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

  def getMarkerData(self, markerid):
		try:
			markerdata = self.markerdataproxy(markerid)
			print "got markerdata %s" % (markerdata,)
			return markerdata
		except rospy.ServiceException, e:
			print "Failed to get MarkerData for %s because of %s" % (markerid,e)

  def getMarkerChain(self, markerchain):
		try:
			markerchaindata = self.markerchainproxy(markerchain)
			print "got markerchaindata %s" % (markerchaindata,)
			return markerchaindata
		except rospy.ServiceException, e:
			print "Failed to get MarkerChainData for %s because of %s" % (markerchain,e)

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

# vim:ts=2:sw=2:nolist:noexpandtab:
