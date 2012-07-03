#!/usr/bin/python
import roslib; roslib.load_manifest('raw_fetch_and_carry')
import rospy

import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()

class move_to_area(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded'],
            input_keys=['area_to_approach', 'areas'])

    def execute(self, userdata):
	targetidx = userdata.area_to_approach
        premarkerspec = userdata.areas[userdata.area_to_approach]['premarker']
        finalposspec = userdata.areas[userdata.area_to_approach]['finalpos']
        rospy.loginfo("approaching area idx %d premarker spec %s finalpos spec %s", targetidx, repr(premarkerspec), repr(finalposspec))
	# goto expected location of first marker
	handle_base = sss.move("base", premarkerspec)
	while True:                
	    rospy.sleep(0.1)
	    base_state = handle_base.get_state()
	    if (base_state == actionlib.simple_action_client.GoalStatus.SUCCEEDED):
		break
	    elif (base_state == actionlib.simple_action_client.GoalStatus.ACTIVE):
		continue
	    else:
		print 'last state: ',base_state
		return "failed"
	# instruct marker based navigation to follow marker vector
	handle_base = sss.move("markers", targetidx)
	while True:                
	    rospy.sleep(0.1)
	    base_state = handle_base.get_state()
	    if (base_state == actionlib.simple_action_client.GoalStatus.SUCCEEDED):
		break
	    elif (base_state == actionlib.simple_action_client.GoalStatus.ACTIVE):
		continue
	    else:
		print 'last state: ',base_state
		return "failed"
	# move to final target position
	handle_base = sss.move("base", finalposspec)
	while True:                
	    rospy.sleep(0.1)
	    base_state = handle_base.get_state()
	    if (base_state == actionlib.simple_action_client.GoalStatus.SUCCEEDED):
		break
	    elif (base_state == actionlib.simple_action_client.GoalStatus.ACTIVE):
		continue
	    else:
		print 'last state: ',base_state
		return "failed"
        return 'succeeded'

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
