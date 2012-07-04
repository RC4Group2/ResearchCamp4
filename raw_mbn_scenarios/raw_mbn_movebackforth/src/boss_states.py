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

        markerchain_name = userdata.areas[userdata.area_to_approach]['markerchain']

	# TODO get pose (abs location) of first marker in marker chain into variable 'premarkerpose'
        finalpose = userdata.areas[userdata.area_to_approach]['finalpose']

        rospy.loginfo("approaching area idx %d premarkerpose %s finalpose %s", targetidx, repr(premarkerpose), repr(finalpose))

	# goto location of first marker (premarkerpose)
	handle_base = sss.move("base", premarkerpose)
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

	# instruct marker based navigation to follow marker chain of specified name
	handle_base = sss.move("markers", markerchain_name)
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

	# move to target position in arena (finalpose)
	handle_base = sss.move("base", finalpose)
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
