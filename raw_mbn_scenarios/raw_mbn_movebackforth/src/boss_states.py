#!/usr/bin/python
import roslib; roslib.load_manifest('raw_fetch_and_carry')
import rospy

import smach
import smach_ros
import time

class move_to_area(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded'],
            input_keys=['area_to_approach', 'areas'])

    def execute(self, userdata):
	targetidx = userdata.area_to_approach
        targetcoo = userdata.areas[userdata.area_to_approach]
        rospy.loginfo("approaching area idx %d coo %f %f %f", (targetidx, targetcoo[0], targetcoo[1], targetcoo[2]))
	# find first marker
	# follow marker vector
	# move to target position
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
	time.sleep(userdata.waittime)
        return 'succeeded'
