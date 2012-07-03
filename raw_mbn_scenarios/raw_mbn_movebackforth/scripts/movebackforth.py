#!/usr/bin/python
import roslib; roslib.load_manifest('raw_mbn_movebackforth')
import rospy

import smach
import smach_ros

# generic states
from generic_basic_states import *
#from generic_navigation_states import *
#from generic_state_machines import *

# scenario specific states
from boss_states import *

# main
def main():
    rospy.init_node('movebackforth')

    #MoveSM = smach.StateMachine(outcomes=['succeeded', 'failed'])
    #with MoveSM:
#	smach.StateMachine.add('LOCATEFIRSTMARKER', locatefirstmarker(),
#		transitions={'succeeded'

    SM = smach.StateMachine(outcomes=['overall_failed'])
    
    # world knowledge
    SM.userdata.waittime = 2
    SM.userdata.area_to_approach = 0; 
			# x, y, z
    SM.userdata.areas = [[0.033 + 0.024 - 0.32, 0, 0.14],
			 [0.033 + 0.024 - 0.235, 0, 0.14]];
                                            # x, y, z, roll, pitch, yaw
    #SM.userdata.rear_platform_free_poses = [[0.033 + 0.024 - 0.32,  0.0, 0.14, 0, -math.pi + 0.2, 0, "/arm_link_0"],   #front pos
    #                                        [0.033 + 0.024 - 0.28,  0.0, 0.14, 0, -math.pi + 0.3, 0, "/arm_link_0"],    #rear pos
    #                                        [0.033 + 0.024 - 0.235, 0.0, 0.14, 0, -math.pi + 0.3, 0, "/arm_link_0"]]
    
    # open the container
    with SM:
        # add states to the container
        
        smach.StateMachine.add('INIT_ROBOT', init_robot(),
            transitions={'succeeded':'MOVE_TO_AREA', 
                         'failed':'overall_failed'})
        
        #smach.StateMachine.add('MOVE_TO_AREA', MoveSM, 
        smach.StateMachine.add('MOVE_TO_AREA', move_to_area(),
            transitions={'succeeded':'FIND_NEW_GOAL',
                         'failed':'overall_failed'})
        
        smach.StateMachine.add('FIND_NEW_GOAL', find_new_goal(),
            transitions={'succeeded':'WAIT',
                         'failed':'overall_failed'})

        smach.StateMachine.add('WAIT', wait(),
            transitions={'succeeded':'MOVE_TO_AREA'})
            
    # Start SMACH viewer
    smach_viewer = smach_ros.IntrospectionServer('MOVEBACKANDFORTH', SM, 'MOVEBACKANDFORTH')
    smach_viewer.start()

    SM.execute()

    # stop SMACH viewer
    rospy.spin()
    # smach_thread.stop()
    smach_viewer.stop()

if __name__ == '__main__':
    main()
