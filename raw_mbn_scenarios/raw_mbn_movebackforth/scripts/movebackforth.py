#!/usr/bin/python
import roslib; roslib.load_manifest('raw_mbn_movebackforth')
import rospy

import smach
import smach_ros

# generic states
from generic_basic_states import *
#from generic_navigation_states import *
#from generic_state_machines import *

from geometry_msgs.msg import Quaternion, PoseStamped, Point

# scenario specific states
from boss_states import *

def preparePose(x,y,theta):
  goal_pose = PoseStamped()
  # TODO: this should get the frame from the input marker message...
  goal_pose.header.frame_id = "/map"
  goal_pose.header.stamp = rospy.Time.now()
  goal_pt = Point(x,y,theta)
  goal_pose.pose.position = goal_pt
  # TODO: this should be aligned with the april tag pose!
  qq = tf.transformations.quaternion_from_euler(0, 0, 0.0)
  goal_pose.pose.orientation = Quaternion(*qq)
  return goal_pose

# main
def main():
    rospy.init_node('movebackforth')

    #MoveSM = smach.StateMachine(outcomes=['succeeded', 'failed'])
    #with MoveSM:
#	smach.StateMachine.add('LOCATEFIRSTMARKER', locatefirstmarker(),
#		transitions={'succeeded'

    SM = smach.StateMachine(outcomes=['overall_failed'])
    
    # world knowledge
    SM.userdata.waittime = 5.0
    SM.userdata.area_to_approach = 0; 
			# name of marker
		        # x, y, z, roll, pitch, yaw
    SM.userdata.areas = [{'markerchain': 'pathA1_A2',
                          'finalpose': preparePose(2.17, -1.49, 0)},
                         {'markerchain': 'pathA2_A1',
                          'finalpose': preparePose(1.34, 2.26, 0)}]
    
    # open the container
    with SM:
        # add states to the container
        
        #smach.StateMachine.add('INIT_ROBOT', init_robot(),
        #    transitions={'succeeded':'MOVE_TO_AREA', 
        #                 'failed':'overall_failed'})
        
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
