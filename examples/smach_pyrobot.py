#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from pyrobot import Robot

bot = Robot("locobot")

# define state GoHome
class GoHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state GoHome')
        bot.arm.go_home()
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'

# define state Move
class Move(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        target_joints = [[0.408, 0.721, -0.471, -1.4, 0.920], [-0.675, 0, 0.23, 1, -0.70]]
        rospy.loginfo('Moving')
        for joint in target_joints:
                bot.arm.set_joint_positions(joint, plan=False)
                time.sleep(1)
        return 'outcome2'

# main
def main():
    #rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('GoHome', GoHome(), transitions={'outcome1':'Move', 'outcome2':'outcome4'})
        smach.StateMachine.add('Move', Move(), transitions={'outcome2':'GoHome'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
