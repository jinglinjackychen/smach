#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time

t = 2

# define state
class STATE_UNKOWN(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['init'])

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE_UNKOWN')
        time.sleep(t)
        return 'init'


# define state
class STATE_GO_HOME(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_home'])

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE_GO_HOME')
        time.sleep(t)
        return 'go_home'

class STATE_POSE_ESTIMATION(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['have_objects','no_object'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE_POSE_ESTIMATION')
        if self.counter < 2:
            self.counter += 1
            time.sleep(t)
            return 'have_objects'
        else:
            time.sleep(t)
            return 'no_object'

class STATE_PICKING(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pick'])

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE_PICKING')
        time.sleep(t)
        return 'pick'

class STATE_CHECK_GRASP(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE_CHECK_GRASP')
        if self.counter < 2:
            self.counter += 1
            time.sleep(t)
            return 'success'
        else:
            time.sleep(t)
            return 'failure'

class STATE_APRILTAG_LOC(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['apriltag_location'])

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE_APRILTAG_LOC')
        time.sleep(t)
        return 'apriltag_location'

class STATE_PLACING(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['have_objects','no_object'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE_PLACING')
        if self.counter < 2:
            self.counter += 1
            time.sleep(t)
            return 'have_objects'
        else:
            time.sleep(t)
            return 'no_object'

class STATE_PLAN_ERROR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['error'])

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE_PLAN_ERROR')
        time.sleep(t)
        return 'error'



# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['END'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('STATE_UNKOWN', STATE_UNKOWN(), transitions={'init':'STATE_GO_HOME'})
        smach.StateMachine.add('STATE_GO_HOME', STATE_GO_HOME(), transitions={'go_home':'STATE_POSE_ESTIMATION'})
        smach.StateMachine.add('STATE_POSE_ESTIMATION', STATE_POSE_ESTIMATION(), transitions={'have_objects':'STATE_PICKING','no_object':'END'})
        smach.StateMachine.add('STATE_PICKING', STATE_PICKING(), transitions={'pick':'STATE_CHECK_GRASP'})
        smach.StateMachine.add('STATE_CHECK_GRASP', STATE_CHECK_GRASP(), transitions={'success':'STATE_APRILTAG_LOC','failure':'STATE_POSE_ESTIMATION'})
        smach.StateMachine.add('STATE_APRILTAG_LOC', STATE_APRILTAG_LOC(), transitions={'apriltag_location':'STATE_PLACING'})
        smach.StateMachine.add('STATE_PLACING', STATE_PLACING(), transitions={'have_objects':'STATE_POSE_ESTIMATION','no_object':'END'})
    
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
