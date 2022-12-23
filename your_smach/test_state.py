#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2'],
                             input_keys=['foo_counter_in1','foo_counter_in2'],
                             output_keys=['foo_counter_out1','foo_counter_out2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if userdata.foo_counter_in1 < 3:
            userdata.foo_counter_out1 = userdata.foo_counter_in1 + 1
            userdata.foo_counter_out2 = userdata.foo_counter_in2 + 2
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['bar_counter_in1','bar_counter_in2'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        rospy.loginfo('Counter1 = %f'%userdata.bar_counter_in1)
        rospy.loginfo('Counter2 = %f'%userdata.bar_counter_in1)         
        return 'outcome1'
        




def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.sm_counter1 = 0
    sm.userdata.sm_counter2 = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 
                                            'outcome2':'outcome4'},
                               remapping={'foo_counter_in1':'sm_counter1','foo_counter_in2':'sm_counter2',  
                                          'foo_counter_out1':'sm_counter1','foo_counter_out2':'sm_counter2'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'},
                               remapping={'bar_counter_in1':'sm_counter1','bar_counter_in2':'sm_counter2'})


    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()