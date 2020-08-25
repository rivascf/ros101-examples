#! /usr/bin/env python

import rospy
import actionlib
import simple_action_msg.msg 

class FibonacciAction(object):
    _feedback = simple_action_msg.msg.FibonacciFeedback()
    _result = simple_action_msg.msg.FibonacciResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, simple_action_msg.msg.FibonacciAction, self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):    
        r = rospy.Rate(1) # Delay a proposito
        success = True

        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)

        rospy.loginfo('%s Ejecutando, creando la secuencia de fibonacci de orden %i inicializada con %i, %i' %(self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))

        for i in range(1, goal.order):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: es Prioritario' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            self._as.publish_feedback(self._feedback)
            r.sleep()

        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: exitoso'% self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('fibonacci')
    server = FibonacciAction(rospy.get_name())
    rospy.spin()
