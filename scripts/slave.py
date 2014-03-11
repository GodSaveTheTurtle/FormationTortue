#!/usr/bin/env python

from __future__ import division

import rospy
from publisher import ThreadedPublisher
from geometry_msgs.msg import Twist

from settings import Settings

import state


class MainThread(ThreadedPublisher):
    # TODO support for more than one publisher

    def __init__(self, commands, target_sim=False, name='turtleX'):
        if target_sim:
            topic = '/%s/cmd_vel' % name
        else:
            topic = '/cmd_vel_mux/input/ teleop'
        super(MainThread, self).__init__(topic, Twist, 1/10.0)
        self.commands = commands
        self.state = None

    def update(self):
        ''' Replace this thread's loop with the listener's activation '''
        if self.commands.next_state:
            if self.state:
                rospy.loginfo('Quitting state %s', type(self.state).__name__)
                self.state.end(self.commands)
            rospy.loginfo('Entering state %s', self.commands.next_state.__name__)
            self.state = self.commands.next_state(self.commands)

        self.state.update(self.commands)

        t = Twist()
        t.linear.x = self.commands.linear_spd
        t.angular.z = self.commands.angular_spd
        self.publish(t)


if __name__ == '__main__':

    name = 'slave'
    simu_mode = rospy.get_param('simu_mode', False)

    if simu_mode:
        from turtlesim.srv import Spawn

        # using rospy.get_name() as parameter for rospy.init_node() is not allowed
        name = rospy.get_param(rospy.get_name() + '/name', name)

        rospy.wait_for_service('spawn')
        spawner = rospy.ServiceProxy('spawn', Spawn)
        spawner(8, 5, 0, name)

        # TODO populate virtual slave data

    try:
        rospy.init_node(name)

        # TODO states

        commands = Settings()
        commands.next_state = state.Obey

        direction = MainThread(commands, simu_mode, name).start()

        rospy.spin()  # waits until rospy.is_shutdown() is true (Ctrl+C)

        rospy.loginfo('\nTerminating the publishers...')
        for thread in (direction):
            thread.terminate()
            thread.join()

        rospy.loginfo('All ok.')

        # TODO networking

        rospy.spin()  # waits until rospy.is_shutdown() is true (Ctrl+C)

    except rospy.ROSInterruptException:
        pass
