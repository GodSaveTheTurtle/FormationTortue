#!/usr/bin/env python

from __future__ import division

import rospy
from publisher import ThreadedPublisher
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Led

from settings import Settings
import state
import testEsclave


class MainThread(ThreadedPublisher):
    # TODO support for more than one publisher

    def __init__(self, commands, target_sim=False, name='turtleX'):
        # if target_sim:
        #     topic = '/%s/cmd_vel' % name
        # else:
        #     topic = '/cmd_vel_mux/input/teleop'
        super(MainThread, self).__init__('/mobile_base/commands/led1', Led, 1/10.0)
        self.commands = commands
        self.state = None
        self.color = 0

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
        print t

        self.color = (self.color + 1) % 4
        self.publish(self.color)


class InstructionSubscriber(object):
    def __init__(self, name, commands):
        self._running = False
        self._sub = None
        self.commands = commands
        self.name = name

    def update(self, data):
        # data format: {'d': 42, 'theta_rad': -5, 'goal_d': 0, 'goal_theta_rad': 0}
        rospy.loginfo('%s update: %s' % (self.name, data))
        # TODO computations here, publish twists
        orientation = 0  # TODO subscribe /odom, etc
        angle, speed = testEsclave.main(data, orientation)
        self.commands.angular_spd = angle
        self.commands.linear_spd = speed

    def terminate(self):
        rospy.logdebug('Terminating %s', type(self).__name__)
        self._running = False
        self._sub.unregister()
        self._sub = None

    def start(self):
        ''' Returns self for chaining '''
        self._running = True
        # TODO connect TCP, wait for reception
        # self._sub = rospy.Subscriber('/%s/instructions' % self.name, Instruction, self.update)
        return self

    def join(self):
        pass


if __name__ == '__main__':

    commands = Settings()
    commands.next_state = state.Obey

    commands.sim_mode = rospy.get_param('sim_mode', False)

    try:
        rospy.init_node('slave')
        name = rospy.get_param('~name', 'slave')

        if commands.sim_mode:
            from turtlesim.srv import Spawn
            rospy.wait_for_service('spawn')
            spawner = rospy.ServiceProxy('spawn', Spawn)
            spawner(8, 5, 0, name)
            # TODO populate virtual slave data

        rospy.loginfo(commands)
        rospy.loginfo(name)

        # direction = MainThread(commands, commands.sim_mode, name).start()
        sub = InstructionSubscriber(name, commands).start()

        rospy.spin()  # waits until rospy.is_shutdown() is true (Ctrl+C)

        rospy.loginfo('\nTerminating the threads...')
        for thread in [sub]:
            thread.terminate()
            thread.join()

        rospy.loginfo('All ok.')

        # TODO networking

        rospy.spin()  # waits until rospy.is_shutdown() is true (Ctrl+C)

    except rospy.ROSInterruptException:
        pass
