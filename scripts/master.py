#!/usr/bin/env python

from __future__ import division
import socket

import rospy

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import state
from publisher import ThreadedPublisher
from settings import Settings
from formation.msg import Instruction


class MainThread(ThreadedPublisher):
    # TODO support for more than one publisher

    def __init__(self, commands, target_sim=False):
        if target_sim:
            topic = '/turtle1/cmd_vel'
        else:
            topic = '/cmd_vel_mux/input/teleop'
        super(MainThread, self).__init__(topic, Twist, 1/10.0)
        self.commands = commands
        for slave_name in self.commands.slaves:
            self.commands.slaves[slave_name]['pub'] = rospy.Publisher('/%s/instructions' % slave_name, Instruction)
        self.state = None

    def update(self):
        ''' Update the state '''
        if self.commands.next_state:
            if self.state:
                rospy.logdebug('Quitting state %s', type(self.state).__name__)
                self.state.end(self.commands)
            rospy.logdebug('Entering state %s', self.commands.next_state.__name__)
            self.state = self.commands.next_state(self.commands)

        self.state.update(self.commands)

        t = Twist()
        t.linear.x = self.commands.linear_spd
        t.angular.z = self.commands.angular_spd
        self.publish(t)
        for slave_name in self.commands.slaves:
            self.commands.slaves[slave_name]['pub'].publish(Instruction(0, 0))


class NetworkThread(ThreadedPublisher):

    def __init__(self, commands):
        ''' That's not really a publisher but w/e, the threading is done easily that way '''
        super(NetworkThread, self).__init__(None, None)
        self.commands = commands
        self.s = None
        self.hostname = '0.0.0.0'
        self.port = 1337

    def start(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((self.hostname, self.port))
        return super(NetworkThread, self).start()

    def loop(self):
        ''' Replace this thread's loop with a blocking wait on the socket '''
        while self._running and not rospy.is_shutdown():
            data = self.s.recv(1024).split(" ")
            if data[0] == 'd':  # Direction command
                lin, ang = [int(i) for i in data[1:]]
                self.commands.in_linear_spd = lin
                self.commands.in_angular_spd = ang
            elif data[0] == 's':
                self.commands.visible_slaves += int(data[1])
            elif data[0] == 'o':
                self.commands.has_obstacle = not self.commands.has_obstacle

    def terminate(self):
        super(NetworkThread, self).terminate()
        # Need to send something to stop the waiting loop
        tmp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tmp_socket.sendto('0 0', (self.hostname, self.port))
        tmp_socket.close()
        self.s.close()


class SimSlaveTracker(object):
    def __init__(self, commands):
        self._running = False
        self._sub = None
        self.commands = commands

    def update(self, *data):
        rospy.logdebug(data)

    def terminate(self):
        rospy.logdebug('Terminating %s', type(self).__name__)
        self._running = False
        self._sub.unregister()
        self._sub = None

    def start(self):
        ''' Returns self for chaining '''
        self._running = True
        # TODO retrieve from kinect instead
        self._sub = rospy.Subscriber('/slave/pose', Pose, self.update)
        return self

    def join(self):
        pass


if __name__ == '__main__':

    commands = Settings()
    commands.next_state = state.RemoteControlled
    commands.sim_mode = rospy.get_param('sim_mode', False)

    for slave_name in rospy.get_param('master/slaves', []):
        commands.slaves[slave_name] = {
            'pub': None,
            'angle': 0,
            'distance': 0
        }

    try:
        rospy.init_node('turtle_alpha')
        # TODO publisher for slave pose, subscribers to slave pose

        threads = []

        if commands.sim_mode:
            threads.append(SimSlaveTracker(commands).start())

        threads.append(MainThread(commands, commands.sim_mode).start())
        threads.append(NetworkThread(commands).start())

        rospy.spin()  # waits until rospy.is_shutdown() is true (Ctrl+C)

        rospy.logdebug('\nTerminating the publishers...')
        threads.reverse()
        for thread in threads:
            thread.terminate()
            thread.join()

        rospy.logdebug('All ok.')

    except rospy.ROSInterruptException:
        pass
