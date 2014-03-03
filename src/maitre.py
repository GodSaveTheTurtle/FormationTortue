#!/usr/bin/env python

import rospy
import socket
from publisher import ThreadedPublisher
from geometry_msgs.msg import Twist

import state


class MainThread(ThreadedPublisher):
    # TODO support for more than one publisher

    def __init__(self, commands, target_sim=False):
        if target_sim:
            topic = '/turtle1/cmd_vel'
        else:
            topic = '/cmd_vel_mux/input/teleop'
        super(MainThread, self).__init__(topic, Twist, 1/10.0)
        self.commands = commands
        self.state = None

    def update(self):
        ''' Replace this thread's loop with the listener's activation '''
        if self.commands['next_state']:
            if self.state:
                print 'Quitting state', type(self.state).__name__
                self.state.end(self.commands)
            print 'Entering state', self.commands['next_state'].__name__
            self.state = self.commands['next_state'](self.commands)

        self.state.update(self.commands)

        t = Twist()
        t.linear.x = self.commands['linear_spd']
        t.angular.z = self.commands['angular_spd']
        self.publish(t)


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
                self.commands['in_linear_spd'] = lin
                self.commands['in_angular_spd'] = ang
            elif data[0] == 's':
                self.commands['visible_slaves'] += int(data[1])
            elif data[0] == 'o':
                self.commands['has_obstacle'] = not self.commands['has_obstacle']

    def terminate(self):
        super(NetworkThread, self).terminate()
        # Need to send something to stop the waiting loop
        tmp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tmp_socket.sendto('0 0', (self.hostname, self.port))
        tmp_socket.close()
        self.s.close()


if __name__ == '__main__':

    # Dictionary that holds the 'global' variables and configuration
    commands = {
        'nb_slaves': 0,  # number of slaves in the formation
        'visible_slaves': 0,  # When != to nb_slaves, the master enters Search state
        'linear_spd': 0,  # linear speed of the robot, after transformation according to the state
        'angular_spd': 0,  # angular speed of the robot, after transformation according to the state
        'in_linear_spd': 0,  # linear speed instruction recieved from the remote controller
        'in_angular_spd': 0,  # linear speed instruction recieved from the remote controller
        'has_obstacle': False,  # should become True when a slave notices an obstacle
        'lost_slave_found': False,  # flag that commands the transition from Search to Escort state
        'next_state': state.RemoteControlled  # That state will be applied at the next iteration
    }

    try:
        rospy.init_node('maitre')

        direction = MainThread(commands, True).start()
        network = NetworkThread(commands).start()

        rospy.spin()  # waits until rospy.is_shutdown() is true (Ctrl+C)

        print '\nTerminating the publishers...'
        for thread in (network, direction):
            thread.terminate()
            thread.join()

        print 'All ok.'

    except rospy.ROSInterruptException:
        pass
