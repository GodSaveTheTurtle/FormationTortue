#!/usr/bin/env python

import rospy
import socket
from publisher import ThreadedPublisher
from geometry_msgs.msg import Twist


class DirectionController(ThreadedPublisher):

    def __init__(self, commands, target_sim=False):
        if target_sim:
            topic = '/turtle1/cmd_vel'
        else:
            topic = '/cmd_vel_mux/input/teleop'
        super(DirectionController, self).__init__(topic, Twist, 1/10.0)
        self.commands = commands

    def update(self):
        ''' Replace this thread's loop with the listener's activation '''
        t = Twist();
        t.linear.x = self.commands['linear_spd']
        t.angular.z = self.commands['angular_spd']
        self.publish(t)


class NetworkThread(ThreadedPublisher):
    def __init__(self, commands):
        ''' That's not really a publisher but w/e, the threading is done easily that way '''
        super(NetworkThread, self).__init__(None, None)
        self.commands = commands
        self.s = None

    def start(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind(('0.0.0.0', 1337))
        super(NetworkThread, self).start()

    def loop(self):
        ''' Replace this thread's loop with a blocking wait on the socket '''
        while self._running and not rospy.is_shutdown():
            data = self.s.recv(1024).split(" ")
            lin, ang = [int(i) for i in data]
            t = Twist();
            self.commands['linear_spd'] = lin/100.0
            self.commands['angular_spd'] = ang/10.0

    def terminate(self):
        super(NetworkThread, self).terminate()
        self.s.close()

            

if __name__ == '__main__':

    # Dictionary that holds the 'global' variables
    commands = {
        'linear_spd': 0,
        'angular_spd': 0
    }

    try:
        rospy.init_node('maitre')

        direction = DirectionController(commands, True).start()
        network = NetworkThread(commands).start()

        rospy.spin()  # waits until rospy.is_shutdown() is true (Ctrl+C)

        print '\nTerminating the publishers...'
        for thread in (direction, network):
            thread.terminate()
            thread.join()

        print 'All ok.'

    except rospy.ROSInterruptException:
        pass
