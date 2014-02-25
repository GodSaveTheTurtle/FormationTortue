#!/usr/bin/env python

import rospy
import socket
from publishers import ThreadedPublisher
from geometry_msgs.msg import Twist


class DirectionController(ThreadedPublisher):

    def __init__(self, target_sim=False):
        if target_sim:
            topic = '/turtle1/cmd_vel'
        else:
            topic = '/cmd_vel_mux/input/teleop'
        super(DirectionController, self).__init__(topic, Twist, 1/10.0)

        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(('0.0.0.0', 1337))

    def loop(self):
        ''' Replace this thread's loop with the listener's activation '''
        while True:
            data = self.s.recv(1024)
            print data


if __name__ == '__main__':
    try:
        rospy.init_node('maitre')

        direction = DirectionController(False).start()

        rospy.spin()  # waits until rospy.is_shutdown() is true (Ctrl+C)

        print '\nTerminating the publishers...'
        direction.terminate()
        direction.join()

        print 'All ok.'

    except rospy.ROSInterruptException:
        pass
