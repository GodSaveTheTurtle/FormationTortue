#!/usr/bin/env python

from __future__ import division

import socket

import rospy
from geometry_msgs.msg import Twist

from settings import Settings, SlaveData
from thread_utils import StateSwitcher, RosThread, SimpleSubscriber
import state
from nav_msgs.msg import Odometry


class MainThread(StateSwitcher):

    def __init__(self, shared_data, target_sim=False, name='turtleX'):
        if target_sim:
            topic = '/{}/cmd_vel'.format(name)
        else:
            topic = '/cmd_vel_mux/input/teleop'
        super(MainThread, self).__init__(shared_data, topic, Twist)


class MasterListener(RosThread):
    BUFFER_SIZE = 256

    def __init__(self, shared_data, ip):
        super(MasterListener, self).__init__()
        self._shared_data = shared_data
        self._socket = None
        self._master_ip = ip

    def start(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.connect((self._master_ip, self._shared_data.slave_port))
        self._socket.send(self._shared_data.self_color)
        super(MasterListener, self).start()

    def loop(self):
        while self._running and not rospy.is_shutdown:
            data = self._socket.recv(MasterListener.BUFFER_SIZE)
            self._shared_data.slaves[self._shared_data.self_color] = SlaveData(data)
        self._socket.close()


class OdometrySubscriber(SimpleSubscriber):
    def __init__(self):
        # TODO if sim: use /turtleX/pose
        super(OdometrySubscriber, self).__init__('/odom', Odometry)

    def update(self, data):
        print data


def setup():
    shared_data = Settings()
    shared_data.next_state = state.Obey
    shared_data.sim_mode = rospy.get_param('sim_mode', False)
    return shared_data


if __name__ == '__main__':

    shared_data = setup()

    try:
        rospy.init_node('slave')
        name = rospy.get_param('~name', 'slave')

        if shared_data.sim_mode:
            from turtlesim.srv import Spawn
            rospy.wait_for_service('spawn')
            spawner = rospy.ServiceProxy('spawn', Spawn)
            spawner(8, 5, 0, name)
            # TODO populate virtual slave data

        rospy.loginfo(shared_data)
        rospy.loginfo(name)

        threads = []
        threads.append(MainThread(shared_data, shared_data.sim_mode, name).start())
        threads.append(MasterListener(name, shared_data).start())

        rospy.spin()  # waits until rospy.is_shutdown() is true (Ctrl+C)

        # Sequentially terminate the threads in reverse declaration order
        threads.reverse()
        rospy.logdebug('\nTerminating the threads...')
        for thread in threads:
            thread.terminate()
            thread.join()

        rospy.logdebug('All ok.')
    except rospy.ROSInterruptException:
        pass
