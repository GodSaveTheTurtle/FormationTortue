#!/usr/bin/env python

from __future__ import division
import socket
from threading import Thread

import rospy
from geometry_msgs.msg import Twist

import state
from thread_utils import RosThread, StateSwitcher
from data_utils import Settings, SlaveData, Color


class MainThread(StateSwitcher):

    def __init__(self, shared_data, target_sim=False):
        if target_sim:
            topic = '/turtle1/cmd_vel'
        else:
            topic = '/cmd_vel_mux/input/teleop'
        super(MainThread, self).__init__(shared_data, topic, Twist)


class RemoteControlListener(RosThread):

    def __init__(self, shared_data):
        super(RemoteControlListener, self).__init__()
        self.shared_data = shared_data
        self.s = None
        self.hostname = '0.0.0.0'
        self.port = shared_data.control_port

    def start(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((self.hostname, self.port))
        return super(RemoteControlListener, self).start()

    def loop(self):
        ''' Replace this thread's loop with a blocking wait on the socket '''
        while self._running and not rospy.is_shutdown():
            data = self.s.recv(1024).split(" ")
            if data[0] == 'd':  # Direction command
                lin, ang = [int(i) for i in data[1:]]
                self.shared_data.in_linear_spd = lin
                self.shared_data.in_angular_spd = ang
            elif data[0] == 's':
                self.shared_data.visible_slaves += int(data[1])
            elif data[0] == 'o':
                self.shared_data.has_obstacle = not self.shared_data.has_obstacle

    def terminate(self):
        super(RemoteControlListener, self).terminate()
        # Need to send something to stop the waiting loop
        tmp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tmp_socket.sendto('0 0', (self.hostname, self.port))
        tmp_socket.close()
        self.s.close()


class SlaveSocketServer(object):
    BUFFER_SIZE = 256

    def __init__(self, shared_data):
        self._shared_data = shared_data
        self._shared_data.connected_slaves = 0
        self._socket = None

    def _register_connection(self, *connection_info):
        conn, addr = connection_info
        data = conn.recv(SlaveSocketServer.BUFFER_SIZE)
        print data
        try:
            self._shared_data.slaves[Color.reverse_mapping[data]].conn = conn
            self._shared_data.connected_slaves += 1
        except KeyError:
            conn.close()

    def start(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.bind(('0.0.0.0', self._shared_data.slave_port))
        self._socket.listen(self._shared_data.nb_slaves)

        while self._shared_data.connected_slaves < self._shared_data.nb_slaves:
            Thread(target=self._register_connection, args=self._socket.accept())

    def terminate(self):
        self._socket.shutdown()  # Closes the connections
        self._socket.close()

    def join(self):
        pass


def setup():
    shared_data = Settings()
    shared_data.next_state = state.RemoteControlled
    shared_data.sim_mode = rospy.get_param('sim_mode', False)

    for slave_name in rospy.get_param('master/slaves', []):
        shared_data.slaves[slave_name] = SlaveData()
    return shared_data


if __name__ == '__main__':

    shared_data = setup()

    print shared_data

    try:
        rospy.init_node('turtle_alpha')

        threads = []

        threads.append(SlaveSocketServer(shared_data).start())
        threads.append(MainThread(shared_data).start())
        threads.append(RemoteControlListener(shared_data).start())

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
