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

    def __init__(self, shared_data):
        if shared_data.sim_mode:
            topic = '/turtle1/cmd_vel'
        else:
            topic = '/cmd_vel_mux/input/teleop'
        super(MainThread, self).__init__(shared_data, topic, Twist)

    def update(self):
        super(MainThread, self).update()

        t = Twist()
        t.linear.x = self._shared_data.linear_spd
        t.angular.z = self._shared_data.angular_spd
        rospy.logdebug(t)
        self.publish(t)


class RemoteControlListener(RosThread):
    BUFFER_SIZE = 256

    def __init__(self, shared_data):
        super(RemoteControlListener, self).__init__()
        self.shared_data = shared_data
        self.s = None
        self.hostname = ''
        self.port = shared_data.control_port

    def start(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind((self.hostname, self.port))
        return super(RemoteControlListener, self).start()

    def loop(self):
        ''' Replace this thread's loop with a blocking wait on the socket '''
        while self._running and not rospy.is_shutdown():
            data = self.s.recv(RemoteControlListener.BUFFER_SIZE).split(" ")
            rospy.logdebug('Remote command: %s', data)
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
    ''' Waits for all the slaves to be connected, in the current thread (blocks the rest of the execution '''
    BUFFER_SIZE = 256

    def __init__(self, shared_data):
        self._shared_data = shared_data
        self._shared_data.connected_slaves = 0
        self._socket = None

    def _register_connection(self, *connection_info):
        conn, addr = connection_info
        data = conn.recv(SlaveSocketServer.BUFFER_SIZE)

        try:
            self._shared_data.slaves[Color.reverse_mapping[data]].conn = conn
            rospy.loginfo('Registered the %s slave', data)
            self._shared_data.connected_slaves += 1
        except KeyError:
            conn.close()
            rospy.logerr('Invalid slave id: %s', data)

    def start(self):
        ''' Note: blocks until all the slaves are connected '''
        address = ''  # The socket will listen on all interfaces
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((address, self._shared_data.slave_port))
        self._socket.listen(self._shared_data.nb_slaves)

        while self._shared_data.connected_slaves < self._shared_data.nb_slaves and not rospy.is_shutdown():
            try:
                c, a = self._socket.accept()
                self._register_connection(c, a)
                # Thread(target=self._register_connection, args=self._socket.accept()).start()
            except socket.error, e:
                rospy.loginfo('Socket error: %s', e)

        return self

    def terminate(self):
        self._socket.shutdown(socket.SHUT_RDWR)  # Closes the connections
        self._socket.close()

    def join(self):
        pass


def setup():
    shared_data = Settings()
    shared_data.next_state = state.RemoteControlled
    shared_data.sim_mode = rospy.get_param('sim_mode', False)

    slaves = rospy.get_param('master/slaves', ['yellow'])
    shared_data.nb_slaves = len(slaves)
    shared_data.visible_slaves = len(slaves)  # To disable switch to search mode

    for slave_name in slaves:
        shared_data.slaves[slave_name] = SlaveData()

    return shared_data


if __name__ == '__main__':
    try:
        rospy.init_node('turtle_alpha')

        shared_data = setup()
        rospy.loginfo(shared_data)

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
            rospy.loginfo('Terminated: %s', type(thread).__name__)

        rospy.logdebug('All ok.')

    except rospy.ROSInterruptException:
        pass
