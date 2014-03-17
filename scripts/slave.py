#!/usr/bin/env python

from __future__ import division

import socket

import rospy
from geometry_msgs.msg import Twist

from data_utils import Settings, SlaveData
from thread_utils import StateSwitcher, RosThread, OdometrySubscriber
import state


class SlaveMainThread(StateSwitcher):

    def __init__(self, shared_data):
        if shared_data.sim_mode:
            topic = '/{}/cmd_vel'.format(shared_data.self_color)
        else:
            topic = '/cmd_vel_mux/input/teleop'
        super(SlaveMainThread, self).__init__(shared_data, topic, Twist)

    def update(self):
        super(SlaveMainThread, self).update()

        t = Twist()
        t.linear.x = self._shared_data.linear_spd
        t.angular.z = self._shared_data.angular_spd
        rospy.logdebug(t)
        self.publish(t)


class MasterListener(RosThread):
    BUFFER_SIZE = 256

    def __init__(self, shared_data, ip):
        super(MasterListener, self).__init__()
        self._shared_data = shared_data
        self._socket = None
        self._master_ip = ip

    def start(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self._socket.connect((self._master_ip, self._shared_data.slave_port))
            rospy.loginfo('Sending: %s', self._shared_data.self_color)
            self._socket.send(self._shared_data.self_color)
        except socket.error, e:
            self._socket = None
            rospy.logerr('Unable to connect to master. \nSocket error: %s', e)
        return super(MasterListener, self).start()

    def loop(self):
        while self._socket and self._running and not rospy.is_shutdown():
            data = self._socket.recv(MasterListener.BUFFER_SIZE)
            if not data:  # Other end of the socket disconnected
                self._running = False
            rospy.logdebug('Instruction read by slave: %s', data)
            self._shared_data.slaves[self._shared_data.self_color].update_from_string(data)
        if self._socket:
            self._socket.close()


def setup_shared_data():
    ''' Must be executed AFTER rospy.init_node so that get get_param calls properly work '''

    shared_data = Settings()
    shared_data.next_state = state.Obey
    shared_data.sim_mode = rospy.get_param('sim_mode', False)
    shared_data.self_color = rospy.get_param('~name', 'turtleX')
    shared_data.slaves[shared_data.self_color] = SlaveData()

    if shared_data.sim_mode:
        shared_data.spawn_position = rospy.get_param('~spawn_position', [5, 7])
    return shared_data


if __name__ == '__main__':
    try:
        rospy.init_node('slave')
        shared_data = setup_shared_data()
        master_ip = rospy.get_param('master_ip', '127.0.0.1')

        if shared_data.sim_mode:
            from turtlesim.srv import Spawn
            rospy.wait_for_service('spawn')
            spawner = rospy.ServiceProxy('spawn', Spawn)
            spawner(shared_data.spawn_position[0], shared_data.spawn_position[1], 0, shared_data.self_color)
            # TODO populate virtual slave data

        threads = []
        threads.append(SlaveMainThread(shared_data).start())
        threads.append(OdometrySubscriber(shared_data).start())
        threads.append(MasterListener(shared_data, master_ip).start())

        rospy.spin()  # waits until rospy.is_shutdown() is true (Ctrl+C)

        # Sequentially terminate the threads in reverse declaration order
        threads.reverse()
        rospy.logdebug('\nTerminating the threads...')
        for thread in threads:
            thread.terminate()
            thread.join()

        rospy.logdebug('All ok.')
    except rospy.ROSInterruptException:
        raise
