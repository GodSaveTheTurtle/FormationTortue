#!/usr/bin/env python

import rospy

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

        # TODO networking

        rospy.spin()  # waits until rospy.is_shutdown() is true (Ctrl+C)

    except rospy.ROSInterruptException:
        pass
