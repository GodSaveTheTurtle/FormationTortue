#!/usr/bin/env python

import rospy

if __name__ == '__main__':

    simu_mode = rospy.get_param('~simu_mode') or False

    if simu_mode:
        import roslib
        roslib.load_manifest('formation')
        # from geometry_msgs.msg import Spawn
        from turtlesim.srv import Spawn

        rospy.wait_for_service('spawn')
        spawner = rospy.ServiceProxy('spawn', Spawn)
        spawner(4, 2, 0, 'turtle2')

        # TODO populate virtual slave data

    try:
        rospy.init_node('esclave')

        # TODO etats

        rospy.spin()  # waits until rospy.is_shutdown() is true (Ctrl+C)

    except rospy.ROSInterruptException:
        pass
