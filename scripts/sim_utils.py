import math

from turtlesim.msg import Pose

from thread_utils import SimpleSubscriber, RosThread


class TurtleSimTracker(RosThread):
    ''' Alternative to using the camera to find the slaves. Uses the appropriate turtlesim topics instead '''

    def __init__(self, slave_data):
        super(TurtleSimTracker, self).__init__(1)
        self.slave_data = slave_data
        self.slave_tackers = {}
        for slave_name in self.slave_data:
            self.slave_tackers[slave_name] = PoseSubscriber(slave_name)
        self.master_tracker = PoseSubscriber('turtle1')

    def start(self):
        self.master_tracker.start()
        for slave_name in self.slave_tackers:
            self.slave_tackers[slave_name].start()
        return super(TurtleSimTracker, self).start()

    def update(self):
        master_pose = self.master_tracker.pose
        if not master_pose:
            return

        for sn in self.slave_data:
            pose = self.slave_tackers[sn].pose
            if pose:
                # Distance
                self.slave_data[sn].d = math.hypot(master_pose.x - pose.x, master_pose.y - pose.y)

                # Angle with the slave, on the world's referential
                self.slave_data[sn].theta_rad = math.atan2(pose.y - master_pose.y, pose.x - master_pose.x)
                # Angle difference with the master's orientation
                self.slave_data[sn].theta_rad = (master_pose.theta - self.slave_data[sn].theta_rad) % (2*math.pi)


class PoseSubscriber(SimpleSubscriber):
    def __init__(self, name):
        super(PoseSubscriber, self).__init__('/{}/pose'.format(name), Pose)
        self.name = name
        self.pose = None

    def update(self, pose):
        self.pose = pose
