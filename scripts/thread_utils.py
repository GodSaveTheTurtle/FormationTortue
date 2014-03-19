import rospy

from threading import Thread

#from kobuki_msgs.msg import Led
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose


from controller import DirectionalKeyListener


class RosThread(object):

    '''
    Creates a thread calling the update() method at the chosen frequency.
    Override loop() to change the function used as target for the thread
    '''

    def __init__(self, frequency=1):
        self._thread = Thread(target=self.loop)
        self._frequency = frequency
        self._running = False

    def loop(self):
        while self._running and not rospy.is_shutdown():
            self.update()
            rospy.sleep(self._frequency)

    def update(self):
        pass

    def terminate(self):
        rospy.loginfo('Terminating %s', type(self).__name__)
        self._running = False

    def start(self):
        ''' Returns self for chaining '''
        self._running = True
        self._thread.start()
        return self

    def join(self):
        self._thread.join()


# Could probably be used through multiple inheritance instead, but w/e...
class ThreadedPublisher(RosThread):

    def __init__(self, topic, type, frequency=1):
        super(ThreadedPublisher, self).__init__(frequency)
        if topic and type:
            self._pub = rospy.Publisher(topic, type)
        else:
            self._pub = None

    def publish(self, *args):
        if self._pub:
            rospy.logdebug(*args)
            self._pub.publish(*args)


class StateSwitcher(ThreadedPublisher):

    ''' Handles switching threads and updating the current thread '''

    def __init__(self, shared_data, topic=None, msg_type=None, freq=1 / 10.0, name='turtleX'):
        super(StateSwitcher, self).__init__(topic, msg_type, freq)
        self._shared_data = shared_data
        self._state = None

    def update(self):
        if self._shared_data.next_state:
            if self._state:
                rospy.loginfo('Quitting state %s', type(self._state).__name__)
                self._state.end(self._shared_data)
            rospy.loginfo('Entering state %s', self._shared_data.next_state.__name__)
            self._state = self._shared_data.next_state(self._shared_data)

        self._state.update(self._shared_data)


class SimpleSubscriber(object):

    ''' Uses the same interface as RosThread/ThreadedPublisher for simplicity in calling code '''

    def __init__(self, topic, msg_type, handler=None):
        self.topic = topic
        self.msg_type = msg_type
        self._sub = None

        if not handler:
            self._handler = self.update
        else:
            self._handler = handler

    def update(self, data):
        pass

    def terminate(self):
        rospy.loginfo('Terminating %s', type(self).__name__)
        self._sub.unregister()

    def start(self):
        rospy.loginfo('Subscribing to %s', self.topic)
        self._sub = rospy.Subscriber(self.topic, self.msg_type, self._handler)
        return self

    def join(self):
        pass


class OdometrySubscriber(SimpleSubscriber):

    def __init__(self, shared_data):
        if shared_data.sim_mode:
            topic = '/{}/pose'.format(shared_data.self_color)
            topic_type = Pose
        else:
            topic = '/odom'
            topic_type = Odometry
        super(OdometrySubscriber, self).__init__(topic, topic_type)
        self._shared_data = shared_data

    def update(self, data):
        rospy.logdebug('Odometry data: %s', data)
        # TODO read from /odom
	if self._shared_data.sim_mode:
        	self._shared_data.orientation = data.theta
	else:
        	self._shared_data.orientation = data.pose.pose.orientation.w
		

#class LedController(ThreadedPublisher):
# class LedController(ThreadedPublisher):
#     def __init__(self):
#         super(LedController, self).__init__('/mobile_base/commands/led1', Led)
#         self.color = 0

#     def update(self):
#         self.color = (self.color + 1) % 4
#         self.publish(self.color)


class DirectionController(ThreadedPublisher):

    LINEAR_SPD = 0.15
    ANGULAR_SPD = 0.5

    KEY_BINDINGS = {
        ord('a'): Twist(Vector3(LINEAR_SPD, 0, 0),
                        Vector3(0, 0, ANGULAR_SPD)),
        ord('z'): Twist(Vector3(LINEAR_SPD, 0, 0),
                        Vector3(0, 0, 0)),
        ord('e'): Twist(Vector3(LINEAR_SPD, 0, 0),
                        Vector3(0, 0, -ANGULAR_SPD)),
        ord('q'): Twist(Vector3(0, 0, 0),
                        Vector3(0, 0, ANGULAR_SPD)),
        ord('s'): Twist(Vector3(-LINEAR_SPD, 0, 0),
                        Vector3(0, 0, 0)),
        ord('d'): Twist(Vector3(0, 0, 0),
                        Vector3(0, 0, -ANGULAR_SPD)),
        ord('w'): Twist(Vector3(-LINEAR_SPD, 0, 0),
                        Vector3(0, 0, -ANGULAR_SPD)),
        ord('x'): Twist(Vector3(-LINEAR_SPD, 0, 0),
                        Vector3(0, 0, ANGULAR_SPD)),
        ord(' '): Twist(Vector3(0, 0, 0),
                        Vector3(0, 0, 0))
    }

    def __init__(self, target_sim=False):
        if target_sim:
            topic = '/turtle1/cmd_vel'
        else:
            topic = '/cmd_vel_mux/input/teleop'
        super(DirectionController, self).__init__(topic, Twist, 1 / 10.0)
        self.key_listener = DirectionalKeyListener(callback=self.on_key_pressed,
                                                   direction_mappings=DirectionController.KEY_BINDINGS)

    def loop(self):
        ''' Replace this thread's loop with the listener's activation '''
        self.key_listener.activate()

    def on_key_pressed(self, direction, key, listener):
        ''' Listener style, to be used with DirectionalKeyListener '''
        self.publish(direction)
