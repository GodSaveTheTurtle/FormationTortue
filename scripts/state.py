import math

import rospy

import testFormation
import testEsclave

from detection_couleur import ColorTracking
from sim_utils import TurtleSimTracker


class State(object):

    def __init__(self, commands):
        ''' Constructor, do stuff required when entering that state '''
        commands.next_state = None
        pass

    def update(self, commands):
        ''' Effect of the state: examine the command dict, compute the commands to be executed.'''
        pass

    def end(self, commands):
        ''' Stuff done before exiting the state '''
        pass


#####################################################################################################################
### Master States
#####################################################################################################################

class RemoteControlled(State):
    ''' Standard master state. Controlled from the android app, and sends instructions to all the slaves '''

    LIN_SPEED_MULT = 1/500.0  # Multiplier for the linear speed (input is bewteen -100 and +100)
    ANG_SPEED_MULT = 1/75.0  # Multiplier for the angular speed (input is bewteen -100 and +100)

    def __init__(self, commands):
        super(RemoteControlled, self).__init__(commands)
        if commands.sim_mode:
            self._ctrack = TurtleSimTracker(commands.slaves).start()
        else:
            self._ctrack = ColorTracking(commands.slaves)
        # TODO send something to the android client to display the joysticks

    def update(self, commands):
        super(RemoteControlled, self).update(commands)

        if commands.connected_slaves < commands.nb_slaves:
            rospy.loginfo('connected slaves: %d/%d', commands.connected_slaves, commands.nb_slaves)
        elif commands.has_obstacle:
            commands.next_state = Obstacle
        elif commands.visible_slaves != commands.nb_slaves:
            commands.next_state = Search
        else:
            # Update own's speed
            commands.linear_spd = commands.in_linear_spd * RemoteControlled.LIN_SPEED_MULT
            commands.angular_spd = commands.in_angular_spd * RemoteControlled.ANG_SPEED_MULT

            self.notify_slaves(commands)

    def notify_slaves(self, commands):
        # Compute instructions
        testFormation.run(commands.slaves)
        rospy.logdebug(commands.slaves)

        # Send to slaves
        for slave in commands.slaves:
            slave_data = commands.slaves[slave]
            slave_data.master_theta_rad = commands.orientation
            slave_data.send()

    def end(self, commands):
        super(RemoteControlled, self).end(commands)
        self._ctrack = None  # TODO: is there something to stop?
        commands.linear_spd = 0
        commands.angular_spd = 0


class Obstacle(State):
    ''' Currently only stops the robot while there is a detected obstacle '''
    def __init__(self, commands):
        super(Obstacle, self).__init__(commands)
        # TODO stop slaves, send something to the android client

    def update(self, commands):
        super(Obstacle, self).update(commands)

        if commands.has_obstacle:
            commands.linear_spd = 0
            commands.angular_spd = 0
        else:
            commands.next_state = RemoteControlled


class Search(State):
    ''' Should rotate on the same spot to locate lost slaves '''

    def __init__(self, commands):
        super(Search, self).__init__(commands)
        # TODO stop slaves, send something to the android client
        self.remaining_search_time = 10 * 600  # in 10th of seconds. This is 5 minutes

    def update(self, commands):
        super(Search, self).update(commands)
        self.remaining_search_time += 1

        if self.remaining_search_time <= 0:
            commands.nb_slaves -= 1  # a slave has successfully escaped
            commands.next_state = RemoteControlled
        elif commands.lost_slave_found:
            commands.next_state = Escort
        else:
            commands.linear_spd = 0
            commands.angular_spd = 3


class Escort(State):
    '''
    Once the lost slave is located, commands it to move back to its position in formation, and keeps rotating
    to keep it visible.
    '''

    def __init__(self, commands):
        super(Escort, self).__init__(commands)
        # TODO send something to the android client

    def update(self, commands):
        super(Escort, self).update(commands)
        #TODO command stray slave


#####################################################################################################################
### Slave States
#####################################################################################################################

class Wait(State):
    ''' Stops moving and waits. Used when the master is searching for other slaves '''
    def __init__(self, commands):
        super(Wait, self).__init__(commands)
        # TODO send something to the android client

    def update(self, commands):
        super(Wait, self).update(commands)
        #TODO command stray slave


class Obey(State):
    ''' Slave standard state. Moves according to the received instructions '''

    def __init__(self, commands):
        super(Obey, self).__init__(commands)
        # TODO send something to the android client

    def update(self, commands):
        super(Obey, self).update(commands)

        rospy.loginfo('in: %s', commands.slaves[commands.self_color])
        #angle, speed = testEsclave.trigonometrique_bis(commands.slaves[commands.self_color], commands.orientation)
        #angle, speed = testEsclave.deterministe(commands.slaves[commands.self_color], commands.orientation)
        # angle, speed = testEsclave.deterministe_bis(commands.slaves[commands.self_color], commands.orientation)
        angle, speed = testEsclave.deterministe_bis(commands.slaves[commands.self_color], commands.orientation)

        rospy.loginfo('out: {angle: %f, speed: %f}', angle, speed)

        # Linear speed of the slave
        commands.linear_spd = min(speed, 0.5)

        # Angular speed of the slave
        commands.angular_spd = angle
