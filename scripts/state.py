import testFormation
import rospy

from detection_couleur import ColorTracking
from formation.msg import Instruction


class State(object):

    def __init__(self, commands):
        ''' Constructior, do stuff required when entering that state '''
        commands.next_state = None
        pass

    def update(self, commands):
        ''' Effect of the state: examine the command dict, compute the commands to be executed.'''
        pass

    def end(self, commands):
        ''' Stuff done before exiting the state '''
        pass


class RemoteControlled(State):
    LIN_SPEED_MULT = 1/75.0  # Multiplier for the linear speed (input is bewteen -100 and +100)
    ANG_SPEED_MULT = 1/40.0  # Multiplier for the angular speed (input is bewteen -100 and +100)

    def __init__(self, commands):
        super(RemoteControlled, self).__init__(commands)
        self.ctrack = ColorTracking()
        # TODO send something to the android client to display the joysticks

    def update(self, commands):
        super(RemoteControlled, self).update(commands)

        if commands.has_obstacle:
            commands.next_state = Obstacle
        elif commands.visible_slaves != commands.nb_slaves:
            commands.next_state = Search
        else:
            # Update own's speed
            commands.linear_spd = commands.in_linear_spd * RemoteControlled.LIN_SPEED_MULT
            commands.angular_spd = commands.in_angular_spd * RemoteControlled.ANG_SPEED_MULT

            self.notify_slaves(commands)

    def end(self, commands):
        super(RemoteControlled, self).end(commands)
        self.commands.linear_spd = 0
        self.commands.angular_spd = 0

    def notify_slaves(self, commands):
        # What do we see? Computations etc
        self.ctrack.update_slave_positions(commands.slaves)

        # Compute instructions
        testFormation.run(commands.slaves)
        # testFormation.regulationMessagesFlow(self.dicoRobots)
        rospy.logdebug(commands.slaves)

        # Send to slaves
        for slave in commands.slaves:
            commands.slaves[slave]['pub'].publish(
                Instruction(commands.slaves[slave]['tetaSetPoint'], self.dicoRobots[0]['setDistance']))


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
    def __init__(self, commands):
        super(Escort, self).__init__(commands)
        # TODO send something to the android client

    def update(self, commands):
        super(Escort, self).update(commands)
        #TODO command stray slave


class Wait(State):
    def __init__(self, commands):
        super(Wait, self).__init__(commands)
        # TODO send something to the android client

    def update(self, commands):
        super(Wait, self).update(commands)
        #TODO command stray slave


class Obey(State):
    def __init__(self, commands):
        super(Obey, self).__init__(commands)
        # TODO send something to the android client

    def update(self, commands):
        super(Obey, self).update(commands)
        #TODO command stray slave
