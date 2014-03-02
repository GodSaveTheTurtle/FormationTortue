class State(object):

    def __init__(self, commands):
        ''' Constructior, do stuff required when entering that state '''
        commands['state'] = None
        pass

    def update(self, commands):
        ''' Effect of the state: examine the command dict, compute the commands to be executed.'''
        pass

    def end(self, commands):
        ''' Stuff done before exiting the state '''
        pass


class RemoteControlled(State):
    def __init__(self, commands):
        super(RemoteControlled, self).init(commands)
        # TODO send something to the android client to display the joysticks

    def update(self, commands):
        super(RemoteControlled, self).update(commands)
        # TODO: compute what to send to the slaves

        if commands['has_obstacle']:
            commands['next_state'] = Obstacle
        elif commands['visible_slaves'] != commands['nb_slaves']:
            commands['next_state'] = Search


class Obstacle(State):
    ''' Currently only stops the robot while there is a detected obstacle '''
    def __init__(self, commands):
        super(Obstacle, self).init(commands)
        # TODO stop slaves, send something to the android client

    def update(self, commands):
        super(Obstacle, self).update(commands)

        if commands['has_obstacle']:
            commands['linear_spd'] = 0
            commands['angular_spd'] = 0
        else:
            commands['next_state'] = RemoteControlled


class Search(State):
    def __init__(self, commands):
        super(Search, self).init(commands)
        # TODO stop slaves, send something to the android client
        self.remaining_search_time = 10 * 600  # in 10th of seconds. This is 5 minutes

    def update(self, commands):
        super(Search, self).update(commands)
        self.remaining_search_time += 1

        if self.remaining_search_time <= 0:
            commands['next_state'] = RemoteControlled  # TODO: do something to avoid directly getting back to search
        elif commands['lost_slave_found']:
            commands['next_state'] = Escort
        else:
            commands['linear_spd'] = 0
            commands['angular_spd'] = 3


class Escort(State):
    def __init__(self, commands):
        super(Escort, self).init(commands)
        # TODO send something to the android client

    def update(self, commands):
        super(Escort, self).update(commands)
        #TODO command stray slave
