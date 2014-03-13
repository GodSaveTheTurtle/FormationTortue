
def enum(**enums):
    reverse = dict((value, key) for key, value in enums.iteritems())
    enums['reverse_mapping'] = reverse
    return type('Enum', (), enums)


Color = enum(yellow='yellow', pink='pink')


class Printable(object):
    # def __str__(self):
    #     out = '{\n'
    #     for member in dir(self):
    #         if not callable(member) and not member.startswith('__'):
    #             out += '\t%s: %s\n' % (member, getattr(self, member))
    #     out += '}'
    #     return out

    def __repr__(self):
        out = '{\n'
        for member_name in dir(self):
            if not member_name.startswith('__'):
                member = getattr(self, member_name)
                if not callable(member):
                    out += '\t%s: %r\n' % (member_name, member)
        out += '}'
        # return str(dir(self))
        return out


class Settings(Printable):
    ''' Dictionary that holds the 'global' variables and configuration'''

    def __init__(self):
        self.nb_slaves = 1  # number of slaves in the formation
        self.visible_slaves = 0  # When ! = to nb_slaves the master enters Search state
        self.connected_slaves = 0
        self.linear_spd = 0  # linear speed of the robot after transformation according to the state
        self.angular_spd = 0  # angular speed of the robot after transformation according to the state
        self.in_linear_spd = 0  # linear speed instruction recieved from the remote controller
        self.in_angular_spd = 0  # linear speed instruction recieved from the remote controller
        self.has_obstacle = False  # should become True when a slave notices an obstacle
        self.lost_slave_found = False  # flag that commands the transition from Search to Escort state
        self.next_state = None  # That state will be applied at the next iteration
        self.sim_mode = False
        self.slave_port = 1338
        self.control_port = 1337
        self.self_color = None  # To be used only in slaves
        self.slaves = {
            ## Format: 'color': SlaveData
        }


class SlaveData(Printable):

    def __init__(self, str_repr=None):
        super(SlaveData, self).__init__()

        if str_repr:
            pass  # TODO

        self.d = 0              # from Kinect
        self.theta_rad = 0      # from Kinect
        self.goal_d = 0
        self.goal_theta_rad = 0
        self.conn = None

    def send(self):
        if self.conn:
            msg = '{} {} {} {}'.format(self.d, self.theta_rad, self.goal_d, self.goal_theta_rad)
            self.conn.send(msg)
