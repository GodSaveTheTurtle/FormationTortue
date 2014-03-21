
def enum(**enums):
    '''
    No enums until python 3.4. This is used instead.
    Allows to map a member of the returned type to a value, and vice versa through Enum.reverse_mapping
    '''
    reverse = dict((value, key) for key, value in enums.iteritems())
    enums['reverse_mapping'] = reverse
    return type('Enum', (), enums)

# Example usage: `Color.yellow == Color.reverse_mapping['yellow']`
Color = enum(yellow='yellow', pink='pink')


class Printable(object):
    ''' Utility class, allows to print the object like a dict instead of <class....> '''

    def __repr__(self):
        out = '{\n'
        for member_name in dir(self):
            if not member_name.startswith('__'):
                member = getattr(self, member_name)
                if not callable(member):
                    out += '\t%s: %r\n' % (member_name, member)
        out += '}'
        return out


class Settings(Printable):
    '''
    Dictionary that holds the 'global' variables and configuration
    One is instanciated in the main of master, another one in slave's. It is then passed as parameter to other
    threads and functions
    '''

    def __init__(self):
        self.nb_slaves = 0             # number of slaves in the formation
        self.visible_slaves = 0        # When ! = to nb_slaves the master enters Search state
        self.connected_slaves = 0
        self.linear_spd = 0            # linear speed of the robot after transformation according to the state
        self.angular_spd = 0           # angular speed of the robot after transformation according to the state
        self.in_linear_spd = 0         # linear speed instruction recieved from the remote controller
        self.in_angular_spd = 0        # linear speed instruction recieved from the remote controller
        self.has_obstacle = False      # should become True when a slave notices an obstacle
        self.lost_slave_found = False  # flag that commands the transition from Search to Escort state
        self.next_state = None         # That state will be applied at the next iteration
        self.sim_mode = False
        self.slave_port = 1338         # TCP port used between the slaves and the master
        self.control_port = 1337       # UDP port used with the android app
        self.self_color = None         # In simulation mode, it is used as name of the turtle for topic operations
                                       # For the slaves, it's also their identifier in the communications
        self.orientation = 0           # Orientation of the robot, read from /odom or /turtleX/pose
        self.slaves = {
            ## Format: 'color': SlaveData
        }


class SlaveData(Printable):
    ''' Holds the data of a single slave: instructions, connection status, etc. '''

    def __init__(self):
        super(SlaveData, self).__init__()
        self.d = 0                 # current distance from the master
        self.theta_rad = 0         # current angle from the master's orientation
        self.goal_d = 0            # distance instruction
        self.goal_theta_rad = 0    # angle instruction
        self.master_theta_rad = 0  # master's orientation
        self.conn = None           # master side only. Socket used for the communication with the slave

    def update_from_string(self, str_repr):
        ''' Updates the current instance from the string representiation used on network communications '''
        if str_repr:
            tmp = str_repr.split(' ')
            self.d = float(tmp[0])
            self.theta_rad = float(tmp[1])
            self.goal_d = float(tmp[2])
            self.goal_theta_rad = float(tmp[3])
            self.master_theta_rad = float(tmp[4])

    def send(self):
        ''' Serializes and sends the state of current instance to the slave '''
        if self.conn:
            msg = '{} {} {} {} {}'.format(
                self.d, self.theta_rad, self.goal_d, self.goal_theta_rad, self.master_theta_rad)
            self.conn.send(msg)
