class Settings:
    ''' Dictionary that holds the 'global' variables and configuration'''

    nb_slaves = 0  # number of slaves in the formation
    visible_slaves = 0  # When ! = to nb_slaves the master enters Search state
    linear_spd = 0  # linear speed of the robot after transformation according to the state
    angular_spd = 0  # angular speed of the robot after transformation according to the state
    in_linear_spd = 0  # linear speed instruction recieved from the remote controller
    in_angular_spd = 0  # linear speed instruction recieved from the remote controller
    has_obstacle = False  # should become True when a slave notices an obstacle
    lost_slave_found = False  # flag that commands the transition from Search to Escort state
    next_state = None  # That state will be applied at the next iteration
    sim_mode = False
    slaves = {
        'yellow': {
            'ip': '192.168.0.110',
            'distance': 42,
            'angle': -5
        }
    }
