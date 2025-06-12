# libraries
import logging
import sys
import time
import math
import numpy as np # for quintic polynomial
from threading import Event

# crazyflie libraries
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.localization import LighthouseConfigWriter
from cflib.utils.reset_estimator import reset_estimator
from cflib.positioning.position_hl_commander import PositionHlCommander

# global variable/constants
URI = 'radio://0/78/2M/E7E7E7E7E5' # update depending on crazyflie's radio id
LH_CONFIG_PATH = "C/temp/path" # insert path to lighthouse configuration file
DEFAULT_HEIGHT = 0.5
DEFAULT_VELOCITY = 0.5
deck_attached_event = Event()
logging.basicConfig(level=logging.ERROR)

# check flow deck is attached
def param_deck_flow(name, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

# call back check for writing lighthouse config to crazyflie
def data_stored_callback():
    print("Configuration data has been successfully stored.")

# set initial position of the drone
def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)

# run the sequence by sending the crazyflie to waypoints via position_hl_commander
def run_sequence(scf, sequence, base_x, base_y, base_z, yaw):
    cf = scf.cf

    # Arm the Crazyflie
    cf.platform.send_arming_request(True)
    time.sleep(1.0)

    for position in sequence:
        if len(position) == 4:
            x, y, z, yaw = position
        else:
            x, y, z = position
            yaw = yaw  # keep previous or default to 0

        print(f'Setting position {x:.2f}, {y:.2f}, {z:.2f}, yaw: {yaw:.2f}')

        for i in range(50):
            with PositionHlCommander(scf, default_velocity=DEFAULT_VELOCITY,default_height=DEFAULT_HEIGHT) as pc:
                cf.pc.go_to(x, y, z=z, velocity=DEFAULT_VELOCITY)
                time.sleep(0.1)

    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

# land at the x y position it is currently at
def land(scf):
     with PositionHlCommander(scf, default_velocity=DEFAULT_VELOCITY,default_height=DEFAULT_HEIGHT) as pc:
         pc.land(velocity=DEFAULT_VELOCITY, landing_height= 0.01)
         scf.cf.commander.send_stop_setpoint()

# solve the quintic polynomial
def quintic_polynomial(p0, pf, v0=0, vf=0, a0=0, af=0, T=5):
    # Solve for coefficients that satisfy boundary conditions
    M = np.array([
        [0,     0,     0,    0,   0, 1],
        [T^5, T^4, T^3, T^2, T, 1],
        [0,     0,     0,    0,   1, 0],
        [5*T^4, 4*T^3, 3*T^2, 2*T, 1, 0],
        [0,     0,     0,    2,   0, 0],
        [20*T^3, 12*T^2, 6*T, 2, 0, 0]
    ])
    b = np.array([p0, pf, v0, vf, a0, af])
    return np.linalg.solve(M, b)

# calculate a trajectory to be completed in T seconds using the quintic polynomial
def calc_polyTrajectory(goal, base_x, base_y, base_z, base_yaw, T=5, freq=50):
    # Generate arrays of trajectory setpoints using quintic polynomial interpolatoin
    steps = int(T * freq)
    t_vals = np.linspace(0, T, steps)

    coeffs_x = quintic_polynomial(T, base_x, goal[0])
    coeffs_y = quintic_polynomial(T, base_y, goal[1])
    coeffs_z = quintic_polynomial(T, base_z, goal[2])
    coeffs_yaw = quintic_polynomial(T, base_yaw, goal[3])

    sequence = []
    for t in t_vals:
        x = np.polyval(coeffs_x, t)
        y = np.polyval(coeffs_y, t)
        z = np.polyval(coeffs_z, t)
        yaw = np.polyval(coeffs_yaw, t)
        # Convert yaw from radians to degrees
        yaw = math.degrees(yaw)
        # Append the setpoint to the sequence
        sequence.append((x, y, z, yaw)) # 4D setpoint

    return sequence

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # initialize and config lighthouse
    lighthouse_config = LighthouseConfigWriter()
    lighthouse_config.write_and_store_config_from_file(data_stored_callback, LH_CONFIG_PATH)

    # Set these to the position and yaw based on how your Crazyflie is placed
    # on the floor
    initial_x = 0.0
    initial_y = 0.0
    initial_z = 0.0
    initial_yaw = 90  # In degrees
    # 0: positive X direction
    # 90: positive Y direction
    # 180: negative X direction
    # 270: negative Y direction

    # set goal coordinate
    goal_x = 1.0
    goal_y = 1.0
    goal_z = 1.0
    goal_yaw = 0
    goal = [goal_x, goal_y, goal_z, goal_yaw]

    # Calculate trajectory using quintic polynomial interpolation
    sequence = calc_polyTrajectory(goal, initial_x, initial_y, initial_z, initial_yaw)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',cb=param_deck_flow)
        time.sleep(1)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
        reset_estimator(scf)
        run_sequence(scf, sequence, initial_x, initial_y, initial_z, initial_yaw)
        time.sleep(10) # hover for 10 seconds
        land(scf)