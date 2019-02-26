import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import csv


from planning_utils import a_star, heuristic, create_grid, prune_path, adjust_bearing, read_home
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL
        self.goal_global_position = goal_global_position # added to get global position from drone

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            print('Local Position callback for takeoff !')
            # print('DEBUG: self.local_position[2]: ', self.local_position[2])
            # print('DEBUG: self.target_position[2]: ', self.target_position[2])
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        print('DEBUG: target_position[2]: '.format(self.target_position[2]))
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        print('DEBUG Waypoint: {0}'.format(self.waypoints)) # check waypoint to see if empty
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        colliders_file = 'colliders.csv'
        
        lat0, lon0 = read_home(colliders_file)
        print(f'Home lat: {lat0}, lon: {lon0}')
            
        # TODO: set home position to (lon0, lat0, 0) from udacidrone api
        self.set_home_position = (lon0, lat0, 0)
        print('DEBUG drone home lon0: {0}   lat0: {1}, alt: 0.0'.format(lon0, lat0))
        # TODO: retrieve current global position
        local_north, local_east, local_down = global_to_local(self.global_position, self.global_home)
        print(f'Local values  North: {local_north}, East: {local_east}, Down: {local_down}')
        
        # TODO: convert to current local position using global_to_local()
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=3)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        grid_start_north = int(np.ceil(local_north - north_offset))
        grid_start_east = int(np.ceil(local_east - east_offset))
        print('Grid start N: {0}  E: {1}'.format(grid_start_north, grid_start_east))
        grid_start = (grid_start_north, grid_start_east)
        # TODO: convert start position to current position rather than map center
        # grid_start = ((start_north + -north_offset), (start_east + -east_offset))
        
        # Set goal as some arbitrary position on the grid
        #grid_goal = (grid_start[0] + 10, grid_start[1] + 10)
        goal_north, goal_east, goal_alt = global_to_local(self.goal_global_position, self.global_home)
        grid_goal = int(np.ceil(goal_north - north_offset)), int(np.ceil(goal_east - east_offset))
        
        # TODO: adapt to set goal as latitude / longitude position and convert converted above 

        # Run A* to find a path from start to goal
        """
        NOTE: if simulator times out - it may need to be restarted
        """
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print(path)
        # TODO: prune path to minimize number of waypoints
        print("pruning the path")
        path = prune_path(path)
        print('DEBUG: after pruning the path:')
        print(path)
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        
        # Convert path to waypoints
        waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        print('show 1st waypoint: ', waypoints[0])

        # add bearing to waypoints 
        waypoints = adjust_bearing(waypoints)
        print('waypoints with bearing: ', waypoints[1])
        
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--goal_lon', type=str, default='-122.39755', help="Goal longitude")
    parser.add_argument('--goal_lat', type=str, default='37.79258', help="Goal latitude")
    parser.add_argument('--goal_alt', type=str, default='0', help ="Goal altitude")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    goal_global_position = np.fromstring(f'{args.goal_lon},{args.goal_lat},{args.goal_alt}',dtype='Float64', sep=',')
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
