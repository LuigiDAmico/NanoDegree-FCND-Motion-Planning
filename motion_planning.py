import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import csv

from planning_utils import a_star, heuristic, create_grid, prune_path
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

    print("States")


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        print("__init__")

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
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
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
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

    def getFirstRowOfCSV(self):

        with open('colliders.csv', 'r') as f:
            reader = csv.reader(f, delimiter=',')
            # get header from first row
            header = next(reader)
       
        # we substringing and assuming the header always is in a standard format of: "lat0 ff.fff, lan0 ff.fff" where ff.fff are numerics of any length and sign (positive or negative)
        return [float(header[0][5:]),float(header[1][6:])]
        
    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_LAT = 37.792653
        TARGET_LON = -122.397075
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # TODO: read lat0, lon0 from colliders into floating point values
        print("# TODO: read lat0, lon0 from colliders into floating point values")
        homeGPSLat, homeGPSLon = self.getFirstRowOfCSV()
        print("homeGPSLon = " +  str(homeGPSLon))
        print("homeGPSLat = " +  str(homeGPSLat))
                
        # TODO: set home position to (lon0, lat0, 0)
        print("# TODO: set home position to (lon0, lat0, 0)")
        self.set_home_position(homeGPSLon, homeGPSLat, 0)
        print("# home position set")

        # TODO: retrieve current global position
        print("# TODO: retrieve current global position")
        currentGPSLat = self._latitude 
        currentGPSLon = self._longitude 
        currentGPSAlt = self._altitude
        print("currentGPSLon = " + str(currentGPSLon))
        print("currentGPSLat = " + str(currentGPSLat))        
        print("currentGPSAlt = " + str(currentGPSAlt))

        # TODO: convert to current local position using global_to_local()
        print("# TODO: convert to current local position using global_to_local()")
        geodetic_current_coordinates = [currentGPSLon, currentGPSLat, currentGPSAlt]
        geodetic_home_coordinates = [homeGPSLon, homeGPSLat, 0]
        print("geodetic_current_coordinates " + str(geodetic_current_coordinates))
        print("geodetic_home_coordinates " + str(geodetic_home_coordinates))
        local_coordinates_NED = global_to_local(geodetic_current_coordinates, geodetic_home_coordinates)
        print("local_coordinates_NED " + str(local_coordinates_NED))

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        
        # Define starting point on the grid (this is just grid center)
        # grid_start = (-north_offset, -east_offset)
        # print("original grid_start " + str(grid_start))

        # TODO: convert start position to current position rather than map center
        print("# TODO: convert start position to current position rather than map center")
        grid_start = (int(-north_offset+local_coordinates_NED[0]), int(-east_offset+local_coordinates_NED[1]))
        print("Overriding start location to " + str(grid_start))

        # Set goal as some arbitrary position on the grid
        # grid_goal = (-north_offset + 10, -east_offset + 20)
        
        # TODO: adapt to set goal as latitude / longitude position and convert
        geodetic_goal_cordinates = [TARGET_LON, TARGET_LAT , TARGET_ALTITUDE] # center of MAP
        print("geodetic_goal_cordinates " + str(geodetic_goal_cordinates))
        goal_coordinates_NED = global_to_local(geodetic_goal_cordinates, geodetic_home_coordinates)
        print("goal_coordinates_NED " + str(goal_coordinates_NED))
        grid_goal = (int(-north_offset+goal_coordinates_NED[0]), int(-east_offset+goal_coordinates_NED[1]))
        print("grid_goal" + str(grid_goal))

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)

        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        
        # TODO: prune path to minimize number of waypoints
        print("path len before pruning " + str(len(path)))
        path = prune_path(path)
        print("path len after pruning " + str(len(path)))
        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
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
	args = parser.parse_args()
	print("__main__")

	conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
	drone = MotionPlanning(conn)
	time.sleep(1)

	drone.start()

