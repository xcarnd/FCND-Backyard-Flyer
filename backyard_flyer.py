import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5
    

class BackyardFlyer(Drone):
    
    SMALL_ENOUGH = 0.1
    
    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.next_waypoint_idx = 0
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)
        
    def local_position_callback(self):
        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        if self.flight_state == States.TAKEOFF:
            altitude = -1.0 * self.local_position[2]
            if abs(altitude - self.target_position[2]) < BackyardFlyer.SMALL_ENOUGH:
                self.waypoint_transition()
        elif self.flight_state == States.LANDING:
            altitude = -1.0 * self.local_position[2]
            if abs(altitude) < BackyardFlyer.SMALL_ENOUGH:
                self.disarming_transition()

    def velocity_callback(self):
        """
        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_state == States.WAYPOINT:
            v = (self.local_velocity[0] ** 2 + self.local_velocity[1] ** 2 + self.local_velocity[2] ** 2) ** 0.5
            if self.has_reach_waypoint(self.next_waypoint_idx) and v < BackyardFlyer.SMALL_ENOUGH:
                self.waypoint_transition()

    def has_reach_waypoint(self, idx):
        wp_n, wp_e = self.all_waypoints[idx]
        dist = ((self.local_position[0] - wp_n) ** 2 + (self.local_position[1] - wp_e) ** 2) ** 0.5
        return dist < BackyardFlyer.SMALL_ENOUGH

    def state_callback(self):
        """
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            self.manual_transition()

    def calculate_box(self):
        """
        1. Return waypoints to fly a box
        """
        # how about fly a box counter-clockwise?
        self.all_waypoints.clear()
        self.all_waypoints.append((10, 0))
        self.all_waypoints.append((10, -10))
        self.all_waypoints.append((0, -10))
        self.all_waypoints.append((0, 0))

    def arming_transition(self):
        print("arming transition")
        self.take_control()
        self.arm()
        self.set_home_position( self.global_position[0],
                                self.global_position[1],
                                self.global_position[2])
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        print("takeoff transition")
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")
        if self.flight_state == States.WAYPOINT:
            if self.next_waypoint_idx >= 3:
                self.landing_transition()
                return
            else:
                self.next_waypoint_idx += 1
        if self.flight_state == States.TAKEOFF:
            self.calculate_box()
            self.next_waypoint_idx = 0
            self.flight_state = States.WAYPOINT
        next_wp_north, next_wp_east = self.all_waypoints[self.next_waypoint_idx]
        print("Next waypoint: {}, {}".format(next_wp_north, next_wp_east))
        self.cmd_position(next_wp_north, next_wp_east, self.target_position[2], 0)

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
