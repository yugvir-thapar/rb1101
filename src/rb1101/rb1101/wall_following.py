import random
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from numpy.linalg import norm
import time
from rclpy.logging import set_logger_level, LoggingSeverity


np.set_printoptions(2, suppress=True) # Print numpy arrays to specified d.p. and suppress scientific notation (e.g. 1e-5)
set_logger_level("WallFollowingNode", level=LoggingSeverity.INFO) # Configure to either LoggingSeverity.DEBUG instead to print more details during run

is_simulation = True

# Movement mode:
#   "ROTATE" - turn the robot body 90deg at corners
#   "STRAFE" - never rotate; slide sideways/forward and change travel direction at corners instead
#   "ARC"    - drive quarter-circle arcs around corners (radius = right_wall_buffer_distance)
movement_mode = "STRAFE"
assert movement_mode in ("ROTATE", "STRAFE", "ARC"), f"Unknown movement_mode: {movement_mode}"

# Algorithm (when to leave the wall), works with any movement_mode:
#   "PLAIN"   - always follow the right wall
#   "HEADING" - leave the wall (don't turn right at an outside corner) whenever facing the initial heading (turn sum mod 4 == 0)
#   "PLEDGE"  - leave the wall only when the cumulative turn sum is exactly 0 (left = +1, right = -1)
algorithm = "PLEDGE"
assert algorithm in ("PLAIN", "HEADING", "PLEDGE"), f"Unknown algorithm: {algorithm}"

# Stop for good after this many 90deg turns (every left or right corner counts as 1). None = no limit.
max_turns = 12

if is_simulation:
    # Simulation settings
    parameters=[
        ('ransac.start_inliers', 15), # 25
        ('ransac.inliers_decrease_factor', 5), # 3
        ('ransac.max_iter', 20), # 100
        ('ransac.point_distance_threshold', 0.03), # 0.01
        ('translate_velocity', 0.8),
        ('translate_limit', 1.4),
        ('turn_velocity', 0.6),
        ('turn_limit', 2.8), 
        ('diagonal_x_threshold', (0.2, 0.5)), # (min, max) thresholds for wall detection on left/right/diagonal walls
        ('diagonal_y_threshold', (0.2, 0.5)), # (min, max) thresholds for wall detection on left/right/diagonal walls
        ('robot_dimensions', (0.23, 0.1)),  # Length x Width
        ('vertical_threshold', 0.25), # Front threshold distance to wall
        ('right_wall_buffer_distance', 0.2),
        ('angle_tolerance', 5),
        ('slide_correction_factor', 3),
        ('turn_correction_factor', 2)
    ]

else:
    # IRL settings
    parameters=[
        ('ransac.start_inliers', 15), # 25
        ('ransac.inliers_decrease_factor', 5), # 3
        ('ransac.max_iter', 20), # 100
        ('ransac.point_distance_threshold', 0.03), # 0.01
        ('translate_velocity', 0.2), # Feel free to toggle this speed, but be aware that the robot may miss walls at higher speeds
        ('translate_limit', 0.4), # DO NOT CHANGE THIS LIMIT; marks will be penalised for changing irl speed limits
        ('turn_velocity', 0.6),
        ('turn_limit', 1.0), # DO NOT CHANGE THIS LIMIT; marks will be penalised for changing irl speed limits
        ('diagonal_x_threshold', (0.05, 0.5)), # (min, max) thresholds for wall detection on left/right/diagonal walls
        ('diagonal_y_threshold', (0.05, 0.5)), # (min, max) thresholds for wall detection on left/right/diagonal walls
        ('robot_dimensions', (0.1, 0.05)),  # Length x Width
        ('vertical_threshold', 0.25), # Front threshold distance to wall
        ('right_wall_buffer_distance', 0.16),
        ('angle_tolerance', 15),
        ('slide_correction_factor', 4),
        ('turn_correction_factor', 1)
    ]


def ransac(points,maxIter,threshold,d):
    '''RANSAC (Random Sample Consensus) implementation modified from https://github.com/creminem94/Advanced-Wall-Following/tree/main'''
    i = 0
    bestLine = None
    bestB = None
    bestC = None
    bestInliers = list()
    bestOutliers = list()
    if len(points) < 2:
        return bestLine, bestB, bestC, bestInliers, bestOutliers # Need at least 2 points to define a line
    random.seed()
    while i < maxIter:
        i = i+1
        idx1 = random.randrange(len(points))
        idx2 = random.randrange(len(points))
        while idx2 == idx1:
            idx2 = random.randrange(len(points))
        
        #model
        B = points[idx1]
        C = points[idx2]
        inliers = list()
        outliers = list()
        for j  in range(0, len(points)):
            if j == idx1 or j == idx2:
                continue
            A = points[j]
            dist = point2lineDist(A,B,C)
            if abs(dist) <= threshold:
                inliers.append(A)
            else:
                outliers.append(A)

        if len(inliers) >= d:
            bestLine = (B, C) # Line is defined by the two sampled points
            bestB = B
            bestC = C
            bestInliers = inliers
            bestOutliers = outliers
            break
    
    return bestLine, bestB, bestC, bestInliers, bestOutliers


def point2lineDist(A,B,C):
    '''Helper function for ransac()'''
    #A is the point to "project", B and C are the points that define the
    return norm(np.cross(C-B, B-A))/norm(C-B)


class WallFollowingNode(Node):
    def __init__(self, parameters:list):
        super().__init__('WallFollowingNode')
        self.get_logger().info("Starting WallFollowingNode")
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) # Publish to cmd_vel node

        # Subscribe to the scan ROS topic, which is what the lidar (whether simulated or real) publishes to
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) 
        
        self.timer = self.create_timer(0.1, self.timer_callback)  # Callback to run wall-following logic, runs at 10Hz.

        self.heading = 0 # Direction faced relative to start, in 90deg steps (0-3, 0 = initial heading). = heading_sum mod 4
        self.heading_sum = 0 # Cumulative turns: left = +1, right = -1 (used by PLEDGE)
        self.turn_count = 0 # Total number of 90deg turns, left or right (used for max_turns)
        self.finished = False # True once max_turns is reached; the robot then stays stopped
        self.ransacLineParams = []
        self.cooldown = 0
        self.last_scan = None
        self.scan = None # last_scan rotated into the travel frame (index 0 = travel direction)
        self.travel_dir = 0 # Travel direction relative to robot body, in degrees CCW: 0=forward, 90=left, 180=back, 270=right. Always 0 in ROTATE mode.
        self.declare_parameters(namespace='', parameters=parameters)
        self.get_logger().info(f"Movement mode: {movement_mode}, algorithm: {algorithm}, max turns: {max_turns}")
        if movement_mode == "ARC":
            self.check_arc_clearance()


    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        if len(msg.ranges) <= 360:
            self.last_scan = np.array(msg.ranges)
        else:
            self.last_scan = np.array(msg.ranges)[::2] 
            if is_simulation:
                self.get_logger().error("Error: is_simulation is set to True when code is being used on real robot")
                raise ValueError


    def fitRansacLines(self, nInliers, start_idx:int=180, stop_idx:int=345):
        '''RANSAC (Random Sample Consensus) implementation modified from https://github.com/creminem94/Advanced-Wall-Following/tree/main'''
        maxIter = self.get_parameter('ransac.max_iter').value
        threshold = self.get_parameter('ransac.point_distance_threshold').value
        
        if stop_idx > self.scan.size:
            ranges = np.append(self.scan, self.scan[:stop_idx-self.scan.size])
        else:
            ranges = self.scan
        scan_x = ranges[start_idx:stop_idx] * np.cos(np.deg2rad(np.arange(start_idx,stop_idx)))
        scan_y = ranges[start_idx:stop_idx] * np.sin(np.deg2rad(np.arange(start_idx,stop_idx)))
        scan_ranges = ranges[start_idx:stop_idx]
        valid_indices = np.isfinite(scan_x) & np.isfinite(scan_y) & (scan_ranges > 0) # Drop inf/nan/zero (no-return) readings
        valid_indices &= np.logical_and(scan_x < 0.3, scan_y < 0.3)
        points2fit = np.dstack((scan_x[valid_indices], scan_y[valid_indices]))[0]

        ransacLineParams = [] # contains list of pair of points that identify each line
        while(True):
            line, B, C, inliers, outliers = ransac(
                points2fit, maxIter, threshold, nInliers)
            if line is None:
                break

            # Angle of the wall relative to the robot's heading, in [-90, 90).
            # Positive = robot is turned left of the wall (so correct by turning right).
            # A line has no direction, so wrap modulo 180 deg.
            angle = -np.rad2deg(np.arctan2(C[1]-B[1], C[0]-B[0]))
            angle = (angle + 90) % 180 - 90

            ransacLineParams.append({
                "p1": B,
                "p2": C,
                "angle" : angle
            })
            if len(outliers) <= nInliers or len(ransacLineParams) > 3:
                break
            points2fit = outliers
        if len(ransacLineParams) == 0:
            decreasedInliers = nInliers - self.get_parameter('ransac.inliers_decrease_factor').value
            if decreasedInliers > 0:
                self.get_logger().debug(f'Decreased inliers to: {decreasedInliers}')
                return self.fitRansacLines(decreasedInliers, start_idx, stop_idx)

        return ransacLineParams


    def angle_right(self):
        '''Return the angle (in degrees) relative to the right side wall most parallel to the robot. If no walls detected to the right, return None'''
        if len(self.ransacLineParams) > 0:
            angles = list(map(lambda p: p["angle"], self.ransacLineParams)) 
            smallest_angle_idx = np.argmin(np.abs(angles))
            return angles[smallest_angle_idx]
        else:
            return None # No line fitted


    def wall_scan(self, obstacle_threshold:int=5):
        '''Scans and returns a list of Bool with whether a wall is detected in the 8 cardinal and ordinal directions'''
        wall_detected_arr = np.zeros(8)

        dimensions = self.get_parameter('robot_dimensions').value
        # Note: dimensions are deliberately NOT swapped when strafing sideways. Swapping makes the front box
        # (|y| < length = 0.23) wider than the right-wall distance (0.2), so the followed wall reads as "ahead".
        diagonal_x_threshold = self.get_parameter('diagonal_x_threshold').value
        diagonal_y_threshold = self.get_parameter('diagonal_y_threshold').value
        vertical_threshold = self.get_parameter('vertical_threshold').value

        scan_x = self.scan * np.cos(np.deg2rad(np.arange(360)))
        scan_y = self.scan * np.sin(np.deg2rad(np.arange(360)))
        scan_coordinates = np.dstack((scan_x, scan_y))[0]        

        for idx in range(8):
            if idx == 0:
                scan_slice = np.hstack((scan_coordinates[-45:], scan_coordinates[:45]))
            else:
                scan_slice = scan_coordinates[idx*45-45:idx*45+45]

            if idx % 2 == 1: # Diagonal directions
                x_condition = np.logical_and(diagonal_x_threshold[0] < abs(scan_slice[:,0]), abs(scan_slice[:,0]) < diagonal_x_threshold[1])
                y_condition = np.logical_and(diagonal_y_threshold[0] < abs(scan_slice[:,1]), abs(scan_slice[:,1]) < diagonal_y_threshold[1])
                scans_within_sector_indices = np.logical_and(x_condition, y_condition)
            elif idx == 2 or idx == 6: # Left or right
                x_condition = np.abs(scan_slice[:,0]) < dimensions[0]
                y_condition = np.abs(scan_slice[:,1]) < diagonal_y_threshold[1] * 1.3
                scans_within_sector_indices = np.logical_and(x_condition, y_condition)
            else: # Front or back
                x_condition = np.abs(scan_slice[:,0]) < vertical_threshold
                y_condition = np.abs(scan_slice[:,1]) < dimensions[1]
                scans_within_sector_indices = np.logical_and(x_condition, y_condition)

            num_scans_within_sector = np.flatnonzero(scans_within_sector_indices).size
            if num_scans_within_sector > obstacle_threshold:
                wall_detected_arr[idx] = 1

        # Do an extra check for the back right wall (since that's used for right-wall following) to make sure wall detected is parallel to robot
        right_corner_wall = False
        for line in self.back_right_lines:
            if abs(line["angle"]) < 40:
                x_condition = -diagonal_x_threshold[1] < max(line["p1"][0], line["p2"][0]) and min(line["p1"][0], line["p2"][0]) < -dimensions[0]
                y_condition = -diagonal_y_threshold[1] < max(line["p1"][1], line["p2"][1]) and min(line["p1"][1], line["p2"][1]) < -dimensions[1]
                if x_condition and y_condition and wall_detected_arr[5] == 1: # meaning there's a wall that crosses the corner right sector and is roughly parallel with the robot
                    right_corner_wall = True
        if right_corner_wall: wall_detected_arr[5] = 1

        return wall_detected_arr


    def move_2D(self, x:float=0.0, y:float=0.0, turn:float=0.0):
        '''Publishes a Twist message to ROS to move a robot. Inputs are x and y linear velocities, as well as turn (z-axis yaw) angular velocity.'''
        twist_msg = Twist()
        translate_limit = self.get_parameter('translate_limit').value
        turn_limit = self.get_parameter('turn_limit').value
        x = np.clip(x, -translate_limit, translate_limit)
        y = np.clip(y, -translate_limit, translate_limit)
        turn = np.clip(turn, -turn_limit, turn_limit)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.publisher_.publish(twist_msg)


    def move_relative(self, forward:float=0.0, slide:float=0.0, turn:float=0.0):
        '''Like move_2D, but forward/slide are given in the travel frame (forward = travel direction, slide = travel-left).
        Rotates them by travel_dir into robot body velocities. With travel_dir = 0 this is identical to move_2D.'''
        cos_t, sin_t = {0: (1, 0), 90: (0, 1), 180: (-1, 0), 270: (0, -1)}[self.travel_dir]
        x = forward * cos_t - slide * sin_t
        y = forward * sin_t + slide * cos_t
        self.move_2D(x, y, turn)


    def move_forward(self, speed:float=None, turn_offset:float=0.0, align:bool=False):
        '''Moves in the travel direction (body-forward in rotate mode), applying a turn component if given.
        If align is true, applies a sideways component to maintain distance to the (travel-frame) right wall'''
        translate_velocity = self.get_parameter('translate_velocity').value
        if speed is None: speed = translate_velocity
        right_wall_buffer_distance = self.get_parameter('right_wall_buffer_distance').value

        slide=0.0
        if align:
            right_dist = np.min(self.scan[260:280])
            if right_wall_buffer_distance*3 > right_dist > right_wall_buffer_distance*1.2:
                slide = -translate_velocity/self.get_parameter('slide_correction_factor').value
            elif right_dist < right_wall_buffer_distance*0.8:
                slide = translate_velocity/self.get_parameter('slide_correction_factor').value

        self.move_relative(speed, slide, turn_offset)


    def set_travel_dir(self, delta:int):
        '''Strafe mode: change travel direction by delta degrees (+90 = left, -90 = right) without rotating the body'''
        self.travel_dir = (self.travel_dir + delta) % 360
        self.get_logger().info(f"Travel direction now {self.travel_dir} deg relative to body")
        self.cooldown = 1 # Pause one tick so the next decision uses a fresh scan
        self.last_scan = None
        self.register_turn(+1 if delta > 0 else -1)


    def register_turn(self, direction:int):
        '''Record a completed 90deg corner turn (+1 = left, -1 = right) in any movement mode, and stop for good once
        max_turns is reached. Small heading corrections from align_heading are not counted.'''
        self.turn_count += 1
        self.heading_sum += direction
        self.heading = self.heading_sum % 4
        self.get_logger().info(f"Turn {self.turn_count}/{max_turns}: {'left' if direction > 0 else 'right'}, "
                               f"turn sum = {self.heading_sum}, heading = {self.heading * 90} deg")
        if max_turns is not None and self.turn_count >= max_turns:
            self.finished = True
            self.stop()
            self.get_logger().info(f"Reached {max_turns} turns, stopping")


    def may_turn_right(self):
        '''Whether the robot should follow the wall around an outside corner (turn right).
        False means leave the wall and carry straight on towards the initial heading.'''
        if algorithm == "HEADING":
            return self.heading != 0 # Facing the initial heading: go straight
        if algorithm == "PLEDGE":
            return self.heading_sum != 0 # Turns have fully unwound: go straight
        return True # PLAIN: always follow the wall


    def turn_left_90deg(self):
        '''Hardcoded 90deg turn based on turn velocity'''
        self.get_logger().debug("Turn left 90deg")
        turn_velocity = self.get_parameter('turn_velocity').value
        for _ in range(50):
            self.move_2D(0.0, 0.0, turn_velocity)
            time.sleep(0.05)
        self.get_logger().debug("Turn complete")
        self.cooldown = 5
        self.last_scan = None
        self.register_turn(+1)

    def turn_right_90deg(self):
        '''Hardcoded 90deg turn based on turn velocity'''
        self.get_logger().debug("Turn right 90deg")
        turn_velocity = self.get_parameter('turn_velocity').value
        for _ in range(50):
            self.move_2D(0.0, 0.0, -turn_velocity)
            time.sleep(0.05)
        self.get_logger().debug("Turn complete")
        self.cooldown = 5
        self.last_scan = None
        self.register_turn(-1)


    def arc_turn(self, direction:int, radius:float):
        '''Arc mode: drive a quarter-circle of the given radius (direction +1 = left, -1 = right), moving forward and
        turning at the same time. Open-loop, like turn_left_90deg. radius 0 = rotate in place.
        Speed: as fast as translate_velocity allows, but slowed down so the turn rate stays within turn_limit.'''
        dt = 0.05
        translate_velocity = self.get_parameter('translate_velocity').value
        turn_limit = self.get_parameter('turn_limit').value
        omega = turn_limit if radius <= 1e-3 else min(turn_limit, translate_velocity / radius)
        steps = max(1, int(np.ceil((np.pi / 2) / omega / dt))) # Round up so omega stays within turn_limit
        omega = (np.pi / 2) / (steps * dt) # Adjust so steps * dt * omega is exactly 90deg
        v = omega * radius
        self.get_logger().info(f"[arc] {'left' if direction > 0 else 'right'} arc: R={radius:.3f} m, v={v:.2f} m/s, w={omega:.2f} rad/s, {steps*dt:.2f} s")
        for _ in range(steps):
            self.move_2D(v, 0.0, direction * omega)
            time.sleep(dt)
        self.stop()
        self.cooldown = 2
        self.last_scan = None
        self.register_turn(direction)


    def check_arc_clearance(self):
        '''Arc mode geometry check, using only robot_dimensions (L, W) and the buffer distance b, with arc radius R = b:
        - outside corner: pivoting around the wall end at radius b, the robot's inner side clears it if b > W/2
        - inside corner: the swinging robot corners clear both walls if (R + W/2)^2 + (L/2)^2 < (R + b)^2'''
        L, W = self.get_parameter('robot_dimensions').value
        b = self.get_parameter('right_wall_buffer_distance').value
        R = b
        if not b > W / 2:
            self.get_logger().warn(f"[arc] buffer {b} <= half width {W/2}: outside-corner arcs will hit the wall end")
        if not (R + W/2)**2 + (L/2)**2 < (R + b)**2:
            self.get_logger().warn(f"[arc] robot corners may hit walls during inside-corner arcs (L={L}, W={W}, b={b})")


    def stop(self):
        self.move_2D(0.0, 0.0, 0.0)


    def timer_callback(self):
        """Controller loop. Insert path planning and PID control logic here"""
        if self.finished:
            self.stop()
            return # max_turns reached: stay stopped

        if self.last_scan is None:
            self.get_logger().info("No scan detected, pausing...")
            self.move_2D()
            return # Does not run if no scans received from Lidar
        
        if self.cooldown > 0:
            self.last_scan = None
            self.cooldown -= 1
            self.move_2D()
            return # Does not run if on cooldown

        # View the scan in the travel frame: index 0 = travel direction, 270 = travel-right (no-op when travel_dir is 0)
        self.scan = np.roll(self.last_scan, -self.travel_dir)

        # Use ransac to get walls and calculate angle of robot wrt right wall (if any)
        self.front_right_lines = self.fitRansacLines(self.get_parameter('ransac.start_inliers').value, 270, 360)
        self.back_right_lines = self.fitRansacLines(self.get_parameter('ransac.start_inliers').value, 180, 271)
        self.ransacLineParams = self.front_right_lines + self.back_right_lines
        angle = self.angle_right()
        self.get_logger().debug(f"Angle right: {angle}")
        wall_array = self.wall_scan()
        self.get_logger().debug(wall_array)

        if movement_mode == "STRAFE":
            self.step_strafe(wall_array)
        elif movement_mode == "ARC":
            self.step_arc(angle, wall_array)
        else:
            self.step_rotate(angle, wall_array)


    def align_heading(self, angle, wall_array):
        '''Shared by ROTATE and ARC. Returns (handled, turn_offset): handled=True means a command was already sent this tick
        (rotate in place to re-align with the right wall, or creep towards it); otherwise turn_offset is a small steering correction.'''
        # If angle wrt right wall exceeds threshold, rotate in place to re-align before moving
        turn_offset = 0.0 # No heading correction unless a usable right wall is found
        if angle is not None:
            if abs(angle) > 60: # Unlikely to be the right wall; try ransac again with larger scan radius
                self.front_right_lines = self.fitRansacLines(self.get_parameter('ransac.start_inliers').value, 270, 440)
                self.ransacLineParams = self.front_right_lines + self.back_right_lines
                angle = self.angle_right()

            if angle is not None and abs(angle) <= 60:
                turn_offset = -np.deg2rad(angle) * self.get_parameter('turn_correction_factor').value
                if abs(angle) > self.get_parameter('angle_tolerance').value:
                    self.move_2D(0,0,turn_offset)
                    return True, turn_offset
            elif not wall_array[0]:
                self.move_2D(0.1) # Move slowly to get closer to wall
                return True, turn_offset
            # Otherwise there's a wall in front: fall through so the robot turns left instead of creeping into it
        return False, turn_offset


    def step_rotate(self, angle, wall_array):
        '''Rotate mode: align body to the right wall, and turn the body 90deg at corners'''
        handled, turn_offset = self.align_heading(angle, wall_array)
        if handled:
            return

        ###### INSERT CODE HERE ######
        if wall_array[0]: # Wall in-front
            self.get_logger().info("wall front, turn left")
            self.turn_left_90deg()

        elif wall_array[6]: # Wall right
            self.get_logger().info("wall right, move forward")
            self.move_forward(turn_offset=turn_offset, align=True)
        elif wall_array[5] and self.may_turn_right():
            self.get_logger().info('Right corner, turn right')
            self.turn_right_90deg()
        elif wall_array[5]:
            self.get_logger().info(f"Right corner, but {algorithm} says go straight (turn sum = {self.heading_sum})")
            self.move_forward(turn_offset=turn_offset)
        else:
            self.get_logger().info("Nothing detected, moving forward")
            self.move_forward(turn_offset=turn_offset)
        ##### INSERT CODE HERE ######


    def step_strafe(self, wall_array):
        '''Strafe mode: never rotate. Same right-hand rule as rotate mode, but corners change the travel direction
        instead of the body heading. wall_array is already in the travel frame.'''
        if wall_array[0]: # Wall ahead in travel direction
            self.get_logger().info("[strafe] wall ahead, travel left")
            self.stop()
            self.set_travel_dir(+90)
        elif wall_array[6]: # Wall on travel-right
            self.get_logger().info("[strafe] wall right, move along it")
            self.move_forward(align=True)
        elif wall_array[5] and self.may_turn_right(): # Right wall just ended (only back-right corner remains)
            self.get_logger().info("[strafe] right corner, travel right")
            self.stop()
            self.set_travel_dir(-90)
        elif wall_array[5]:
            self.get_logger().info(f"[strafe] right corner, but {algorithm} says go straight (turn sum = {self.heading_sum})")
            self.move_forward()
        else:
            self.get_logger().info("[strafe] nothing detected, moving in travel direction")
            self.move_forward()


    def step_arc(self, angle, wall_array):
        '''Arc mode: same right-hand rule, but corners are driven as quarter-circle arcs.
        Geometry uses only the buffer distance b (and robot dimensions, see check_arc_clearance):
        - inside corner: a left arc of radius R moves the robot R forward and R left, so starting it when the wall ahead
          is R + b away finishes the turn b from that wall. Normally R = b (start at 2b); if the wall is already closer,
          use a tighter arc R = distance - b (down to 0 = rotate in place).
        - outside corner: arc right around the wall end, starting as soon as it passes the robot's right side. Ideally R = b
          (pivot exactly on the wall end). If the end is already e behind the robot (detected late, e.g. at high speed),
          use R = b - e, which still finishes b from the new wall (the arc's closest approach to the wall end is sqrt(b^2+e^2) >= b).'''
        b = self.get_parameter('right_wall_buffer_distance').value

        def nearest(ranges):
            ranges = np.asarray(ranges, dtype=float)
            ranges = ranges[np.isfinite(ranges) & (ranges > 0)] # Ignore inf/nan/zero (no-return) readings
            return ranges.min() if ranges.size else np.inf

        front_dist = nearest(np.hstack((self.scan[350:], self.scan[:11]))) # +-10deg cone straight ahead
        right_dist = nearest(self.scan[260:280]) # Narrow window directly to the right
        back_right_dist = nearest(self.scan[225:260]) # Behind-right: still sees the wall just after its end passes

        # Corners first, so the trigger isn't delayed by heading correction
        if front_dist <= 2 * b or wall_array[0]: # Inside corner
            if wall_array[0]:
                front_dist = min(front_dist, self.get_parameter('vertical_threshold').value)
            radius = float(np.clip(front_dist - b, 0.0, b)) # R = b normally; tighter if the wall is already closer
            self.get_logger().info(f"[arc] wall ahead at {front_dist:.2f} m, arc left")
            self.arc_turn(+1, radius)
            return
        outside_corner = right_dist > 2 * b and back_right_dist < 2 * b # Right side just cleared, wall still behind-right
        if outside_corner and not self.may_turn_right():
            self.get_logger().info(f"[arc] right wall ended, but {algorithm} says go straight (turn sum = {self.heading_sum})")
        elif outside_corner:
            # How far behind the robot the wall end is: the most-forward wall point behind-right (x < 0 there)
            r = self.scan[180:270]
            valid = np.isfinite(r) & (r > 0) & (r < 2 * b)
            end_x = (r[valid] * np.cos(np.deg2rad(np.arange(180, 270)[valid]))).max()
            behind = float(np.clip(-end_x, 0.0, b))
            self.get_logger().info(f"[arc] right wall ended {behind:.2f} m behind, arc right")
            self.arc_turn(-1, b - behind)
            return

        handled, turn_offset = self.align_heading(angle, wall_array)
        if handled:
            return
        if wall_array[6]:
            self.get_logger().info("[arc] wall right, move forward")
            self.move_forward(turn_offset=turn_offset, align=True)
        else:
            self.get_logger().info("[arc] nothing detected, moving forward")
            self.move_forward(turn_offset=turn_offset)


def main(args=None):
    rclpy.init(args=args)
    wall_following_node = WallFollowingNode(parameters)

    try:
        rclpy.spin(wall_following_node)
    except SystemExit:  
        wall_following_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

