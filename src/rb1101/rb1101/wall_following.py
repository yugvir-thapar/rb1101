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
            bestLine = np.polyfit(inliers[0], inliers[1], 1)
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

        self.heading = 0 # Assume start facing heading 0
        self.ransacLineParams = []
        self.cooldown = 0
        self.last_scan = None

        self.declare_parameters(namespace='', parameters=parameters)


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
        
        if stop_idx > self.last_scan.size:
            ranges = np.append(self.last_scan, self.last_scan[:stop_idx-self.last_scan.size])
        else:
            ranges = self.last_scan
        scan_x = ranges[start_idx:stop_idx] * np.cos(np.deg2rad(np.arange(start_idx,stop_idx)))
        scan_y = ranges[start_idx:stop_idx] * np.sin(np.deg2rad(np.arange(start_idx,stop_idx)))
        valid_indices = np.logical_and(scan_x < 0.3, scan_y < 0.3)
        points2fit = np.dstack((scan_x[valid_indices], scan_y[valid_indices]))[0]

        ransacLineParams = [] # contains list of pair of points that identify each line
        while(True):
            line, B, C, inliers, outliers = ransac(
                points2fit, maxIter, threshold, nInliers)
            if line is None:
                break

            angle = np.rad2deg(np.arctan(C[1]-B[1])/(B[0]-C[0]))
            angle = angle%360
            if angle > 180: angle -= 360
            elif angle < -180: angle += 360

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
            self.get_logger().debug(f'Decreased inliers to: {decreasedInliers}')
            self.fitRansacLines(decreasedInliers)

        return ransacLineParams


    def angle_right(self):
        '''Return the angle (in degrees) relative to the right side wall most parallel to the robot. If no wallS detected to the right, return False'''
        if len(self.ransacLineParams) > 0:
            angles = list(map(lambda p: p["angle"], self.ransacLineParams)) 
            smallest_angle_idx = np.argmin(np.abs(angles))
            return angles[smallest_angle_idx]
        else:
            return False # No line fitted


    def wall_scan(self, obstacle_threshold:int=5):
        '''Scans and returns a list of Bool with whether a wall is detected in the 8 cardinal and ordinal directions'''
        wall_detected_arr = np.zeros(8)

        dimensions = self.get_parameter('robot_dimensions').value
        diagonal_x_threshold = self.get_parameter('diagonal_x_threshold').value
        diagonal_y_threshold = self.get_parameter('diagonal_y_threshold').value
        vertical_threshold = self.get_parameter('vertical_threshold').value
        
        scan_x = self.last_scan * np.cos(np.deg2rad(np.arange(360)))
        scan_y = self.last_scan * np.sin(np.deg2rad(np.arange(360)))
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


    def move_forward(self, speed:float=None, turn_offset:float=0.0, align:bool=False):
        '''Uses move_2D to move forward, applying a turn component if given. If align is true, applies a y-axis twist component to maintain distance to right wall'''
        translate_velocity = self.get_parameter('translate_velocity').value
        if speed is None: speed = translate_velocity
        right_wall_buffer_distance = self.get_parameter('right_wall_buffer_distance').value

        slide=0.0
        if align:
            if right_wall_buffer_distance*3 > np.min(self.last_scan[260:280]) > right_wall_buffer_distance*1.2:
                slide = -translate_velocity/self.get_parameter('slide_correction_factor').value
            elif np.min(self.last_scan[260:280]) < right_wall_buffer_distance*0.8:
                slide = translate_velocity/self.get_parameter('slide_correction_factor').value

        self.move_2D(speed, slide, turn_offset)


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


    def stop(self):
        self.move_2D(0.0, 0.0, 0.0)


    def timer_callback(self):
        """Controller loop. Insert path planning and PID control logic here"""
        if self.last_scan is None:
            self.get_logger().info("No scan detected, pausing...")
            self.move_2D()
            return # Does not run if no scans received from Lidar
        
        if self.cooldown > 0:
            self.last_scan = None
            self.cooldown -= 1
            self.move_2D()
            return # Does not run if on cooldown

        # Use ransac to get walls and calculate angle of robot wrt right wall (if any)
        self.front_right_lines = self.fitRansacLines(self.get_parameter('ransac.start_inliers').value, 270, 360)
        self.back_right_lines = self.fitRansacLines(self.get_parameter('ransac.start_inliers').value, 180, 271)
        self.ransacLineParams = self.front_right_lines + self.back_right_lines
        angle = self.angle_right()
        self.get_logger().debug(f"Angle right: {angle}")
        wall_array = self.wall_scan()
        self.get_logger().debug(wall_array)

        # If angle wrt right wall exceeds threshold, 
        if angle is not None:
            turn_offset = -np.deg2rad(angle) * self.get_parameter('turn_correction_factor').value
            if 60 >= abs(angle):
                if abs(angle) > self.get_parameter('angle_tolerance').value:
                    self.move_2D(0,0,turn_offset)
                    return 
            else: # There shouldn't be a caase where right wall is detected but angle is above 90deg
                self.front_right_lines = self.fitRansacLines(self.get_parameter('ransac.start_inliers').value, 270, 440) # Try ransac again with larger scan radius
                self.ransacLineParams = self.front_right_lines + self.back_right_lines
                angle = self.angle_right()
                if 60 >= abs(angle):
                    if abs(angle) > self.get_parameter('angle_tolerance').value:
                        self.move_2D(0,0,turn_offset)
                        return     
                else:
                    self.move_2D(0.1) # Move slowly to get closer to wall

        ###### INSERT CODE HERE ######
        if wall_array[0]: # Wall in-front
            self.get_logger().info("wall front, turn left")
            self.turn_left_90deg()

        elif wall_array[6]: # Wall right
            self.get_logger().info("wall right, move forward")
            self.move_forward(turn_offset=turn_offset, align=True)
        elif wall_array[5]:
            self.get_logger().info('Right corner, turn right')
            self.turn_right_90deg()
        else:
            self.get_logger().info("Nothing detected, moving forward")
            self.move_forward(turn_offset=turn_offset)
        ##### INSERT CODE HERE ######
    
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

