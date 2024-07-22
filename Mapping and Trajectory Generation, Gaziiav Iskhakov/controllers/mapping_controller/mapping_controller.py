# Importing Libraries
from controller import Robot, Supervisor
from matplotlib import pyplot as plt
from scipy import signal
import numpy as np

# Initialize Constants and Variables
simulation_step_counter = 0  # Simulation step counter
simulation_time = 0  # Simulation time
MAX_ROBOT_SPEED = 6.28  # Maximum speed of the robot

is_convolution_space = False
is_returning = False  # True if the robot is on the return route, False if on the outward route
probabilistic_map = np.zeros((275, 350))  # Probabilistic map for rangefinder data (275 x 350, same as display dimensions)
lidar_angles = np.linspace((2 / 3) * 3.1415, -3.1415 * (2 / 3), 667)  # Angles array for the laser rangefinder Hokuyo URG-04LX-UG01
waypoint_index = 1  # Navigation Waypoint index (starts from 1, index 0 is the robot's starting point (0,0))
waypoints = [(0.0, 0.0), (0.55, -0.5), (0.55, -1.88), (0.5, -2.80), (-0.65, -3.00),
             (-1.75, -2.80), (-1.75, -1.49), (-1.75, -0.49), (-1.30, 0.30), (0.0, 0.55)]  # Waypoints coordinates list
display_x = 0  # x pixel of the robot display
display_y = 0  # y pixel of the robot display
convolution_kernel = np.ones((33, 33))  # Kernel for convolution operations in probabilistic map

# Create Robot Instance
robot = Supervisor()  # Create robot instance
time_step = int(robot.getBasicTimeStep())  # Get the time step of the current world
print("Time Step =", time_step)  # Print the world time step for reference

# Device Initialization
left_motor = robot.getDevice('wheel_left_joint')  # Get left wheel motor device
right_motor = robot.getDevice('wheel_right_joint')  # Get right wheel motor device
display = robot.getDevice('display')  # Get display device
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')  # Get laser rangefinder device
gps = robot.getDevice('gps')  # Get GPS device
compass = robot.getDevice('compass')  # Get compass device

left_motor.setPosition(float('inf'))  # Set left motor position to infinity for speed control
right_motor.setPosition(float('inf'))  # Set right motor position to infinity for speed control
left_motor.setVelocity(MAX_ROBOT_SPEED)  # Set the left motor speed to MAX_ROBOT_SPEED
right_motor.setVelocity(MAX_ROBOT_SPEED)  # Set the right motor speed to MAX_ROBOT_SPEED

lidar.enable(time_step)  # Enable the laser rangefinder
lidar.enablePointCloud()  # Enable the PointCloud for the rangefinder (debug feature)
gps.enable(time_step)  # Enable the GPS
compass.enable(time_step)  # Enable the compass

robot_x_world = gps.getValues()[0]  # Get initial x coordinate of the robot from GPS
robot_y_world = gps.getValues()[1]  # Get initial y coordinate of the robot from GPS
robot_orientation = np.arctan2(compass.getValues()[0], compass.getValues()[1])  # Get initial orientation from compass

marker = robot.getFromDef("marker").getField("translation")  # Get control of the "marker" object position (supervisor functionality)

# Functions
def calculate_distance_and_angle(robot_x, robot_y, target_x, target_y, orientation):
    """
    Calculate the distance and orientation difference between the robot and a target point.
    """
    distance = np.sqrt((robot_x - target_x) ** 2 + (robot_y - target_y) ** 2)  # Calculate distance
    angle_difference = np.arctan2(target_y - robot_y, target_x - robot_x) - orientation  # Calculate orientation difference
    if angle_difference > np.pi:  # Adjust to avoid 180-degree flip due to arctan limits
        angle_difference -= 2 * np.pi
    elif angle_difference < -np.pi:
        angle_difference += 2 * np.pi
    return distance, angle_difference

# Main Control Loop
while robot.step(time_step) != -1:
    simulation_step_counter += 1  # Increment step counter

    # Get GPS & Compass Data
    robot_x_world = gps.getValues()[0]  # Get current x coordinate from GPS
    robot_y_world = gps.getValues()[1]  # Get current y coordinate from GPS
    robot_orientation = np.arctan2(compass.getValues()[0], compass.getValues()[1])  # Get current orientation from compass

    # Calculate Position and Direction Errors
    distance_to_waypoint, angle_to_waypoint = calculate_distance_and_angle(robot_x_world, robot_y_world, waypoints[waypoint_index][0], waypoints[waypoint_index][1], robot_orientation)  # Calculate distance and orientation difference to waypoint
    print("Distance to marker =", distance_to_waypoint, "  Angle to marker =", round((angle_to_waypoint / 3.1415) * 180, 2))  # Print distance and orientation difference

    # Update Waypoint Index
    # Update waypoint index based on proximity and route direction
    if distance_to_waypoint < 0.39 and not is_returning:
        waypoint_index += 1
    elif distance_to_waypoint < 0.39 and is_returning:
        waypoint_index -= 1

    # Update marker position according to waypoints list and index
    marker.setSFVec3f([waypoints[waypoint_index][0], waypoints[waypoint_index][1], 0.0])

    # Define parameters for wheel speed adjustment
    angle_adjustment_param, distance_adjustment_param = 8, 4  # Parameters for wheel speed adjustment
    if abs(angle_to_waypoint) > 0.55:  # Pivot if orientation difference is large
        left_wheel_speed = -angle_to_waypoint
        right_wheel_speed = angle_to_waypoint
    else:  # Drive straight with smoother motion
        left_wheel_speed = -angle_to_waypoint * angle_adjustment_param + distance_to_waypoint * distance_adjustment_param
        right_wheel_speed = angle_to_waypoint * angle_adjustment_param + distance_to_waypoint * distance_adjustment_param
    left_wheel_speed = max(min(left_wheel_speed, 9.99), -9.99)  # Cap speed to within limits
    right_wheel_speed = max(min(right_wheel_speed, 9.99), -9.99)

    # Coordinate Transformation
    world_to_robot_transform = np.array([[np.cos(robot_orientation), -np.sin(robot_orientation), robot_x_world],
                                         [np.sin(robot_orientation), np.cos(robot_orientation), robot_y_world],
                                         [0, 0, 1]])  # Transformation matrix from world to robot coordinates
    lidar_ranges = np.array(lidar.getRangeImage())  # Get data from lidar
    lidar_ranges[lidar_ranges == np.inf] = 100  # Replace infinite values with a large number
    for i in range(80):  # Exclude the first and last 80 values due to robot geometry
        lidar_ranges[i] = 100
        lidar_ranges[666 - i] = 100

    # Transform lidar data from polar to robot coordinates
    robot_coordinates = np.array([lidar_ranges * np.cos(lidar_angles) + 0.202,
                                  lidar_ranges * np.sin(lidar_angles),
                                  np.ones(len(lidar_angles))])
    world_coordinates = world_to_robot_transform @ robot_coordinates  # Convert to world coordinates

    # Probabilistic Mapping
    if simulation_step_counter < 2:  # At the first timestep, initialize coordinates to avoid NaN values
        world_coordinates[0][:] = 0
        world_coordinates[1][:] = 0
        world_coordinates[2][:] = 1
    else:  # Convert world coordinates to display coordinates
        for i in range(667):  # For each lidar point
            display_x = int((world_coordinates[0][i] + 2.75 + 0.25) * 50)  # Convert x world coordinate to display coordinate
            display_y = int(((-world_coordinates[1][i]) + 1.92 + 0.25) * 50)  # Convert y world coordinate to display coordinate

            display_x = min(max(display_x, 0), 274)  # Cap display x coordinate within bounds
            display_y = min(max(display_y, 0), 349)  # Cap display y coordinate within bounds

            probabilistic_map[display_x][display_y] += 0.01  # Increment map value probabilistically
            probabilistic_map[display_x][display_y] = min(probabilistic_map[display_x][display_y], 0.99)  # Cap the max value to 0.99

    # Display Drawing
    for i in range(275):  # Draw the entire map to the display
        for s in range(350):
            intensity = int(probabilistic_map[i, s] * 255)  # Compute intensity for grayscale color
            if intensity > 2:  # Draw pixel if intensity is greater than 2
                color = (intensity * 256**2 + intensity * 256 + intensity)  # Compute grayscale color
                display.setColor(color)
                display.drawPixel(i, s)

    # Clean map edges
    probabilistic_map[:274, 349] = 0
    probabilistic_map[274, :349] = 0
    probabilistic_map[274, 349] = 0

    # Trajectory Tracing
    pX = int((robot_x_world + 2.75 + 0.25) * 50)  # Convert x world coordinate to display coordinate
    pY = int(((-robot_y_world) + 1.92 + 0.25) * 50)  # Convert y world coordinate to display coordinate
    display.setColor(0xFF0000)  # Set color to red
    display.drawPixel(pX, pY)  # Draw the pixel

    left_motor.setVelocity(left_wheel_speed)
    right_motor.setVelocity(right_wheel_speed)

    if waypoint_index > 8:
        is_returning = True

    if waypoint_index == -1:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        print("STOP")
        cmap = signal.convolve2d(probabilistic_map, convolution_kernel, mode='same')
        cspace = cmap > 0.9
        plt.imshow(cspace)
        plt.pause(0.01)
        plt.show()
    else:
        print("Waypoint Number =", waypoint_index)
