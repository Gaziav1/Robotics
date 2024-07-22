#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <math.h>

// Constants
#define TIME_STEP 64
#define MAX_SPEED 6.28
#define NORMAL_SPEED (MAX_SPEED * 0.5)
#define TURN_SPEED (MAX_SPEED * 0.2)
#define WHEEL_RADIUS 0.0201
#define WHEEL_BASE 0.052

// Global variables
double total_distance = 0.0;
double orientation = 0.0;
double xw = 0.0;
double yw = 0.0;
const char *last_black_sensor = NULL;

// Function to calculate wheel speeds in radians per second
void calculate_wheel_speeds(double left_speed, double right_speed, double *left_rad_per_sec, double *right_rad_per_sec) {
  *left_rad_per_sec = (left_speed / MAX_SPEED) * MAX_SPEED;
  *right_rad_per_sec = (right_speed / MAX_SPEED) * MAX_SPEED;
}

int main(int argc, char **argv) {
  wb_robot_init();

  // Initialize motors
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Initialize ground sensors
  WbDeviceTag ground_sensors[3];
  for (int i = 0; i < 3; i++) {
    char sensor_name[4];
    sprintf(sensor_name, "gs%d", i);
    ground_sensors[i] = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(ground_sensors[i], TIME_STEP);
  }

  while (wb_robot_step(TIME_STEP) != -1) {
    // Read sensor values
    double sensor_values[3];
    for (int i = 0; i < 3; i++) {
      sensor_values[i] = wb_distance_sensor_get_value(ground_sensors[i]);
    }

    // Print sensor values for debugging
    printf("%f %f %f\n", sensor_values[0], sensor_values[1], sensor_values[2]);

    // Initialize motor speeds
    double left_speed = NORMAL_SPEED;
    double right_speed = NORMAL_SPEED;

    if (sensor_values[0] < 500 && sensor_values[2] > 500) {
      last_black_sensor = "left";
    } else if (sensor_values[0] > 500 && sensor_values[2] < 500) {
      last_black_sensor = "right";
    }

    // Adjust motor speeds based on sensor values to follow the black line
    if (sensor_values[1] < 500) {
      left_speed = NORMAL_SPEED;
      right_speed = NORMAL_SPEED;
    } else if (sensor_values[0] < 500) {
      left_speed = TURN_SPEED;
      right_speed = NORMAL_SPEED;
    } else if (sensor_values[2] < 500) {
      left_speed = NORMAL_SPEED;
      right_speed = TURN_SPEED;
    } else {
      if (last_black_sensor == "left") {
        left_speed = -TURN_SPEED;
        right_speed = TURN_SPEED;
      } else if (last_black_sensor == "right") {
        left_speed = TURN_SPEED;
        right_speed = -TURN_SPEED;
      } else {
        left_speed = TURN_SPEED;
        right_speed = TURN_SPEED;
      }
    }

    // Calculate odometry
    double left_rad_per_sec, right_rad_per_sec;
    calculate_wheel_speeds(left_speed, right_speed, &left_rad_per_sec, &right_rad_per_sec);
    double delta_x = (left_rad_per_sec + right_rad_per_sec) / 2 * WHEEL_RADIUS * TIME_STEP / 1000;
    double delta_orientation = (right_rad_per_sec - left_rad_per_sec) / WHEEL_BASE * TIME_STEP / 1000;

    // Update position and orientation
    total_distance += delta_x;
    orientation += delta_orientation;
    xw += delta_x * cos(orientation);
    yw += delta_x * sin(orientation);

    // Set the motor speeds
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);

    // Print odometry values for debugging
    printf("total_distance: %f\n", total_distance);
    printf("abs(xw): %f\n", fabs(xw));
    printf("abs(yw): %f\n", fabs(yw));

    // Check if robot is back to starting position within a threshold after minimum conditions are met
    if (total_distance >= 3.15) {
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
      break;
    }
  }

  // Final orientation in degrees
  double final_orientation = fmod((orientation / 3.14159) * 180, 360);
  printf("--------------------\n");
  printf("Total Distance: %.3f meters\n", total_distance);
  printf("Final Orientation: %.1f degrees\n", final_orientation);
  printf("Final Position: xw = %.2f, yw = %.2f\n", xw, yw);

  // Print the final error as Euclidean distance from (0,0)
  double error = sqrt(xw * xw + yw * yw);
  printf("Total Error from Start: %.3f meters\n", error);
  printf("Total Error from Start: %.1f cm\n", error * 100);

  wb_robot_cleanup();
  return 0;
}