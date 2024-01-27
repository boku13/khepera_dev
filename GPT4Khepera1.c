#include <stdio.h>
#include <math.h>

// Constants for Khepera 4
#define WHEEL_RADIUS 0.021      // Wheel radius in meters
#define WHEEL_BASE 0.0945       // Distance between wheels in meters
#define ENCODER_RESOLUTION 3600 // Encoder resolution (ticks per revolution)

// Structure to hold robot's position and orientation
typedef struct
{
    double x;
    double y;
    double theta;
} Pose;

// Function to update odometry using wheel encoders
void update_odometry(Pose *pose, int delta_left, int delta_right)
{
    double delta_theta, delta_s, delta_x, delta_y;

    // Calculate the distance traveled by each wheel
    double delta_left_meters = (2 * M_PI * WHEEL_RADIUS * delta_left) / ENCODER_RESOLUTION;
    double delta_right_meters = (2 * M_PI * WHEEL_RADIUS * delta_right) / ENCODER_RESOLUTION;

    // Calculate the change in orientation and distance
    delta_theta = (delta_right_meters - delta_left_meters) / WHEEL_BASE;
    delta_s = (delta_right_meters + delta_left_meters) / 2.0;

    // Calculate the change in x and y position
    delta_x = delta_s * cos(pose->theta + delta_theta / 2.0);
    delta_y = delta_s * sin(pose->theta + delta_theta / 2.0);

    // Update the pose
    pose->x += delta_x;
    pose->y += delta_y;
    pose->theta += delta_theta;
}

int main()
{
    Pose robot_pose = {0.0, 0.0, 0.0};
    int delta_left = 100;  // Example delta ticks for left wheel
    int delta_right = 120; // Example delta ticks for right wheel

    update_odometry(&robot_pose, delta_left, delta_right);

    printf("Updated Pose: x = %.4f m, y = %.4f m, theta = %.4f rad\n", robot_pose.x, robot_pose.y, robot_pose.theta);
    return 0;
}
