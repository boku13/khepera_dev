#include <stdio.h>
#include <math.h>
#include <unistd.h>

// Constants for Khepera 4
#define WHEEL_RADIUS 0.021      // Wheel radius in meters
#define WHEEL_BASE 0.0945       // Distance between wheels in meters
#define ENCODER_RESOLUTION 3600 // Encoder resolution (ticks per revolution)

// Constants for circular motion
#define CIRCLE_RADIUS 0.5    // Desired circle radius in meters
#define TARGET_SPEED 0.1     // Desired linear speed in m/s
#define CONTROL_INTERVAL 0.1 // Control loop interval in seconds
#define NUM_ITERATIONS 100   // Number of iterations for the control loop

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
    // ... (The same function as in the previous answer)
}

// Function to set the wheel speeds (stub, replace with actual hardware control)
void set_wheel_speeds(int left_speed, int right_speed)
{
    printf("Setting left wheel speed: %d, right wheel speed: %d\n", left_speed, right_speed);
}

int main()
{
    Pose robot_pose = {0.0, 0.0, 0.0};
    int delta_left, delta_right; // Example delta ticks for left and right wheels
    double angular_speed;        // Desired angular speed

    // Calculate the desired angular speed
    angular_speed = TARGET_SPEED / CIRCLE_RADIUS;

    for (int i = 0; i < NUM_ITERATIONS; i++)
    {
        // Read the wheel encoders (stub, replace with actual encoder readings)
        delta_left = 10;  // Example delta ticks for left wheel
        delta_right = 12; // Example delta ticks for right wheel

        // Update odometry
        update_odometry(&robot_pose, delta_left, delta_right);

        // Calculate the required wheel speeds for circular motion
        double left_speed = (2 * TARGET_SPEED - WHEEL_BASE * angular_speed) / (2 * WHEEL_RADIUS);
        double right_speed = (2 * TARGET_SPEED + WHEEL_BASE * angular_speed) / (2 * WHEEL_RADIUS);

        // Convert wheel speeds to encoder ticks
        int left_speed_ticks = (int)(left_speed * ENCODER_RESOLUTION / (2 * M_PI * WHEEL_RADIUS));
        int right_speed_ticks = (int)(right_speed * ENCODER_RESOLUTION / (2 * M_PI * WHEEL_RADIUS));

        // Set wheel speeds
        set_wheel_speeds(left_speed_ticks, right_speed_ticks);

        // Sleep for the control interval
        usleep((int)(CONTROL_INTERVAL * 1e6));
    }

    printf("Final Pose: x = %.4f m, y = %.4f m, theta = %.4f rad\n", robot_pose.x, robot_pose.y, robot_pose.theta);
    return 0;
}
