#include <KoreBot/KoreBot.h>
#include <math.h>

#define TICKS_PER_REVOLUTION 3600 // number of ticks per wheel revolution
#define WHEEL_RADIUS 20           // radius of wheel in millimeters
#define WHEEL_DISTANCE 51         // distance between wheels in millimeters

int main(int argc, char *argv[])
{

    // Initialize KoreBot
    if (kb_init(argc, argv) < 0)
    {
        printf("Could not initialize KoreBot.\n");
        return -1;
    }

    // Get wheel encoder objects
    kb_dev_t *left_encoder = kb_lwheel_encoder_get(0);
    kb_dev_t *right_encoder = kb_rwheel_encoder_get(0);

    // Initialize encoder values
    long left_ticks_prev = 0, right_ticks_prev = 0;
    long left_ticks = 0, right_ticks = 0;
    double left_distance = 0, right_distance = 0;
    double x = 0, y = 0, theta = 0;

    // Loop to update odometry
    while (1)
    {

        // Read encoder values
        kb_lwheel_encoder_read(left_encoder, &left_ticks);
        kb_rwheel_encoder_read(right_encoder, &right_ticks);

        // Calculate distance traveled by each wheel
        left_distance = (left_ticks - left_ticks_prev) * 2 * M_PI * WHEEL_RADIUS / TICKS_PER_REVOLUTION;
        right_distance = (right_ticks - right_ticks_prev) * 2 * M_PI * WHEEL_RADIUS / TICKS_PER_REVOLUTION;

        // Update previous tick counts
        left_ticks_prev = left_ticks;
        right_ticks_prev = right_ticks;

        // Calculate distance traveled by robot
        double distance = (left_distance + right_distance) / 2;

        // Calculate change in angle
        double dtheta = (right_distance - left_distance) / WHEEL_DISTANCE;

        // Update x, y, and theta
        x += distance * cos(theta + dtheta / 2);
        y += distance * sin(theta + dtheta / 2);
        theta += dtheta;

        // Print current position and orientation
        printf("x: %f, y: %f, theta: %f\n", x, y, theta);
    }

    // Close encoder objects and KoreBot
    kb_dev_release(left_encoder);
    kb_dev_release(right_encoder);
    kb_cleanup();

    return 0;
}
