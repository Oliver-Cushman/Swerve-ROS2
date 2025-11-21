#ifndef constants
#define constants
#include <cmath>

namespace constants
{

    struct Swerve
    {
        static constexpr float MAX_LINEAR_VELOCITY = 4.5f;
        static constexpr float MAX_ANGULAR_VELOCITY = 2 * M_PI;
        static constexpr float WHEEL_RADIUS = 0.0508f;
        static constexpr float WHEEL_BASE = 0.5842f;
        // same as WHEEL_BASE for square swerve
        static constexpr float TRACK_WIDTH = WHEEL_BASE;
    };

    struct Input
    {
        static constexpr int MAX_ATTEMPTS = 10;
        static constexpr float GAMEPAD_AXIS_MAX = 32768.0;
    };

};

#endif