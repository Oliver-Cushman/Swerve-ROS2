#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <cmath>
#include "math/GeometryUtil.h"

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

        static inline const std::array<geometry_util::Translation2d, 4> SWERVE_MOD_TRANSLATIONS = {
            geometry_util::Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), 
            geometry_util::Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            geometry_util::Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            geometry_util::Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
        };
    };

    struct Input
    {
        static constexpr int MAX_ATTEMPTS = 10;
        static constexpr float GAMEPAD_AXIS_MAX = 32768.0;
    };

};

#endif