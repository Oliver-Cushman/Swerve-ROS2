#ifndef geometry_util
#define geometry_util
#include <cmath>

namespace geometry_util
{
    struct Translation2d
    {
        const float x;
        const float y;

        Translation2d(float _x, float _y) : x(_x), y(_y) {}
    };

    struct Rotation2d
    {
        const float angle;

        Rotation2d(float _angle) : angle(_angle) {}

        float get_radians() const
        {
            return angle;
        }

        float get_degrees() const
        {
            return angle * 180.0 / M_PI;
        }

        float get_sin() const
        {
            return std::sin(angle);
        }

        float get_cos() const
        {
            return std::cos(angle);
        }

        static Rotation2d atan(const Translation2d translation)
        {
            return std::atan2(translation.y, translation.x);
        }
    };

    struct Pose2d
    {
        const Translation2d translation;
        const Rotation2d rotation;

        Pose2d(Translation2d _translation, Rotation2d _rotation) : translation(_translation), rotation(_rotation) {}
    };

    struct Vector2d
    {
        const float x;
        const float y;

        Vector2d(float _x, float _y) : x(_x), y(_y) {}

        Vector2d(Translation2d _translation) : x(_translation.x), y(_translation.y) {}

        float magnitude() const
        {
            return std::hypot(x, y);
        }

        Vector2d operator+(const Vector2d v) const
        {
            return Vector2d(x + v.x, y + v.y);
        }

        Vector2d operator-(const Vector2d v) const
        {
            return Vector2d(x - v.x, y - v.y);
        }

        Vector2d operator*(const float scalar) const
        {
            return Vector2d(x * scalar, y * scalar);
        }

        Vector2d operator/(const float scalar) const
        {
            return Vector2d(x / scalar, y / scalar);
        }

        // Dot product
        float operator*(const Vector2d v) const
        {
            return x * v.x + y * v.y;
        }
    };

    struct Vector3d
    {
        const float x;
        const float y;
        const float z;
    };

};

#endif