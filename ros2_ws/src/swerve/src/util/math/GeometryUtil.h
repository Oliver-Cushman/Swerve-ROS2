#ifndef GEOMETRY_UTIL_H
#define GEOMETRY_UTIL_H
#include <cmath>

namespace geometry_util
{
    struct Translation2d
    {
        float x;
        float y;

        constexpr Translation2d(float _x, float _y) : x(_x), y(_y) {}
    };

    struct Rotation2d
    {
        float angle;

        constexpr Rotation2d(float _angle) : angle(_angle) {}
        constexpr Rotation2d(float y, float x) : angle(std::atan2(y, x)) {}

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
    };

    struct Pose2d
    {
        const Translation2d translation;
        const Rotation2d rotation;

        constexpr Pose2d(Translation2d _translation, Rotation2d _rotation) : translation(_translation), rotation(_rotation) {}
    };

    struct Vector2d
    {
        float x;
        float y;

        constexpr Vector2d(float _x, float _y) : x(_x), y(_y) {}

        float magnitude() const
        {
            return std::hypot(x, y);
        }

        Vector2d operator+(const Vector2d &v) const
        {
            return Vector2d(x + v.x, y + v.y);
        }

        Vector2d operator-(const Vector2d &v) const
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
        float operator*(const Vector2d &v) const
        {
            return x * v.x + y * v.y;
        }

        Rotation2d angle_difference(const Vector2d &v) const
        {
            return Rotation2d(std::acos(*this * v / (this->magnitude() * v.magnitude())));
        }

        Rotation2d angle() const
        {
            return Rotation2d(y, x);
        }
    };

    struct Vector3d
    {
        float x;
        float y;
        float z;

        constexpr Vector3d(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

        float magnitude() const
        {
            return std::hypot(x, y, z);
        }

        Vector3d operator+(const Vector3d &v) const
        {
            return Vector3d(x + v.x, y + v.y, z + v.z);
        }

        Vector3d operator-(const Vector3d &v) const
        {
            return Vector3d(x - v.x, y - v.y, z - v.z);
        }

        Vector3d operator*(const float scalar) const
        {
            return Vector3d(x * scalar, y * scalar, z * scalar);
        }

        Vector3d operator/(const float scalar) const
        {
            return Vector3d(x / scalar, y / scalar, z / scalar);
        }

        // Dot product
        float operator*(const Vector3d &v) const
        {
            return x * v.x + y * v.y + z * v.z;
        }

        // Cross product
        Vector3d cross(const Vector3d &v) const
        {
            return Vector3d(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
        }

        Rotation2d angle_difference(const Vector3d &v) const
        {
            return Rotation2d(std::acos(*this * v / (this->magnitude() * v.magnitude())));
        }
    };

};

#endif