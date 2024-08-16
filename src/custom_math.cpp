#include "custom_math.h"

vec4::vec4(const vec3 &vec, const float w) : x(vec.x), y(vec.y), z(vec.z), w(w) {}

const float vec3::length() const { return sqrt(dot(*this, *this)); }

vec3 normalize(const vec3 &vec)
{
    return vec / vec.length();
}

float dot(const vec3 &lhs, const vec3 &rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

vec3 cross(const vec3 &lhs, const vec3 &rhs)
{
    return {
        (lhs.y * rhs.z - lhs.z * rhs.y),
        (lhs.z * rhs.x - lhs.x * rhs.z),
        (lhs.x * rhs.y - lhs.y * rhs.x),
    };
}

mat4 point_at(const vec3 &pos, const vec3 &target, const vec3 &up)
{
    vec3 Target, Up;

    Target = target - pos;
    Target = normalize(Target);

    vec3 a = Target * dot(up, Target);

    Up = up - a;
    Up = normalize(Up);

    vec3 newRight = cross(Up, Target);

    mat4 matrix;
    matrix.m[0][0] = newRight.x;
    matrix.m[0][1] = newRight.y;
    matrix.m[0][2] = newRight.z;
    matrix.m[0][3] = 0.0f;
    matrix.m[1][0] = Up.x;
    matrix.m[1][1] = Up.y;
    matrix.m[1][2] = Up.z;
    matrix.m[1][3] = 0.0f;
    matrix.m[2][0] = Target.x;
    matrix.m[2][1] = Target.y;
    matrix.m[2][2] = Target.z;
    matrix.m[2][3] = 0.0f;
    matrix.m[3][0] = pos.x;
    matrix.m[3][1] = pos.y;
    matrix.m[3][2] = pos.z;
    matrix.m[3][3] = 1.0f;
    return matrix;
}

mat4 perspective(const float fFov, const float fAspectRatio, const float fNear,
                 const float fFar)
{
    mat4 mat;
    mat.m[0][0] = fAspectRatio * fFov;
    mat.m[1][1] = fFov;
    mat.m[2][2] = fFar / (fFar - fNear);
    mat.m[3][2] = (-fFar * fNear) / (fFar - fNear);
    mat.m[2][3] = 1.0f;
    mat.m[3][3] = 0.0f;
    return mat;
}

vec3 operator-(const vec3 &lhs, const vec3 &rhs)
{
    return {
        lhs.x - rhs.x,
        lhs.y - rhs.y,
        lhs.z - rhs.z,
    };
}

vec3 operator+(const vec3 &lhs, const vec3 &rhs)
{
    return {
        lhs.x + rhs.x,
        lhs.y + rhs.y,
        lhs.z + rhs.z,
    };
}

vec4 operator+(const vec4 &lhs, const vec3 &rhs)
{
    return {
        lhs.x + rhs.x,
        lhs.y + rhs.y,
        lhs.z + rhs.z,
        lhs.w,
    };
}

vec3 operator/(const vec3 &vec, const float k)
{
    return {
        vec.x / k,
        vec.y / k,
        vec.z / k,
    };
}

vec4 operator/(const vec4 &vec, const float k)
{
    return {
        vec.x / k,
        vec.y / k,
        vec.z / k,
        vec.w,
    };
}

vec3 operator*(const vec3 &vec, const float k)
{
    return {
        vec.x * k,
        vec.y * k,
        vec.z * k,
    };
}

vec4 operator*(const vec4 &lhs, const mat4 &rhs)
{
    vec4 v;
    v.x = lhs.x * rhs.m[0][0] + lhs.y * rhs.m[1][0] + lhs.z * rhs.m[2][0] +
          lhs.w * rhs.m[3][0];
    v.y = lhs.x * rhs.m[0][1] + lhs.y * rhs.m[1][1] + lhs.z * rhs.m[2][1] +
          lhs.w * rhs.m[3][1];
    v.z = lhs.x * rhs.m[0][2] + lhs.y * rhs.m[1][2] + lhs.z * rhs.m[2][2] +
          lhs.w * rhs.m[3][2];
    v.w = lhs.x * rhs.m[0][3] + lhs.y * rhs.m[1][3] + lhs.z * rhs.m[2][3] +
          lhs.w * rhs.m[3][3];
    return v;
}

float deg_to_rad(const float deg)
{
    return deg * M_PI / 180.0;
}

float rad_to_deg(const float rad)
{
    return rad * 180.0 / M_PI;
}
