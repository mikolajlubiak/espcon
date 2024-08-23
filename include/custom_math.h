#pragma once

#include <cmath>
#include <cstring>

struct vec3;

struct mat4
{
    float m[4][4];

    void zero()
    {
        memset(&m, 0, sizeof(float) * 16);
    }

    mat4()
    {
        zero();
    }

    mat4(const float n)
    {
        zero();
        m[0][0] = n;
        m[1][1] = n;
        m[2][2] = n;
        m[3][3] = n;
    }
};

struct vec4
{
    float x, y, z, w;

    vec4() : x(0.0f), y(0.0f), z(0.0f), w(1.0f) {}

    vec4(const float x, const float y, const float z, const float w) : x(x), y(y), z(z), w(w) {}

    vec4(const float x, const float y, const float z) : x(x), y(y), z(z), w(1.0f) {}

    vec4(const vec3 &vec, const float w);

    vec4(const vec3 &vec);
};

struct vec3
{
    float x, y, z;

    vec3() : x(0.0f), y(0.0f), z(0.0f) {}

    vec3(const float x, const float y, const float z) : x(x), y(y), z(z) {}

    vec3(const vec4 &vec) : x(vec.x), y(vec.y), z(vec.z) {}

    const float length() const;
};

vec3 normalize(const vec3 &vec);

float dot(const vec3 &lhs, const vec3 &rhs);

vec3 cross(const vec3 &lhs, const vec3 &rhs);

mat4 point_at(const vec3 &pos, const vec3 &target, const vec3 &up);

mat4 perspective(const float fFov, const float fAspectRatio, const float fNear,
                 const float fFar);

vec3 operator-(const vec3 &lhs, const vec3 &rhs);

vec3 operator+(const vec3 &lhs, const vec3 &rhs);

vec4 operator+(const vec4 &lhs, const vec3 &rhs);

vec3 operator/(const vec3 &vec, const float k);

vec4 operator/(const vec4 &vec, const float k);

vec3 operator*(const vec3 &vec, const float k);

vec4 operator*(const vec4 &lhs, const mat4 &rhs);

float deg_to_rad(const float deg);

float rad_to_deg(const float rad);

float map_floats(float x, float in_min, float in_max, float out_min, float out_max);