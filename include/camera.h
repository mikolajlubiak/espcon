#pragma once

#include <algorithm>

#include "custom_math.h"

// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
enum Camera_Movement
{
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

// Default camera values
constexpr float YAW = -90.0f;
constexpr float PITCH = 0.0f;
constexpr float SPEED = 0.01f;
constexpr float SENSITIVITY = 0.1f;
constexpr float ZOOM = 90.0f;

// An abstract camera class that processes input and calculates the corresponding Euler Angles, Vectors and Matrices for use in OpenGL
class Camera
{
public:
    // camera Attributes
    vec3 Position;
    vec3 Front;
    vec3 Up;
    vec3 Right;
    vec3 WorldUp;
    // euler Angles
    float Yaw;
    float Pitch;
    // camera options
    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;

    // constructor with vectors
    Camera(vec3 position = vec3(0.0f, 0.0f, 0.0f), vec3 up = vec3(0.0f, 1.0f, 0.0f), float yaw = YAW, float pitch = PITCH) : Front(vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY),
                                                                                                                             Zoom(ZOOM), Position(position), WorldUp(up), Yaw(yaw), Pitch(pitch)
    {
        updateCameraVectors();
    }
    // constructor with scalar values
    Camera(float posX, float posY, float posZ, float upX, float upY, float upZ, float yaw, float pitch) : Front(vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM),
                                                                                                          Position(vec3(posX, posY, posZ)), WorldUp(vec3(upX, upY, upZ)), Yaw(yaw), Pitch(pitch)
    {
        updateCameraVectors();
    }

    // returns the view matrix calculated using Euler Angles and the LookAt Matrix
    mat4 GetViewMatrix()
    {
        return point_at(Position, Position + Front, Up);
    }

    // processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
    void ProcessKeyboard(Camera_Movement direction, float deltaTime)
    {
        float velocity = MovementSpeed * deltaTime;
        if (direction == FORWARD)
            Position = Position + Front * velocity;
        if (direction == BACKWARD)
            Position = Position - Front * velocity;
        if (direction == LEFT)
            Position = Position - Right * velocity;
        if (direction == RIGHT)
            Position = Position + Right * velocity;

#ifdef DEBUG
        Serial.printf("%f, %f, %f\n", Position.x, Position.y, Position.z);
#endif
    }

    // processes input received from a mouse input system. Expects the offset value in both the x and y direction.
    void ProcessMouseMovement(float xoffset, float yoffset, bool constrainPitch = true)
    {
        xoffset *= MouseSensitivity;
        yoffset *= MouseSensitivity;

        Yaw += xoffset;
        Pitch += yoffset;

        // make sure that when pitch is out of bounds, screen doesn't get flipped
        if (constrainPitch)
        {
            Pitch = constrain(Pitch, -89.0f, 89.0f);
        }

        // update Front, Right and Up Vectors using the updated Euler angles
        updateCameraVectors();
    }

    // processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
    void ProcessMouseScroll(float yoffset)
    {
        Zoom -= (float)yoffset;
        if (Zoom < 1.0f)
            Zoom = 1.0f;
        if (Zoom > 45.0f)
            Zoom = 45.0f;
    }

private:
    // calculates the front vector from the Camera's (updated) Euler Angles
    void updateCameraVectors()
    {
        // calculate the new Front vector
        vec3 front;
        front.x = cos(deg_to_rad(Yaw)) * cos(deg_to_rad(Pitch));
        front.y = sin(deg_to_rad(Pitch));
        front.z = sin(deg_to_rad(Yaw)) * cos(deg_to_rad(Pitch));
        Front = normalize(front);
        // also re-calculate the Right and Up vector
        Right = normalize(cross(Front, WorldUp)); // normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
        Up = normalize(cross(Right, Front));
    }
};
