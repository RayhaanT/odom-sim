#pragma once
#include "Eigen/Dense"
#include <cmath>
#include <glm/glm.hpp>
#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <vector>
#include <chrono>
#include "math.h"
#define _USE_MATH_DEFINES

// Sensor data
extern double leftTrackingWheel;
extern double rightTrackingWheel;
extern double backTrackingWheel;

// Robot metrics
const double wheelbase = 11.5;
const double backOffset = 6;
const double lrOffset = wheelbase/2;
const double rlOffset = wheelbase/2;

// GL containers
extern GLuint driveVBO;
extern GLuint driveVAO;

class XDrive {
private:
    const double acceleration = 1;
    const double angularAcceleration = M_PI / 2;
    const double maxSpeed = 4;
    const double maxAngularSpeed = 2 * M_PI;
    glm::vec2 localVelocity;

    glm::vec2 position;
    double orientation;
    std::chrono::high_resolution_clock::time_point lastUpdate;

public:
    // Constructors
    XDrive();
    XDrive(glm::vec2 startPos);
    XDrive(glm::vec2 startPos, double startOrientation);

    // Control
    void strafe(glm::vec2 drive, double turn);

    // Utility
    glm::vec2 localToGlobal(glm::vec2 v);
    glm::mat4 getMatrix();
};

glm::vec2 rotateVector(glm::vec2 v, double angle);