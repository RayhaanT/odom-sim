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

// Robot metrics
const double wheelbase = 10.25;
const double backOffset = 6;
// const double lrOffset = wheelbase/2;
// const double rlOffset = wheelbase/2;

// GL containers
extern GLuint driveVBO;
extern GLuint driveVAO;

#define STARTX 36
#define STARTY 8
#define STARTO 0

class XDrive {
private:
    const double angularStoppingDecel = 24 * M_PI;
    const double stoppingDecel = 144;
    const double acceleration = 48 + stoppingDecel;
    const double angularAcceleration = 20 * M_PI;
    const double maxSpeed = 48;
    const double maxAngularSpeed = 4 * M_PI;
    glm::vec2 localVelocity;
    double angularVelocity;

    glm::vec2 position;
    double orientation;
    double lastUpdate;

public:
    // Constructors
    XDrive();
    XDrive(glm::vec2 startPos);
    XDrive(glm::vec2 startPos, double startOrientation);

    // Control
    void strafe(glm::vec2 drive, double turn);
    void strafeGlobal(glm::vec2 dir, double turn);

    // Utility
    glm::vec2 localToGlobal(glm::vec2 v);
    glm::vec2 globalToLocal(glm::vec2 vec);
    glm::mat4 getMatrix();

    glm::vec2 getPosition();
    double getOrientation();
};

glm::vec2 rotateVector(glm::vec2 v, double angle);