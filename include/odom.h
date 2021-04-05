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
const double wheelbase = 11.5;
const double backOffset = 6;
// const double lrOffset = wheelbase/2;
// const double rlOffset = wheelbase/2;

const double angularStoppingDecel = 15 * M_PI;
const double stoppingDecel = 70;
const double acceleration = 100 + stoppingDecel;
const double angularAcceleration = 4 * M_PI;
const double maxSpeed = 48;
const double maxAngularSpeed = 4 * M_PI;

const double viscousFrictionCoeff = 0.5;
const double viscousAngularCoeff = 0.5;

const double maxForce = acceleration / 2.828;

// GL containers
extern GLuint driveVBO;
extern GLuint driveVAO;

double degToRad(double d);
double radToDeg(double r);

class Motor {
private:
    const double jerk = 10000; // Consider nearly infinite jerk
    const double stoppingJerk = 5000; // Slightly less stopping jerk
    glm::vec2 position;
    double orientation;
    double output;
    double lastUpdate = 0;

public:
    Motor();
    Motor(glm::vec2 p, double o);

    void setPower(double power);
    glm::vec2 getForce();
    glm::vec2 getPosition();
};

class XDrive {
private:
    glm::vec2 localVelocity;
    double angularVelocity;

    glm::vec2 position;
    double orientation;
    double lastUpdate;

    std::vector<Motor> motors = {
        Motor(glm::vec2( wheelbase / 24,  wheelbase / 24), degToRad( 45)), // front right
        Motor(glm::vec2(-wheelbase / 24,  wheelbase / 24), degToRad(-45)), // front left
        Motor(glm::vec2( wheelbase / 24, -wheelbase / 24), degToRad(-45)), // back  right
        Motor(glm::vec2(-wheelbase / 24, -wheelbase / 24), degToRad( 45))  // back  left
    };

    glm::vec2 getNetForce();
    double getNetTorque();

public:
    // Constructors
    XDrive();
    XDrive(glm::vec2 startPos);
    XDrive(glm::vec2 startPos, double startOrientation);

    // Control
    void strafe(glm::vec2 drive, double turn);
    void update();

    // Utility
    glm::vec2 localToGlobal(glm::vec2 v);
    glm::vec2 globalToLocal(glm::vec2 vec);
    glm::mat4 getMatrix();

    glm::vec2 getPosition();
    double getOrientation();
};

glm::vec2 rotateVector(glm::vec2 v, double angle);