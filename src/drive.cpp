#include "odom.h"
#include "tracking.h"
#include "glm/gtc/matrix_transform.hpp"
#include <iostream>

Vector2 glmToCustom(glm::vec2 v) {
    return Vector2(v.x, v.y);
}

glm::vec2 customToGLM(Vector2 v) {
    return glm::vec2(v.getX(), v.getY());
}

// constructors
XDrive::XDrive() : XDrive(glm::vec2(0), 0) { }

XDrive::XDrive(glm::vec2 startPos) : XDrive(startPos, 0) { }

XDrive::XDrive(glm::vec2 startPos, double startOrientation) {
    this->orientation = startOrientation;
    this->position = startPos;
}

void XDrive::strafeGlobal(glm::vec2 dir, double turn) {
    Vector2 cDir = toLocalCoordinates(glmToCustom(dir));
	double xVel = cDir.getX();
	double yVel = cDir.getY();

    strafe(glm::vec2(xVel, yVel), turn);
}

// turn on interval [-1, 1], ||drive|| <= 1
void XDrive::strafe(glm::vec2 drive, double turn) {
    glm::vec2 accel = (float)acceleration * drive;
    double angAccel = angularAcceleration*turn;

    auto t = glfwGetTime();
    auto deltaT = t - lastUpdate;

    localVelocity = localVelocity + (accel * (float)deltaT);
    if(glm::length(localVelocity) > maxSpeed) {
        localVelocity = glm::normalize(localVelocity) * (float)maxSpeed;
    }
    angularVelocity += angAccel * deltaT;
    if(abs(angularVelocity) > maxAngularSpeed) {
        angularVelocity = angularVelocity / abs(angularVelocity) * maxAngularSpeed;
    }

    if(turn == 0) {
        if(angularVelocity > 0) {
            angularVelocity -= std::min(angularStoppingDecel * deltaT, abs(angularVelocity));
        }
        else {
            angularVelocity += std::min(angularStoppingDecel * deltaT, abs(angularVelocity));
        }
    }
    if(localVelocity.x > 0) {
        localVelocity.x -= std::min((float)(stoppingDecel * deltaT), abs(localVelocity.x));
    }
    else {
        localVelocity.x += std::min((float)(stoppingDecel * deltaT), abs(localVelocity.x));
    }
    if(localVelocity.y > 0) {
        localVelocity.y -= std::min((float)(stoppingDecel * deltaT), abs(localVelocity.y));
    }
    else {
        localVelocity.y += std::min((float)(stoppingDecel * deltaT), abs(localVelocity.y));
    }

    glm::vec2 velocity = localToGlobal(localVelocity);

    // std::cout << angularVelocity << std::endl;
    // std::cout << velocity.x << " " << velocity.y << std::endl;
    glm::vec2 oldPosition = position;
    position = position + ((float)deltaT * velocity);
    orientation += angularVelocity * deltaT;
    orientation = fmod(orientation, 2 * M_PI);
    lastUpdate = t;

    Vector2 dP = glmToCustom(globalToLocal(position - oldPosition));
    double dO = angularVelocity * deltaT;

    leftTrackingWheel.update(dP, dO);
    rightTrackingWheel.update(dP, dO);
    backTrackingWheel.update(dP, dO);
}

glm::vec2 XDrive::localToGlobal(glm::vec2 vec) {
    return rotateVector(vec, orientation);
}

glm::vec2 XDrive::globalToLocal(glm::vec2 vec) {
    return rotateVector(vec, -orientation);
}

glm::mat4 XDrive::getMatrix() {
    glm::mat4 mat;
    mat = glm::scale(mat, glm::vec3(2.0f/144));
    mat = glm::translate(mat, glm::vec3(position, 0) + glm::vec3(-144/2, -144/2, 0));
    mat = glm::rotate(mat, (float)orientation, glm::vec3(0, 0, 1));
    return mat;
}

glm::vec2 rotateVector(glm::vec2 vec, double angle)
{
    // x = cos(a), y = sin(a)
    // cos(a + b) = cos(a)cos(b) - sin(a)sin(b)
    double newX = (vec.x * cos(angle)) - (vec.y * sin(angle));

    // sin(a + b) = sin(a)cos(b) + cos(a)sin(b)
    double newY = (vec.y * cos(angle)) + (vec.x * sin(angle));

    return glm::vec2(newX, newY);
}

glm::vec2 XDrive::getPosition() {
    return this->position;
}

double XDrive::getOrientation() {
    return this->orientation;
}