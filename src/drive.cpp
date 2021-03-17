#include "odom.h"
#include "glm/gtc/matrix_transform.hpp"

using namespace std;
using namespace std::chrono;

// constructors
XDrive::XDrive() : XDrive(glm::vec2(0), 0) { }

XDrive::XDrive(glm::vec2 startPos) : XDrive(startPos, 0) { }

XDrive::XDrive(glm::vec2 startPos, double startOrientation) {
    this->orientation = startOrientation;
    this->position = startPos;
}

// turn on interval [-1, 1], ||drive|| <= 1
void XDrive::strafe(glm::vec2 drive, double turn) {
    glm::vec2 accel = (float)acceleration * drive;
    double angAccel = angularAcceleration*turn;

    localVelocity = localVelocity + accel;
    if(localVelocity.length() > maxSpeed) {
        localVelocity = glm::normalize(localVelocity);
    }

    glm::vec2 velocity = localToGlobal(localVelocity);

    auto t = high_resolution_clock::now();
    auto deltaT = duration_cast<duration<double>>(t - lastUpdate);
    position = position + ((float)deltaT.count() * velocity);
    lastUpdate = t;
}

glm::vec2 XDrive::localToGlobal(glm::vec2 vec) {
    return rotateVector(vec, orientation);
}

glm::mat4 XDrive::getMatrix() {
    glm::mat4 mat;
    mat = glm::scale(mat, glm::vec3(1.0f/12));
    // mat = glm::translate(mat, glm::vec3(position, 0));
    // mat = glm::rotate(mat, (float)orientation, glm::vec3(0, 0, 1));
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