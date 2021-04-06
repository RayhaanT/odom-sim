#include "odom.h"
#include "tracking.h"
#include "glm/gtc/matrix_transform.hpp"
#include <iostream>
#include <chrono>
#include <thread>

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
    double straight = drive.y;
    double right = drive.x;

    double scalar = 1;
    if (abs(right) + abs(straight) + abs(turn) > 1) {
        scalar = abs(right) + abs(straight) + abs(turn);
    }

    motors[0].setPower((straight - right + turn) / scalar); // front right
    motors[1].setPower((straight + right - turn) / scalar); // front left
    motors[2].setPower((straight + right + turn) / scalar); // back  right
    motors[3].setPower((straight - right - turn) / scalar); // back  left

    update();
}

glm::vec2 calculateLinearFriction(glm::vec2 localVel, float friction) {
    glm::vec2 norm = glm::normalize(localVel);

    double straight = norm.y;
    double right = norm.x;

    double scalar = 1;
    if (abs(right) + abs(straight) > 1) {
        scalar = abs(right) + abs(straight);
    }

    Vector2 fr = rotateVector(Vector2(0, (straight - right) / scalar), degToRad( 45));  // front right
    Vector2 fl = rotateVector(Vector2(0, (straight + right) / scalar), degToRad(-45));  // front left
    Vector2 br = rotateVector(Vector2(0, (straight + right) / scalar), degToRad(-45));  // back  right
    Vector2 bl = rotateVector(Vector2(0, (straight - right) / scalar), degToRad( 45));  // back  left

    Vector2 net = fr + fl + br + bl;

    glm::vec2 viscousFriction = -localVel * (float)viscousFrictionCoeff;

    return (float)(net.getMagnitude() * friction / 2) * -glm::vec2(norm.x, norm.y) + viscousFriction;
}

void XDrive::update() {
    auto t = glfwGetTime();
    auto deltaT = t - lastUpdate;

    glm::vec2 driveForce = getNetForce();

    double angAccel = getNetTorque();
    glm::vec2 oldVelocity = localVelocity;
    localVelocity = localVelocity + (driveForce * (float)deltaT);
    if(glm::length(localVelocity) > maxSpeed) {
        localVelocity = glm::normalize(localVelocity) * (float)maxSpeed;
    }
    angularVelocity += angAccel * deltaT;
    if(abs(angularVelocity) > maxAngularSpeed) {
        angularVelocity = angularVelocity / abs(angularVelocity) * maxAngularSpeed;
    }

    // Friction
    if(angularVelocity != 0) {
        double angVelSign = angularVelocity/abs(angularVelocity);
        double angFriction = (-angularStoppingDecel * deltaT * angVelSign)
            + (-angularVelocity * deltaT * viscousAngularCoeff);
        angularVelocity += angFriction;
        if(angVelSign != angularVelocity / abs(angularVelocity)) {
            angularVelocity = 0;
        }
    }

    if(glm::length(localVelocity) != 0) {
        glm::vec2 lastVelDir = glm::normalize(localVelocity);
        glm::vec2 linearFriction = calculateLinearFriction(
            localVelocity * (float)deltaT,
            (float)(stoppingDecel * deltaT));
        localVelocity += linearFriction;
        if (glm::length(lastVelDir - glm::normalize(localVelocity)) > 0.1) {
            localVelocity = glm::vec2(0, 0);
        }
    }

    glm::vec2 velocity = localToGlobal(localVelocity);

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

glm::vec2 XDrive::getNetForce() {
    glm::vec2 net(0, 0);
    for(auto m : motors) {
        net += m.getForce();
    }
    return net;
}

double XDrive::getNetTorque() {
    glm::vec3 net(0, 0, 0);
    for(auto m : motors) {
        net += glm::cross(glm::vec3(m.getPosition(), 0), glm::vec3(m.getForce(), 0));
    }
    return net.z;
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

glm::vec2 rotateVector(glm::vec2 vec, double angle) {
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

// ----------------- Motor Class ----------------- //

Motor::Motor() {}

Motor::Motor(glm::vec2 p, double o) {
    this->position = p;
    this->orientation = o;
}

glm::vec2 Motor::getForce() {
    glm::vec2 f(0, output);
    return customToGLM(rotateVector(glmToCustom(f), orientation));
}

glm::vec2 Motor::getPosition() {
    return position;
}

// -1 <= power <= 1
void Motor::setPower(double power) {
    if(abs(power) > 1) {
        power = power > 0 ? 1 : -1;
    }

    output = maxForce * power;

    // auto t = glfwGetTime();
    // auto deltaT = t - lastUpdate;

    // auto target = power * maxForce;
    // // If the target and current output are opposites
    // if(output/abs(output) != target/abs(target)) {
    //     output += std::min(jerk * deltaT * power, target - output);
    // }
    // else {
    //     // If the output is exceeding the target
    //     if(abs(output) > abs(target)) {
    //         printf("%f\n", output);
    //         output -= (output/abs(output)) * stoppingJerk;
    //     }
    //     // If the output is less than the target
    //     else {
    //         output += std::min(jerk * deltaT * power, target - output);
    //     }
    // }

    // if(abs(output) > maxForce) {
    //     output = output > 0 ? maxForce : -maxForce;
    // }

    // lastUpdate = t;
}