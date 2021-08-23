#include "tracking.h"
#include "motionControl.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <chrono>
#include <thread>
#include "macros.h"

#define TURN_TOLERANCE 0.03
#define DISTANCE_TOLERANCE 0.5
#define TURN_INTEGRAL_TOLERANCE 0.3
#define DISTANCE_INTEGRAL_TOLERANCE 3

const float driveP = 4.00; // 4.00
const float driveI = 0.10;
const float driveD = 16.00; // 16.00

const float turnP = 1.0;
const float turnI = 0.5;
const float turnD = 2.5;

PIDInfo turnConstants(turnP, turnI, turnD);
PIDInfo driveConstants(driveP, driveI, driveD);

/**
 * Convert an angle from negative to positive, "flipping"
 * it around the positive x-axis
 * 
 * @param angle the angle to convert in radians
 * @return the converted angle in radians
*/
double flipAngle(double angle) {
	if(angle > 0) {
		return -(2 * M_PI) + angle;
	}
	else {
		return (2 * M_PI) + angle;
	}
}

/**
 * Get the distance from a starting point to a target
 * Made obsolete by the Vector2 class
 * 
 * @param tx x-coord of the target
 * @param tx y-coord of the target
 * @param sx x-coord of the start
 * @param sy y-coord of the start
 * @return the distance
*/
float getDistance(float tx, float ty, float sx, float sy) {
	float xDiff = tx - sx;
	float yDiff = ty - sy;
	return sqrt((xDiff*xDiff) + (yDiff*yDiff));
}

/**
 * Interface with the XDrive strafe class using custom
 * vector implementation
 * Stands in for the original PROS-based strafe function
 * used by the motion algos
 * 
 * @param dir the direction to strafe
 * @param turn the speed to turn at
*/
void strafe(Vector2 dir, double turn) {
    chassis.strafeGlobal(customToGLM(dir), turn);
}

/**
 * Strafe to a new position relative to the current position
 * 
 * @param offset the desired offset from the current pos in local space
 * @param aOffset the desired offset from the current angle in radians
*/
void strafeRelative(Vector2 offset, double aOffset) {
	if(offset.getMagnitude() == 0) {
		turnToAngle(trackingData.getHeading() + aOffset);
	}
	else if(aOffset == 0) {
		strafeToPoint(trackingData.getPos() + offset);
	}
	else {
		strafeToOrientation(trackingData.getPos() + offset, trackingData.getHeading() + aOffset);
	}
	return;
}

void alignAndShoot(Vector2 goal, double angle, uint8_t balls, bool intake) {

	strafeToOrientation(goal, angle);

	if(intake) {
		if(balls == 1) { shootClean(balls); }
		else { shootStaggeredIntake(balls); }
	}
	else {
		if (balls == 1) { shootClean(balls); }
		else { shootStaggered(balls); }
	}

	stopRollers();
}

/**
 * Strafe to a point on the field and turn to a
 * given angle at the same time. Prioritizes turning
 * via vector projection to prevent the chassis from arcing.
 * 
 * @param target the target spot on the field in global space
 * @param angle the desired final angle in global space
*/
void strafeToOrientation(Vector2 target, double angle) {
	trackingData.suspendAngleModulus();
	double time = glfwGetTime();
    angle = angle * M_PI / 180;
	if (abs(angle - trackingData.getHeading()) > degToRad(180)) {
		angle = flipAngle(angle);
	}
	PIDController distanceController(0, driveConstants, DISTANCE_TOLERANCE, DISTANCE_INTEGRAL_TOLERANCE);
	PIDController turnController(angle, turnConstants, TURN_TOLERANCE, TURN_INTEGRAL_TOLERANCE);

	do {
		// Angle controller
		float tVel = turnController.step(trackingData.getHeading());

		// Distance controller
		Vector2 delta = target - trackingData.getPos(); // distance vector
		Vector2 dNorm = delta.normalize();

		// Get the "forward" vector and calculate the dot product
		Vector2 alignment = rotateVector(dNorm, turnController.getError());
		float dotScalar = dot(alignment, dNorm);

		// Step drive PID if dot is not negative
		Vector2 driveVec(0, 0);
		if(dotScalar > 0) {
			float strVel = -distanceController.step(delta.getMagnitude() * dotScalar);
			driveVec = rotateVector(Vector2(strVel, 0), delta.getAngle());
		}

		strafe(driveVec, tVel);

		// Timeout after 4 secs if something goes wrong
		if(glfwGetTime() - time > 4) {
			break;
		}

		// pros::delay(20) equivalent
    	std::this_thread::sleep_for(std::chrono::milliseconds(20));
	} while(!distanceController.isSettled() || !turnController.isSettled());

	trackingData.resumeAngleModulus();
}

/**
 * Strafe to a point on the field without turning
 * 
 * @param target the target spot on the field in global space
*/
void strafeToPoint(Vector2 target) {
	double time = glfwGetTime();
	PIDController distanceController(0, driveConstants, DISTANCE_TOLERANCE, DISTANCE_INTEGRAL_TOLERANCE);

	do {
		Vector2 delta = target - trackingData.getPos();
		float vel = -distanceController.step(delta.getMagnitude());
		Vector2 driveVec = rotateVector(Vector2(vel, 0), delta.getAngle());
		strafe(driveVec, 0);

		if(glfwGetTime() - time > 4) {
			break;
		}

    	std::this_thread::sleep_for(std::chrono::milliseconds(20));
	} while(!distanceController.isSettled());
}

/**
 * Turn to a given angle without strafing
 * 
 * @param target the target angle in global space
*/
void turnToAngle(double target) {
	trackingData.suspendAngleModulus();

    target = target * M_PI / 180;
	if(abs(target - trackingData.getHeading()) > degToRad(180)) {
		target = flipAngle(target);
	}


	double time = glfwGetTime();
	PIDController turnController(target, turnConstants, TURN_TOLERANCE, TURN_INTEGRAL_TOLERANCE);

	do {
		float vel = turnController.step(trackingData.getHeading());
		strafe(Vector2(0, 0), vel);

		if(glfwGetTime() - time > 4) {
			break;
		}

    	std::this_thread::sleep_for(std::chrono::milliseconds(20));
	} while(!turnController.isSettled());

	trackingData.resumeAngleModulus();
}

PIDInfo::PIDInfo(double _p, double _i, double _d) {
    this->p = _p;
    this->i = _i;
    this->d = _d;
}

PIDController::PIDController(double _target, PIDInfo _constants, double _tolerance, double _integralTolerance = 1) {
	this->target = _target;
    this->lastError = target;
    this->constants = _constants;
    this->tolerance = _tolerance;
	this->integralTolerance = _integralTolerance;
}

/**
 * Step the PID loop forward with updated sensor input
 * 
 * @param newSense the updated sensor reading
 * @return the output determine by the PID algorithm
*/
double PIDController::step(double newSense) {

    // calculate error terms
    sense = newSense;
    error = target - sense;
	if(first) {
		lastError = error;
		first = false;
	}
    integral += error;
    derivative = error - lastError;
    lastError = error;
    // Disable the integral until it enters a usable range of error
    if(error == 0 || abs(error) > integralTolerance) {
        integral = 0;
    }
    speed = (constants.p * error) + (constants.i * integral) + (constants.d * derivative);
    if(abs(error) <= tolerance) {
		if(!settling) {
			settleStart = glfwGetTime();
		}
		settling = true;
        speed = 0;
		if(glfwGetTime() - settleStart > SETTLE_DELAY) {
			settled = true;
		}
    }
	else {
		settling = false;
		settled = false;
	}

    return speed;
}

void PIDController::reset() {
    target = 0;
    sense = 0;
    lastError = 0;
    integral = 0;
    derivative = 0;
    speed = 0;
    error = 0;
}

double PIDController::getError() {
    return target - sense;
}
bool PIDController::isSettled() {
	return settled;
}