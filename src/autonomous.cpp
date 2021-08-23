#include "motionControl.h"
#include "tracking.h"
#include "macros.h"
#include <chrono>
#include <thread>

// Goals
Vector2 goalBL(17.5, 5.5);
Vector2 goalCL(20, 74);
Vector2 goalTL(15, 128);
Vector2 goalTC(74, 127);
Vector2 goalTR(130, 131);
Vector2 goalCR(122, 67);
Vector2 goalBR(140 - 17.5, 5.5);
Vector2 goalBC(70, 7);

void fullAuto();
void homeRowAuto();

/**
 * Autonomous function. Runs in a thread started in main
*/
void myAutonomous() {
    // UNCOMMENT THIS IN THE PROS PROJECT
    // update = pros::Task(updateSysMan, (void *)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Update system manager");
    // rollers.fullReset();
    // intake.fullReset();

    suspendDrive = true;

    // ------------- Used to test PID constants ------------- //
    // while(1) {
    //     strafeToPoint(Vector2(144 / 2, 144 / 2));
    //     // strafeToPoint(Vector2(30, 144 - 30));
    //     // strafeToPoint(Vector2(144 - 30, 144 - 30));
    //     // strafeToPoint(Vector2(144 - 30, 30));
    //     // strafeToPoint(Vector2(30, 30));

    //     strafeToPoint(Vector2(40, 40));
    //     strafeToPoint(Vector2(50, 40));

    //     turnToAngle(90);
    //     turnToAngle(180);
    //     turnToAngle(-90);
    //     turnToAngle(90);
    // }

    // Run the desired auto routine
    // fullAuto();

    // strafeToPoint(Vector2(70, 70));
    // turnToAngle(90);
    // turnToAngle(190);
    // turnToAngle(359);

    // suspendDrive = false;
    // return;

    // Flipout and get second ball
    flipout();
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    in();
    strafeRelative(Vector2(0, 18), 0);
	stopRollers();

    // Score a ball in the first goal
    alignAndShoot(goalBL, 135, 1);

    // Get the center-bottom-middle ball
	strafeToPoint(Vector2(24, 18));
	Vector2 secondBall(36 + 36, 42);
	turnToAngle(radToDeg((secondBall-trackingData.getPos()).getAngle()) - 90);
	in();
	strafeToPoint(secondBall);

    // Score 2 in the center-bottom goal
    turnToAngle(-180);
    stopRollers();
    alignAndShoot(goalBC, radToDeg(trackingData.getHeading()), 2);

    // Pick up 4th ball
	strafeToPoint(secondBall);
	Vector2 thirdBall(secondBall.getX() + 36, secondBall.getY() - 26);
	turnToAngle(radToDeg((thirdBall - trackingData.getPos()).getAngle()) - 90);
	in();
	strafeToPoint(thirdBall);

    // Score 1 in the bottom-right
    turnToAngle(-135);
    stopRollers();
    alignAndShoot(goalBR, -135, 1);

    // Back up and pick up the 5th ball
    strafeRelative(Vector2(-20, 20));
    in();
    strafeToOrientation(Vector2(114.5, 45), -10);

    strafeToPoint(Vector2(114.5, 67));

    // Score in the 4th goal
    turnToAngle(-90);
    stopRollers();
    alignAndShoot(goalCR, -90, 1);
    strafeRelative(Vector2(-20, 0));

    Vector2 topRightBall = thirdBall + Vector2(0, 86);
    in();
    strafeToOrientation(topRightBall, -1);
    turnToAngle(-50);
    stopRollers();
    strafeToPoint(topRightBall + Vector2(20, 10));
    shootStaggered(3);
    stopRollers();

    suspendDrive = false;

    // UNCOMMENT THIS IN THE PROS PROJECT
	// rollers.reset();
	// intake.reset();
	// update.remove();
}

/**
 * Autonomous routines that scores >=1 ball in each goal
*/
void fullAuto() {
    flipout();

    // Flipout
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    in();
    strafeToPoint(Vector2(36, 28));
    stopRollers();

    // First goal
    strafeToOrientation(goalBL, 135);
    shootStaggered(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    stopRollers();

    // Second ball
    strafeToPoint(Vector2(28, 24));
    in();
    strafeToOrientation(Vector2(13.5, 32), 60);

    // Third ball
    strafeToOrientation(Vector2(36, 56), 30);
    strafeToPoint(Vector2(30.5, 70));
    stopRollers();

    // Second goal
    strafeToOrientation(goalCL, 90);
    shootStaggered(2);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    stopRollers();

    // Fourth and fifth balls
    in();
    strafeToPoint(goalCL + Vector2(10, 0));
    strafeToOrientation(Vector2(14.5, 106), 72);
    Vector2 fifthBall(28.5, 117.5);
    turnToAngle(-45);
    strafeToPoint(fifthBall);

    // Third goal
    strafeToOrientation(goalTL, 45);
    shootStaggered(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    stopRollers();

    // Sixth ball
    strafeToPoint(trackingData.getPos() + Vector2(10, -10));
    in();
    strafeToOrientation(Vector2(62.5, 105), -115);

    // Fourth goal
    stopRollers();
    strafeToOrientation(goalTC, 0);
    shootStaggered(2);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    stopRollers();

    // Seventh ball
    strafeToPoint(trackingData.getPos() - Vector2(0, 10));
    Vector2 seventhBall(109, 123.5);
    in();
    strafeToOrientation(seventhBall, radToDeg((seventhBall - trackingData.getPos()).getAngle()));

    // Fifth goal
    stopRollers();
    strafeToOrientation(goalTR, -45);
    shootStaggered(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    stopRollers();

    // Eighth ball
    strafeToPoint(trackingData.getPos() + Vector2(-30, -30));
    in();
    turnToAngle(-135);
    strafeToPoint(Vector2(112, 85));

    // Sixth goal
    stopRollers();
    strafeToOrientation(goalCR, -90);
    shootStaggered(2);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    stopRollers();

    // Ninth ball
    strafeToPoint(trackingData.getPos() + Vector2(-10, 0));
    Vector2 ninthBall(108, 28);
    turnToAngle(radToDeg((ninthBall - trackingData.getPos()).getAngle()));
    in();
    strafeToPoint(ninthBall);
    
    // Seventh goal
    stopRollers();
    strafeToOrientation(goalBR, -135);
    shootStaggered(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    stopRollers();

    // Tenth ball
    strafeToPoint(trackingData.getPos() + Vector2(-30, 30));
    turnToAngle(90);
    in();
    strafeToPoint(Vector2(73, 55.5));
    stopRollers();

    // Eighth goal
    strafeToOrientation(goalBC, 180);
    shootStaggered(2);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    stopRollers();

    // Last goal
    strafeToPoint(trackingData.getPos() + Vector2(0, 10));
    turnToAngle(0);
    in();
    strafeToPoint(Vector2(72.5, 64.5));
    shootStaggeredIntake(5);
}

/**
 * Auto routine that expands on the in-person tested home row routine
*/
void homeRowAuto() {
    flipout();

    // Flipout
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    in();
    strafeToPoint(Vector2(36, 28));
    stopRollers();

    // First goal
    strafeToOrientation(goalBL, 135);
    shootStaggered(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    stopRollers();

    // Second ball
    strafeToPoint(Vector2(28, 24));
    Vector2 secondBall(73.5, 50.5);
    turnToAngle(radToDeg((secondBall - trackingData.getPos()).getAngle()) - 90);
    in();
    strafeToPoint(secondBall);

    // Second goal
    strafeToOrientation(goalBC, 180);
    shootStaggered(2);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    stopRollers();

    // Third ball
    strafeToPoint(secondBall);
    Vector2 thirdBall(109, 25);
    turnToAngle(radToDeg((thirdBall - trackingData.getPos()).getAngle()) - 90);
    in();
    strafeToPoint(thirdBall);

    // Third goal
    turnToAngle(-135);
    strafeToOrientation(goalBR, -135);
    shootStaggered(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    stopRollers();

    // Fourth ball
    strafeToPoint(trackingData.getPos() + Vector2(-20, 20));
    turnToAngle(-90);
    in();
    strafeToPoint(Vector2(129, 38.5));

    // Fifth ball
    strafeToOrientation(Vector2(100, 38.5), -40);
    strafeToPoint(Vector2(113.5, 65.5));
    stopRollers();

    // Fourth goal
    strafeToOrientation(goalCR, -90);
    shootStaggered(3);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    stopRollers();

    // Sixth ball
    strafeToPoint(trackingData.getPos() + Vector2(-10, 0));
    in();
    strafeToOrientation(Vector2(129.5, 105), -60);
    turnToAngle(45);
    strafeToPoint(Vector2(117.5, 115));

    // Fifth goal
    strafeToOrientation(goalTR, -45);
    shootStaggered(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    stopRollers();
}