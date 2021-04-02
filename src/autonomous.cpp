#include "motionControl.h"
#include "tracking.h"
#include "macros.h"
#include <chrono>
#include <thread>

void myAutonomous() {
    // UNCOMMENT THIS IN THE PROS PROJECT
    // update = pros::Task(updateSysMan, (void *)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Update system manager");
    // rollers.fullReset();
    // intake.fullReset();

    suspendDrive = true;

    flipout();

    strafeToPoint(Vector2(144/2, 144/2));

    // std::this_thread::sleep_for(std::chrono::milliseconds(400));
    // in();
    // strafeToPoint(Vector2(24, 18));
	// stopRollers();
	// strafeToOrientation(Vector2(5.5, 5.5), 135);
	// shootStaggered();
    // std::this_thread::sleep_for(std::chrono::milliseconds(600));
    // stopRollers();
    // strafeToPoint(Vector2(24, 18));
	// Vector2 secondBall(36 + 24, 38);
	// turnToAngle(radToDeg((secondBall-trackingData.getPos()).getAngle()) - 90);
	// in();
	// strafeToPoint(secondBall);
    // std::this_thread::sleep_for(std::chrono::milliseconds(400));
    // stopRollers();
    // turnToAngle(-180);
	// strafeToPoint(Vector2(59, 11));
	// shootStaggered();
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // stopRollers();
    // strafeToPoint(secondBall);
	// Vector2 thirdBall(secondBall.getX() + 36, secondBall.getY() - 24);
	// turnToAngle(radToDeg((thirdBall - trackingData.getPos()).getAngle()) - 90);
	// in();
	// strafeToPoint(thirdBall);
    // std::this_thread::sleep_for(std::chrono::milliseconds(400));
    // stopRollers();
    // turnToAngle(-135);
	// strafeToOrientation(Vector2(115, 5), -135);
	// shootStaggeredIntake();
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // stopRollers();

    suspendDrive = false;

    // UNCOMMENT THIS IN THE PROS PROJECT
	// rollers.reset();
	// intake.reset();
	// update.remove();
}