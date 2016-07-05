#include "SimpleOdoController.h"

SimpleOdoController::SimpleOdoController(DriverAdaptor &inAdaptor)
    : OdoHomingController(inAdaptor),
      k_v(1.0), // create: 0.5
      k_omega(2.0) // create: 0.5
{
}

void SimpleOdoController::activate() {
}

void SimpleOdoController::deactivate() {
}

void SimpleOdoController::fire() {
    // Compute goal pose in robot coordinates.  Formula: R(theta)(goalI - posI)
    // where goalI and posI are vectors describing the position of the goal and
    // the robot, respectively.  R is the clockwise rotation matrix.  Note that
    // this controller does not correct for final orientation at the goal.
    double th = adaptor.getTheta();
    double diffX = goalIX - adaptor.getX();
    double diffY = goalIY - adaptor.getY();
    double c = cos(th);
    double s = sin(th);
    double goalRX = c * diffX + s * diffY;
    double goalRY = -s * diffX + c * diffY;
    double desiredTh = atan2(goalRY, goalRX);

    double v = k_v;
    double omega = desiredTh * k_omega;

    adaptor.setSpeed(v, omega);

    lastGoalRX = goalRX;
    lastGoalRY = goalRY;
}

string SimpleOdoController::getDescription() {
    return "SimpleController";
}

bool SimpleOdoController::isStalled() {
    return false;
}
