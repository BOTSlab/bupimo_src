#include "Odo1Controller.h"

Odo1Controller::Odo1Controller(DriverAdaptor &inAdaptor)
    : OdoHomingController(inAdaptor),
      k_v(1.5), // create: 0.5
      k_omega(2.0) // create: 2.0
{
}

void Odo1Controller::activate() {
}

void Odo1Controller::deactivate() {
}

void Odo1Controller::fire() {
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

    // Apply the control law which attempts to minimize goalRX and goalRY
    // simultaneously.
    double v = k_v * goalRX;
    double omega = k_omega * goalRY;

    if (fabs(goalRY) > fabs(goalRX)) {
        // We need to make a big turn.  If the goal is nearby then omega may
        // not be large enough to exceed the stall speed.  So we will turn on
        // the spot instead of using the usual control law.
        v = 0;

        // The angle to turn is at least 45.  So we don't bother with
        // proportional control, but simply go for a fast turn.
        omega = 0.5*k_omega;
        if (goalRY < 0)
            omega *= -1;
    }

    // Does the robot appear to be stalled?
    stalled = (fabs(lastGoalRX) - fabs(goalRX) < 0.01 &&
               fabs(lastGoalRY) - fabs(goalRY) < 0.01);

    /*
    ostringstream oss;
    oss << "goalRX: " << goalRX << ", goalRY: " << goalRY << ", stalled: "
        << stalled;
    adaptor.print(oss.str());
    */

    adaptor.setSpeed(v, omega);

    lastGoalRX = goalRX;
    lastGoalRY = goalRY;
}

string Odo1Controller::getDescription() {
    return "Odo1Controller";
}

bool Odo1Controller::isStalled() {
    return stalled;
}
