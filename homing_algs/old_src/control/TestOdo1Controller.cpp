#include "TestOdo1Controller.h"

TestOdo1Controller::TestOdo1Controller(DriverAdaptor &inAdaptor) :
    Controller(inAdaptor),
    odo1Controller(inAdaptor),
    index(-1)
{
}

TestOdo1Controller::~TestOdo1Controller() {
}

void TestOdo1Controller::activate() {
    // Pick new goal.
    odo1Controller.setGoal(1, 1, 0);
}

void TestOdo1Controller::deactivate() {
}

void TestOdo1Controller::fire() {
    odo1Controller.fire();
}

string TestOdo1Controller::getDescription() {
    return "TestOdo1Controller";
}
