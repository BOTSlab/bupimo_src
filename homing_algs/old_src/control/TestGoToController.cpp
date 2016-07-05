#include "TestGoToController.h"

TestGoToController::TestGoToController(DriverAdaptor &inAdaptor) :
    Controller(inAdaptor)
{
}

TestGoToController::~TestGoToController() {
}

void TestGoToController::activate() {
    adaptor.print("goTo called");
    adaptor.goTo(adaptor.getX() + 1.0, adaptor.getY(), 0);//1.57);
}

void TestGoToController::deactivate() {
}

void TestGoToController::fire() {
}

string TestGoToController::getDescription() {
    return "TestGoToController";
}
