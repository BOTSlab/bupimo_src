#include "SingleConfig.h"
#include "Driver.h"
#include "ManualController.h"
#include "CaptureController.h"
#include "VisualHomingController.h"
#include "RouteHomingController.h"
#include "TestGoToController.h"
#include <glog/logging.h>

int main(int argc, char *argv[]) {

    google::InitGoogleLogging(argv[0]);

    Driver driver;
    DriverAdaptor &adaptor = driver.getDriverAdaptor();

    ManualController manual(adaptor);
    driver.addController(&manual);

    CaptureController cap(adaptor);
    driver.addController(&cap);

    VisualHomingController homing(adaptor);
    driver.addController(&homing);

    RouteHomingController routing(adaptor);
    driver.addController(&routing);

    TestGoToController testGoTo(adaptor);
    driver.addController(&testGoTo);
    
    driver.loop();
    return 0;
}
