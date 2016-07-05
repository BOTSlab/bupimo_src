#include "CaptureController.h"
#include <sstream>
#include <iomanip>

CaptureController::CaptureController(DriverAdaptor &inAdaptor) :
    ManualController(inAdaptor),
    state(FIRST),
    index(0),
    displacementSinceCapture(0),
    lastX(0),
    lastY(0),
    odoFile("odo.txt"),
    WAIT_AFTER_CAPTURE(true),
    CAPTURE_DISPLACEMENT(0.5)
{
    odoFile << "#odoX odoY odoTh" << endl;
}

CaptureController::~CaptureController() {
}

void CaptureController::activate() {
}

void CaptureController::deactivate() {
}

void CaptureController::fire() {

    if (state == FIRST || state == CAPTURE) {
        // Capture!  Determine filename and then save the image to the cur. dir.
        adaptor.getImg(img);
        ostringstream oss;
        oss << setfill('0') << setw(3);
        oss << index;
        oss << ".bmp";
        adaptor.print("Saving:");
        adaptor.print(oss.str());
        img->save(oss.str());

        adaptor.captureFromObservationCamera(oss.str());

        // Print the robot's pose to the text file.
        odoFile << adaptor.getX() << " " << adaptor.getY() << " " 
                << adaptor.getTheta() << endl;

        index++;
        displacementSinceCapture = 0;
        if (WAIT_AFTER_CAPTURE) {
            state = WAITING;
            adaptor.print("CaptureController: Hit 'c' to continue...");
        } else
            state = MOVING;

    } else if (state == WAITING) {
        // We have just captured an image.  We should wait for confirmation
        // from the user before continuing (he may need to place a marker at
        // the capture site).
        int key = adaptor.getKey();
        if (key == 'c') {
            adaptor.print("CaptureController: Manual control restored.");
            state = MOVING;
        }
        adaptor.setSpeed(0, 0);

    } else if (state == MOVING) {

        ManualController::fire();
        double x = adaptor.getX();
        double y = adaptor.getY();
        double dx = x - lastX;
        double dy = y - lastY;
        lastX = x;
        lastY = y;
        displacementSinceCapture += sqrt(dx*dx + dy*dy);
        if (displacementSinceCapture > CAPTURE_DISPLACEMENT) {
            adaptor.stop();
            state = CAPTURE;
        }
    } 
}

string CaptureController::getDescription() {
    return "CaptureController";
}
