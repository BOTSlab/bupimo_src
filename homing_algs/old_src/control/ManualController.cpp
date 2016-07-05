#include "ManualController.h"
#include <sstream>

ManualController::ManualController(DriverAdaptor &inAdaptor) :
    Controller(inAdaptor),
    img(NULL), window(NULL),
    vMax(0.1), omegaMax(0.4), displayImage(false)
{
    if (displayImage)
        window = new ImgWindow("ManualController: Camera Image");
}

ManualController::~ManualController() {
    if (displayImage)
        delete window;
}

void ManualController::activate() {
}

void ManualController::deactivate() {
}

void ManualController::fire() {
    if (displayImage) {
        adaptor.getImg(img);
        window->setImg(*img);
        window->refresh();
    }

    int key = adaptor.getKey();
    // Modify forward velocity.
    if (key == KEY_UP    /*keyboard*/ || key == 'w') {
        v = vMax;
    } else {
        v = 0;
    }

    // Modify rotationl velocity.
    if (key == KEY_RIGHT /*keyboard*/ || key == 'd'){
        omega = -omegaMax;
    } else if (key == KEY_LEFT  /*keyboard*/ || key == 'a'){
        omega = omegaMax;
    } else {
        omega = 0;
    }

    adaptor.setSpeed(v, omega);
}

string ManualController::getDescription() {
    return "ManualController";
}
