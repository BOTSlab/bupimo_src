/**
 * Manual Controller.
 */

#ifndef MANUALCONTROLLER_H
#define MANUALCONTROLLER_H

#include "Controller.h"
#include "ImgWindow.h"

class ManualController : public Controller {
public:
    ManualController(DriverAdaptor &inAdaptor);
    virtual ~ManualController();
    virtual void activate();
    virtual void deactivate();
    virtual void fire();
    virtual string getDescription();

protected:
    Img *img;
    ImgWindow *window;

private:
    // Current forward and rotational speed.
    double v, omega;

    // Maximum forward and rotational speed.
    double vMax, omegaMax;

    // Whether we should display a window with the current image.
    bool displayImage;
};

#endif
