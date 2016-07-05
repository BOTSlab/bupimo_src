/**
 * Controller for servoing the robot to face the object in the scene that has
 * undergone some movement.
 */

#ifndef SIFTTRACKERCONTROLLER_H
#define SIFTTRACKERCONTROLLER_H

#include "Controller.h"
#include "ImgWindow.h"
#include "SiftExtractor.h"

class SiftTrackerController : public Controller {
public:
    SiftTrackerController();
    virtual ~SiftTrackerController();

private:
    void handleStateTransitions();
    bool handleStateEffects();

    void drawKeys(vector<Keypoint*> &filtKeys, Img &src, ImgWindow &window);

    ImgWindow window;
    Img *img;
    bool first;
    SiftExtractor *extractor;
    vector<Keypoint*> ssKeys, keys, filtKeys;
    vector<Match> matches;
    int width;
    Img* drawImg;
};

#endif
