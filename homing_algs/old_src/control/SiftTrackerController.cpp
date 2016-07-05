#include "SiftTrackerController.h"
#include <sstream>
using namespace std;

void SiftTrackerController::drawKeys(vector<Keypoint*> &filtKeys,
                                     Img &src, ImgWindow &window) {
    // Create a copy of the input image and draw dots at the locations of
    // the keypoints.
    if (drawImg == NULL)
        drawImg = new Img(src);
    else
        *drawImg = src;

    window.setImg(*drawImg);

    for (int i=0; i<filtKeys.size(); i++) {
        Keypoint *key = filtKeys[i];
        int cx = (int) key->x;
        int cy = (int) key->y;
        double sigmaDelta = key->sigma - key->match->sigma;

        int radius = 1 + (int)(10.0 * sigmaDelta);
        if (sigmaDelta > 0)
            window.drawCircle(cx, cy, radius, 1, 0, 0);
        else
            window.drawCircle(cx, cy, radius, 0, 1, 0);
        
        ostringstream text;
        text << (((int)(sigmaDelta * 1e3)) / 1e3);
        window.drawText(cx, cy, text.str().c_str(), 1, 1, 1);
    }

    window.refresh();
}

SiftTrackerController::SiftTrackerController() :
    Controller(true),
    window("Sift Tracker", 4),
    img(NULL),
    first(true)
{
}

SiftTrackerController::~SiftTrackerController() {
    // Free SS keypoints
    for (int i=0; i<ssKeys.size(); i++)
        delete ssKeys[i];
    delete drawImg;
    listener.print("SiftTrackerController: destructor");
    cout << "SiftTrackerController: destructor" << endl;
}

void SiftTrackerController::handleStateTransitions() {
}

bool SiftTrackerController::handleStateEffects() {
    if (first) {
        // Capture initial image.
        camera->getImg(img);
        width = img->getWidth();
        extractor = new SiftExtractor(width, img->getHeight());
        first = false;

        // Capture keypoints to track.
        extractor->extract(*img, ssKeys);
    } else {

        // Free keypoints
        for (int i=0; i<keys.size(); i++)
            delete keys[i];
        keys.clear();
        filtKeys.clear();
        matches.clear();
    
        // Capture new image.
        camera->getImg(img);

        // Extract keypoints.
        extractor->extract(*img, keys);

        // Find matched keypoints.
        extractor->match(keys, ssKeys, matches);

        // Put together a list of the matched keypoints.  Filter out those
        // which have moved only a short distance from the snapshot.
        double distThresh = -50.0; // 50;
        int nMovedLeft = 0, nMovedRight = 0;
        double avgLeftDX = 0, avgRightDX = 0;
        for (int i=0; i<matches.size(); i++) {
            Keypoint *key = keys[matches[i].a];
            Keypoint *ssKey = ssKeys[matches[i].b];

            // Add the current key's match.
            key->match = ssKey;

            // Filter out keys which have not moved very much.
            double dx = key->x - ssKey->x;
            double dy = key->y - ssKey->y;
            double distance = sqrt(dx*dx + dy*dy);
            if (distance > distThresh) {

                filtKeys.push_back(key);
                if (dx > 0) {
                    nMovedRight++;
                    avgRightDX += dx;
                } else {
                    nMovedLeft++;
                    avgLeftDX -= dx;
                }
            }
        }
        if (nMovedRight > 0) avgRightDX /= nMovedRight;
        if (nMovedLeft > 0) avgLeftDX /= nMovedLeft;

        drawKeys(filtKeys, *img, window);


        int threshold = 6;
        double k_omega = 0.5 / width;
        double omega;
        if (nMovedRight - nMovedLeft > threshold)
            omega = -k_omega*avgRightDX;
        else if (nMovedLeft - nMovedRight > threshold)
            omega = k_omega*avgRightDX;
        else
            omega = 0.0;
        position.SetSpeed(0.0, omega);
        
        return false;
    }
}
