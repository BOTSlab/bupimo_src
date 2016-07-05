#include "ImgWindow.h"
#include "ImgOps.h"
#include "OCVCamera.h"
#include "SiftExtractor.h"

Img* drawImg = NULL;

void drawKeys(vector<Keypoint*> &contractedKeys, 
              vector<Keypoint*> &expandedKeys, 
              vector<Keypoint*> &staticKeys, 
              Img &src, ImgWindow &window) {
    // Create a copy of the input image and draw dots at the locations of
    // the keypoints.
    if (drawImg == NULL)
        drawImg = new Img(src);
    else
        *drawImg = src;

    window.setImg(*drawImg);

    for (int i=0; i<contractedKeys.size(); i++) {
        int cx = (int) contractedKeys[i]->x;
        int cy = (int) contractedKeys[i]->y;
        window.drawCircle(cx, cy, 5, 1, 0, 0);
    }
    for (int i=0; i<expandedKeys.size(); i++) {
        int cx = (int) expandedKeys[i]->x;
        int cy = (int) expandedKeys[i]->y;
        window.drawCircle(cx, cy, 5, 0, 1, 0);
    }
    for (int i=0; i<staticKeys.size(); i++) {
        int cx = (int) staticKeys[i]->x;
        int cy = (int) staticKeys[i]->y;
        window.drawCircle(cx, cy, 5, 0, 0, 1);
    }

    window.refresh();

    //delete drawImg;
}

int main( int argc, char *argv[] ) {

    Camera *camera = new OCVCamera;
    Img *img = NULL;
    ImgWindow window("Camera image");

    // Capture initial image.
    camera->getImg(img);
    int width = img->getWidth();
    int height = img->getHeight();
    SiftExtractor extractor(width, height);

    // Capture keypoints to track.
    vector<Keypoint*> ssKeys;
    extractor.extract(*img, ssKeys);

    while (true) {
        // Capture new image.
        camera->getImg(img);

        // Extract keypoints.
        vector<Keypoint*> keys;
        extractor.extract(*img, keys);

        // Find matched keypoints.
        vector<Match> matches;
        extractor.match(keys, ssKeys, matches);
        cout << "matches: " << matches.size() << endl;

        // Put together a list of the matched keypoints.  Filter out those
        // which have moved only a short distance from the snapshot.
        double distThresh = 50.0;
        vector<Keypoint*> contractedKeys, expandedKeys, staticKeys;
        for (int i=0; i<matches.size(); i++) {
            Keypoint *key = keys[matches[i].a];
            Keypoint *ssKey = ssKeys[matches[i].b];

            // Filter out keys which have not moved very much.
            double dx = key->x - ssKey->x;
            double dy = key->y - ssKey->y;
            double distance = sqrt(dx*dx + dy*dy);
            if (distance > distThresh) {
                // Compare scales to determine if key has contracted or expanded
                if (key->sigma < ssKey->sigma)
                    contractedKeys.push_back(key);
                else if (key->sigma > ssKey->sigma)
                    expandedKeys.push_back(key);
                else
                    staticKeys.push_back(key);
            }
        }

        drawKeys(contractedKeys, expandedKeys, staticKeys, *img, window);

        // Free keypoints
        for (int i=0; i<keys.size(); i++) delete keys[i];
    }

    // Free keypoints
    for (int i=0; i<ssKeys.size(); i++) delete ssKeys[i];

    ImgWindow::waitAll();
    delete camera;
    return 0;
}
