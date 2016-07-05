/**
 * Given a configuration parameter and two images from the command-line, draw
 * the SIFT keys for both images, the matches between them, and the position of
 * the computed home direction according to HiSS (Homing in Scale Space).
 *
 * Usage:
 *
 *      drawHissMatches CONFIG IMAGE_S IMAGE_C IDEAL
 *
 * CONFIG should be an integer from 0-2 and will be interpreted as follows:
 *      0: Draw only the contracted features and matches and the home direction
 *         computed from these features.
 *      1: Draw only the expanded features and matches and the home direction
 *         computed from these features.
 *      2: Draw all features and matches and the overall home direction.
 *
 * IDEAL is the ideal angle for this pair of images, specified in degrees.
 */

#include "ImgWindow.h"
#include "ImgOps.h"
#include "SiftExtractor.h"
#include "Angles.h"
#include <cstdlib>
#include <iostream>
#include <string>
using namespace std;

const int GAP = 5;
const int BOTTOM_BORDER = 50;

void drawMarker(ImgWindow &window, double angle, int width, int height, 
                double red, double green, double blue, int lineRadius) {

    angle = Angles::constrainAngle(angle);
    if (angle < 0) angle += TWO_PI;
    int homeX = (int)(width * (1.0 - angle / TWO_PI));
    for (int x=homeX-lineRadius; x<=homeX+lineRadius; x++)
        window.drawArrow(x, 2*height+GAP+BOTTOM_BORDER, x, 2*height+GAP,
                         red, green, blue);
}

int main( int argc, char *argv[] ) {

    if (argc != 5) {
        cout << "usage:\n\tdrawHissMatches CONFIG IMAGE_S IMAGE_C IDEAL" <<
             endl;
        return 0;
    }

    int config = atoi(argv[1]);
    assert(config >= 0 && config <= 2);

    Img S(argv[2]);
    Img C(argv[3]);
    int width = S.getWidth();
    int height = S.getHeight();
    SiftExtractor extractor(width, height);

    assert(S.hasKeypoints());
    assert(C.hasKeypoints());
    vector<Keypoint*> &sKeys = S.getKeypoints();
    vector<Keypoint*> &cKeys = C.getKeypoints();

    // Compute matches between S and C.
    vector<Match> matches;
    extractor.match(sKeys, cKeys, matches);

    // Do the HiSS thing...
    vector<Keypoint*> contracted;
    vector<Keypoint*> expanded;
    for (int m=0; m<matches.size(); m++) {
        Keypoint *keyInS = sKeys[matches[m].a];
        Keypoint *keyInC = cKeys[matches[m].b];
        if (keyInC->sigma > keyInS->sigma)
            expanded.push_back(keyInC);
        else if (keyInC->sigma != keyInS->sigma)
            contracted.push_back(keyInC);
    }
    int nCon = (int) contracted.size(); 
    int nExp = (int) expanded.size(); 

    // Compute the angular mean of these two feature sets.
    double cosSum = 0;
    double sinSum = 0;
    for (int i=0; i<nCon; i++) {
        Keypoint *key = contracted[i];
        double theta = TWO_PI * (1.0 - key->x / width);
        cosSum += cos(theta);
        sinSum += sin(theta);
    }
    double bearingCon = atan2(sinSum, cosSum);
    sinSum = 0;
    cosSum = 0;
    for (int i=0; i<nExp; i++) {
        Keypoint *key = expanded[i];
        double theta = TWO_PI * (1.0 - key->x / width);
        cosSum += cos(theta);
        sinSum += sin(theta);
    }
    double bearingExp = atan2(sinSum, cosSum);

    // Finally compute the weighted average.
    double s, c;
    if (config == 2) {
        s = nCon * sin(bearingCon) + nExp * sin(bearingExp + M_PI);
        c = nCon * cos(bearingCon) + nExp * cos(bearingExp + M_PI);
    } else if (config == 1) {
        s = sin(bearingExp + M_PI);
        c = cos(bearingExp + M_PI);
    } else if (config == 0) {
        s = sin(bearingCon);
        c = cos(bearingCon);
    } else {
        assert(false);
    }
    double homeAngle = atan2(s, c);

    // Stack the two images so that we can draw the match vectors.
    Img Big(S.getWidth(), S.getHeight()*2 + GAP + BOTTOM_BORDER);
    Big.setAll(1);
    for (int i=0; i<width; i++) {
        for (int j=0; j<height; j++) {
            Big.set(i, j, S.get(i, j));
            Big.set(i, height+GAP+j, C.get(i, j));
        }
    }

    ImgWindow window("HiSS");
    window.addImg(Big);

    // Draw the matches
    if (config == 2 || config == 0) {
        for (int i=0; i<nCon; i++) {
            Keypoint *key = contracted[i];
            int x0 = key->match->x;
            int y0 = key->match->y;
            int x1 = key->x;
            int y1 = key->y + height + GAP;
            window.drawLine(x0, y0, x1, y1, 0, 1, 0);
        }
    }
    if (config == 2 || config == 1) {
        for (int i=0; i<nExp; i++) {
            Keypoint *key = expanded[i];
            int x0 = key->match->x;
            int y0 = key->match->y;
            int x1 = key->x;
            int y1 = key->y + height + GAP;
            window.drawLine(x0, y0, x1, y1, 1, 0, 0);
        }
    }
    
    // Draw a marker on the bottom indicating the position of the centre of
    // contraction, expansion, or overall home direction.

    if (config == 0 || config == 2)
        drawMarker(window, bearingCon, width, height, 0, 1, 0, 1);

    if (config == 1 || config == 2)
        drawMarker(window, bearingExp + PI, width, height, 1, 0, 0, 1);

    drawMarker(window, homeAngle, width, height, 0, 0, 0, 1);


    // Draw the marker for the ideal angle (specified in degrees).
    double angle = PI_OVER_180 * atoi(argv[4]);
    drawMarker(window, angle, width, height, 0, 0, 1, 0);

    window.interact();
    return 0;
}
