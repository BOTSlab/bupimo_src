/**
 * Go through a stored ImgSequence and extract a sparse route (a sequence of
 * images and odo data).  This is done by iterating through the sequence and
 * comparing home directions computed with odometry with directions computed by
 * visual homing.  The output consists of a sequence of images, along with
 * odometry and true position which are copied (from odo.txt and true.txt) to
 * the given output directory.
 *
 * @author Andrew Vardy
 */

#ifndef ROUTEEXTRACTOR_H
#define ROUTEEXTRACTOR_H

#include "ImgSequence.h"
#include "FeatureHissAlg.h"

class RouteExtractor {
public:
    RouteExtractor(string inSequenceDir, string inOutputDir,
                   double inThreshold, bool inAllowAdjacentExceed);

    virtual ~RouteExtractor();

    void extract();

private:
    double computeHomeAngleVH(int s, int c);
    double computeHomeAngleODO(int s, int c);
    void storeKeys(int inIndex);
    void storeMatchVector(string suffix, int s, int c);


    // The input sequence and the number of images it contains.
    ImgSequence inSequence;
    int nP;

    // The name of the output directory.
    string outputDir;

    // The angular difference threshold.
    double threshold;

    // Should we allow adjacent nodes to exceed the angular difference
    // threshold.  If false then an error message will be printed and the
    // program executed if the threshold is exceeded by adjacent nodes.
    bool allowAdjacentToExceedThreshold;

    int gIndex;

    ofstream *trueFile;
    ofstream *odoFile;
    ofstream *g2pFile;
    ofstream *goalFile;

    SiftExtractor *extractor;

    // An array of vectors of features, one for each image in the input
    // sequence.
    vector<Keypoint*> *keysArray;

    FeatureHomingAlg *homingAlg;

    vector<int> g2p;
};

#endif
