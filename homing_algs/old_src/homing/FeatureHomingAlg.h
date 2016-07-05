/**
 * Pure abstract class for a feature-based homing algorithm that computes the
 * translation component between two omnidirectional images without any
 * assumptions about the rotation component of motion.  That is, the images do
 * not have to be aligned, yet the estimate of translation between them should
 * still be correct (hopefully).
 *
 * This class and all sub-classes are currently based on SIFT features,
 * extracted via 'SiftExtractor'.
 *
 * @author Andrew Vardy
 */

#ifndef FEATUREHOMINGALG_H
#define FEATUREHOMINGALG_H

#include "SiftExtractor.h"

class FeatureHomingAlg {
public:
    /**
     * Returns the home angle when the keys in 'cvKeys' are matched to those in
     * 'ssKeys' according to the pre-comupted matches in 'matches'.  The angle
     * returned is in radians measured counterclockwise with respect to the
     * robot's nose.
     */
    virtual double getHomeAngle(vector<Keypoint*> &ssKeys,
                                vector<Keypoint*> &cvKeys,
                                vector<Match> &matches) = 0;
};

#endif
