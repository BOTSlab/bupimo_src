/*
 * Pure abstract class for a homing algorithm that computes the translation
 * component between two omnidirectional images without any assumptions about
 * the rotation component of motion.  That is, the images do not have to be
 * aligned, yet the estimate of translation between them should still be
 * correct (hopefully).
 *
 * \author Andrew Vardy
 */

#ifndef TOTALHOMINGALG_H
#define TOTALHOMINGALG_H

#include "HomingAlg.h"

class TotalHomingAlg : public HomingAlg {
public:
    virtual ~TotalHomingAlg() {};

    /**
     * Sets 'homeAngle' to the homing angle for the current image CV paired
     * with the stored snapshot image.  This angle is in radians and is
     * measured counterclockwise with respect to the robot's nose.  Also sets
     * the similarity between these two images in 'similarity'.
     */
    virtual void getHomeAngle(Img* CV, double &homeAngle, double &similarity) 
    = 0;

    /**
     * Read the local 'config.cfg' and create a corresponding HomingAlg.
     */
    static TotalHomingAlg* createFromConfig(int width, int height);
};

#endif
