/*
 * Pure abstract class for a homing algorithm that computes the rotation
 * component between two omnidirectional images.
 *
 * \author Andrew Vardy
 */

#ifndef ROTHOMINGALG_H
#define ROTHOMINGALG_H

#include "HomingAlg.h"

class RotHomingAlg : public HomingAlg {
public:
    virtual double currentView( Img* CV ) = 0;
};

#endif
