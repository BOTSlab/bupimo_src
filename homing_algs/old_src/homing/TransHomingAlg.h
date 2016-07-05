/*
 * Pure abstract class for a homing algorithm that computes the translation
 * component between two omnidirectional images, assuming that the rotation
 * component of motion has already been removed.
 *
 * \author Andrew Vardy
 */
#ifndef TRANSHOMINGALG_H
#define TRANSHOMINGALG_H

#include "HomingAlg.h"
#include "Vec2.h"

class TransHomingAlg : public HomingAlg {
public:
    virtual Vec2 currentView(Img* CV) = 0;
};

#endif
