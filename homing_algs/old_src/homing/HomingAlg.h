/*
 * Pure abstract class for a homing algorithm, which could either compute
 * the translation or the rotation between the current and snapshot images.
 *
 * Andrew Vardy
 */

#ifndef HOMINGALG_H
#define HOMINGALG_H

#include "Img.h"

class HomingAlg {
public:
    virtual void snapshot(Img* inSS) = 0;
};

#endif
