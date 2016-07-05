/*
 * Uses the HiSS method for visual homing.  The core code for HiSS is in
 * the 'hiss' subdirectory.
 *
 * Andrew Vardy
 */

#ifndef OLDHISSALG_H
#define OLDHISSALG_H

#include "TotalHomingAlg.h"
#include "Angles.h"
#include "SIFTHoming.h"

class OldHissAlg : public TotalHomingAlg {
public:
    OldHissAlg(int config=0);
    ~OldHissAlg();
    virtual void snapshot(Img* inSS);
    virtual void getHomeAngle(Img* CV, double &homeAngle, double &similarity);
    virtual double getSimilarity(Img* A, Img* B);
private:

    SIFTHoming siftHoming;
};

#endif
