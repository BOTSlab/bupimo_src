/*
 * Uses the Homing in Scale Space (HISS) method for visual homing.
 *
 * Requirements: VLFeat
 *
 * Andrew Vardy
 */

#ifndef HISSALG_H
#define HISSALG_H

#include "TotalHomingAlg.h"
#include "SiftExtractor.h"
#include "Angles.h"

class HissAlg : public TotalHomingAlg {
public:
    HissAlg(int inWidth, int height);
    virtual ~HissAlg();
    virtual void snapshot(Img* inSS);
    virtual void getHomeAngle(Img* CV, double &homeAngle, double &similarity);

private:
    Img *SS;
    int width;
    SiftExtractor extractor;
    bool useCon, useExp, deviationFilter;
    double stdevs;
};

#endif
