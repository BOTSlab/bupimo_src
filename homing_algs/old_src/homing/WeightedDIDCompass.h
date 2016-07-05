/*
 *
 * Andrew Vardy
 */
#ifndef WEIGHTEDDIDCOMPASS_H
#define WEIGHTEDDIDCOMPASS_H

#include "RotHomingAlg.h"
#include "ImgWindow.h"
#include "ImgOps.h"
#include "Angles.h"

class WeightedDIDCompass : public RotHomingAlg {
public:
    enum WeightBasis { DENSITY, DISCRETE };
    enum WeightComb { MULTIPLY, ADD, AND, OR };

    WeightedDIDCompass( int inWidth, int inHeight,
                        int learnStart, int inLearnStop,
                        WeightBasis inWeightBasis, WeightComb inWeightComb,
                        int inSmoothRadius, double inSigma, bool inDebug );
    ~WeightedDIDCompass();
    double currentView( Img* CV );

private:
    void learn( Img *SS, Img *CV );
    void computeFinalWeight();
    float computeAngle( Img *SS, Img *CV, bool useWeight );

    float probDensity( float alpha );
    void computeMinShift( Img *SS, Img *CV );

    int width, height, learnStart, learnStop, learnIndex;
    WeightBasis weightBasis;
    WeightComb weightComb;
    int smoothRadius;
    double sigma;
    Img Rotated, SqdDiff, SqdDiffSmoothed, MinShift, MinShiftValue, InstWeight,
        Weight, FinalWeight;
    bool useBinaryWeights, useRandomWeights, debug;

    ImgWindow *imgWindow;
};

#endif
