/*
 * Uses block matching to find correspondences between images.  Computing home
 * direction using exact vector mapping technique.  Also incorporates a weight
 * on the home vectors based on the similarity between home vector direction and
 * the direction estimated by odometry.
 *
 * Andrew Vardy
 */
#ifndef WEIGHTEDBM_H
#define WEIGHTEDBM_H

#include <cmath>
#include "TransHomingAlg.h"
#include "BlockMatch.h"
#include "ExactVectorMap.h"
#include "HomeDirCombiner.h"
#include "ImgOps.h"
#include "ImgWindow.h"
#include "Angles.h"

class WeightedBM : public TransHomingAlg {
public:
    WeightedBM( int inWidth, int inHeight,
                int inLearnSteps, bool inUseProduct, bool inDebug );
    ~WeightedBM();
    Vec2 currentView( Img* CV );

private:
    float probDensity( float alpha );
    void learn();

    int width, height, learnSteps;
    bool useProduct;
    Img U, V, Mask, Corr, Alpha, Weight, Diff, Prob, Product, FinalWeight;
    Img* CorrVecs;
    ImgWindow *imgWindow;
    BlockMatch blockMatch;
    ExactVectorMap vectorMap;
    HomeDirCombiner combiner;
    bool firstLearn;
    int learnIndex;
    bool debug;
};

#endif
