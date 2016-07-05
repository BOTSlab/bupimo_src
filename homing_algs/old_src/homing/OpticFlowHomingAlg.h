/*
 * Uses optic flow techniques to find correspondences between images.
 * Then computes home direction using exact vector mapping technique.
 *
 * Andrew Vardy
 */
#ifndef OPTICFLOWHOMINGALG_H
#define OPTICFLOWHOMINGALG_H

#include <cmath>
#include "TransHomingAlg.h"
#include "BlockMatch.h"
#include "ExactVectorMap.h"
#include "HomeDirCombiner.h"
#include "ImgOps.h"

class OpticFlowHomingAlg : public TransHomingAlg {
public:
    OpticFlowHomingAlg( int inWidth, int inHeight );
    ~OpticFlowHomingAlg() {};
    void snapshot(Img* inSS);
    Vec2 currentView(Img* CV);
private:
    Img *SS, *U, *V;
    Img *Alpha, *Mask, *Weight;
    BlockMatch blockMatch;

    // These have been made into pointers so that we can wait until U and V
    // have been first computed, then we can use their size to build these
    // guys.
    ExactVectorMap *vectorMap;
    HomeDirCombiner *combiner;
};

#endif
