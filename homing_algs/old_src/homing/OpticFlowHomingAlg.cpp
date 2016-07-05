#include "OpticFlowHomingAlg.h"

OpticFlowHomingAlg::OpticFlowHomingAlg( int inWidth, int inHeight ) : 
    SS(NULL),
    U(NULL),
    V(NULL),
    Alpha(NULL),
    Mask(NULL),
    Weight(NULL),
    blockMatch(inWidth, inHeight),
    vectorMap(NULL),
    combiner(NULL)
{
}

void OpticFlowHomingAlg::snapshot(Img* inSS) {
    SS = inSS;
}

Vec2 OpticFlowHomingAlg::currentView( Img* CV ) {

    //blockMatch.correspond(SS, CV, U, V);
    ImgOps::lucasKanade(*SS, *CV, U, V);

    //U->saveGrd("u.grd");
    //V->saveGrd("v.grd");

    if (vectorMap == NULL) {
        int uvWidth = U->getWidth();
        int uvHeight = U->getHeight();
        vectorMap = new ExactVectorMap(uvWidth, uvHeight);
        combiner = new HomeDirCombiner(uvWidth, uvHeight);
        Alpha = new Img(uvWidth, uvHeight);
        Mask = new Img(uvWidth, uvHeight);
        Weight = new Img(uvWidth, uvHeight, 1);
    }

    vectorMap->map(U, V, Mask, Alpha);
    
    double alpha = combiner->combine(Alpha, Weight);

    //double mag = blockMatch.dissimilarity(U, V);
    //cout << "OpticFlowHomingAlg: angle: " << alpha << endl;

    return Vec2(1, alpha);
}
