#include "WeightedBM.h"
#include <sstream>

WeightedBM::WeightedBM( int inWidth, int inHeight, 
                        int inLearnSteps, bool inUseProduct, bool inDebug ) : 
    width(inWidth), 
    height(inHeight), 
    learnSteps(inLearnSteps),
    useProduct(inUseProduct),
    U(width, height),
    V(width, height),
    Mask(width, height),
    Corr(width, height),
    CorrVecs(NULL),
    Alpha(width, height),
    Weight(width, height),
    Diff(width, height),
    Prob(width, height),
    Product(width, height),
    FinalWeight(width, height),
    blockMatch(width, height),
    vectorMap(width, height),
    combiner(width, height),
    firstLearn(true),
    learnIndex(0),
    debug(inDebug)
{
    if ( debug )
        imgWindow = new ImgWindow("WeightedBM");
}


WeightedBM::~WeightedBM() {
    if ( debug )
        delete imgWindow;
}

float WeightedBM::probDensity( float x ) {
    return (float) ( exp(-0.5*x*x/sigmaOdoAlpha) / 
                     sqrt(2*M_PI*sigmaOdoAlpha) );
}

Vec2 WeightedBM::currentView( Img* CV ) {

    blockMatch.correspond(SS, CV, &U, &V);
    vectorMap.map(&U, &V, &Mask, &Alpha);

    if ( debug ) {
        imgWindow->clear();
        ImgOps::atan2(&V, &U, &Corr);
        CorrVecs = ImgOps::drawVectorImage(&U, &V, 1, 5, 3, true, CorrVecs);
    }

    if ( learnMode == LEARNING && learnIndex++ < learnSteps )
        learn();

    // Set the Weight image
    if ( learnMode == LEARNING || learnMode == NO_LEARNING || 
         sigmaOdoAlpha == 0 )
        Weight.setAll(1);
    else if ( learnMode == USING )
        Weight = Product;

    if ( debug ) {
        imgWindow->addImg("Weight", Weight);
        imgWindow->refresh();
    }

    // Compute the home direction (finalAlpha)
    ImgOps::mult(&Mask, &Weight, &FinalWeight);
    float finalAlpha = combiner.combine(&Alpha, &FinalWeight);

    return Vec2(1, finalAlpha);
}

void WeightedBM::learn() {
    // Estimate the probability of each estimated home direction
    // (alpha---estimated from each matched pair of blocks).
    Diff.setAll(0);
    Prob.setAll(0);
    for ( int y=0; y<height; y++ ) {
        for ( int x=0; x<width; x++ ) {
            if ( Mask.get(x,y) == 0 )
                continue;

            float alpha = Alpha.get(x,y);
            float diff = Angles::getAngularDifference(alpha, odoAlpha);
            float prob = probDensity(diff);
            Prob.set(x, y, prob);

            // For debug
            Diff.set(x, y, diff);
        }
    }

    if ( firstLearn ) {
        Product = Prob;
        firstLearn = false;
    } else {
        if ( useProduct )
            ImgOps::mult(&Product, &Prob, &Product);
        else
            ImgOps::add(&Product, &Prob, &Product);
    }

    if ( debug ) {
        imgWindow->addImg("CorrVecs", *CorrVecs);
        imgWindow->addImg("Prob", Prob);
        imgWindow->addImg("Product", Product);
    }
}
