#include <cmath>
#include <sstream>
#include <iomanip>
#include "WeightedDIDCompass.h"
#include "rng.h"

WeightedDIDCompass::WeightedDIDCompass( int inWidth, int inHeight, 
                                        int inLearnStart, int inLearnStop,
                                        WeightBasis inWeightBasis,
                                        WeightComb inWeightComb,
                                        int inSmoothRadius, double inSigma,
                                        bool inDebug ) : 
    width(inWidth), 
    height(inHeight), 
    learnStart(inLearnStart),
    learnStop(inLearnStop),
    learnIndex(0),
    weightBasis(inWeightBasis),
    weightComb(inWeightComb),
    smoothRadius(inSmoothRadius),
    sigma(inSigma),
    Rotated(width, height),
    SqdDiff(width, height),
    SqdDiffSmoothed(width, height),
    MinShift(width, height),
    MinShiftValue(width, height),
    InstWeight(width, height),
    Weight(width, height),
    FinalWeight(width, height),
    useBinaryWeights(true),
    useRandomWeights(true),
    debug(inDebug)
{
    if ( debug )
        imgWindow = new ImgWindow("WeightedDIDCompass");
}

WeightedDIDCompass::~WeightedDIDCompass() {
    if ( debug )
        delete imgWindow;
}

double WeightedDIDCompass::currentView( Img* CV ) {
    if ( learnMode == LEARNING ) {
        if ( learnIndex >= learnStart && learnIndex <= learnStop )
            learn(SS, CV);
        learnIndex++;
        return computeAngle(SS, CV, false);
    } else if ( learnMode == NO_LEARNING )
        return computeAngle(SS, CV, false);
    else if ( learnMode == USING )
        return computeAngle(SS, CV, true);
}

void WeightedDIDCompass::learn( Img *SS, Img *CV ) {

    if ( learnIndex == learnStart ) {
        switch (weightComb) {
            case MULTIPLY:
                Weight.setAll(1);
                break;
            case ADD:
                Weight.setAll(0);
                break;
            case AND:
                Weight.setAll(1);
                break;
            case OR:
                Weight.setAll(0);
                break;
        };
    }

    computeMinShift(SS, CV);

    // We now determine whether the minimum shift computed for each pixel is
    // close enough to the shift determined by odometry
    InstWeight.setAll(0);
    int i = 0;
    for ( int y=0; y<height; y++ )
        for ( int x=0; x<width; x++ ) {
            float angle = Angles::int2angle((int) MinShift.get(x,y), width);
            float diff = Angles::getAngularDifference(angle, odoTh);

            switch (weightBasis) {
                case (DENSITY):
                    InstWeight.set(x, y, probDensity(diff));
                    break;
                case (DISCRETE):
                    if (diff < sigma) InstWeight.set(x, y, 1);
                    break;
            };
        }

    // The instantaneous weight image for the current position is now
    // established, apply one of several different strategies for combining it
    // with previous weights.
    switch (weightComb) {
        case MULTIPLY:
            ImgOps::mult(&InstWeight, &Weight, &Weight);
            break;
        case ADD:
            ImgOps::add(&InstWeight, &Weight, &Weight);
            break;
        case AND:
            for ( int y=0; y<height; y++ )
                for ( int x=0; x<width; x++ )
                    if (InstWeight.get(x,y) == 0)
                        Weight.set(x,y,0);
            break;
        case OR:
            for ( int y=0; y<height; y++ )
                for ( int x=0; x<width; x++ )
                    if (InstWeight.get(x,y) == 1)
                        Weight.set(x,y,1);
            break;
    };

    computeFinalWeight();
}

void WeightedDIDCompass::computeFinalWeight() {
    if (useBinaryWeights) {
        if (useRandomWeights) {
            int n = 289; // Set 289 random pixels to 1
            RNG rng(time(0));
            FinalWeight.setAll(0);
            for (int i=0; i<n; i++) {
                // Select a random pixel location with uniform
                // probability.
                int x = (int) rng.uniform(0, Weight.getWidth());
                int y = (int) rng.uniform(0, Weight.getHeight());
                FinalWeight.set(x, y, 1);
            }
        } else { 
            float threshold = Weight.getSum() /
                              (Weight.getWidth() * Weight.getHeight());
            ImgOps::threshold(&Weight, threshold, &FinalWeight);
        }
    } else
        FinalWeight = Weight;
}

float WeightedDIDCompass::computeAngle( Img *SS, Img *CV, bool useWeight ) {
// Return the angle that the angle of rotation between the CV and SS by
// finding the rotation of CV with the minimum difference to SS.

    float lowestSSD = FLT_MAX;
    int bestShift = 0;
    for ( int shift=0; shift < width; shift++ ) {
        ImgOps::rotate(CV, shift, &Rotated);
        ImgOps::sqdDiff(SS, &Rotated, &SqdDiff);

        if (smoothRadius > 0)
            ImgOps::smooth(&SqdDiff, &SqdDiffSmoothed, smoothRadius);
        else
            SqdDiffSmoothed = SqdDiff;

        if (useWeight)
            ImgOps::mult(&SqdDiffSmoothed, &FinalWeight, &SqdDiffSmoothed);

        float ssd = SqdDiffSmoothed.getSum();
        if ( ssd < lowestSSD ) {
            lowestSSD = ssd;
            bestShift = shift;
        }
    }

    //if ( debug && learnIndex >= learnStart && learnIndex <= learnStop ) {
    if ( debug ) {
        Img* Temp1 = new Img(width, height);
        Img* Temp2 = new Img(width, height);
        float threshold = FinalWeight.getMax() / 4.0;
        Temp2 = ImgOps::threshold(&FinalWeight, threshold, Temp1);

        imgWindow->clear();
        imgWindow->addImg("SS", *SS);
        imgWindow->addImg("InsWeight", InstWeight);
        imgWindow->addImg("FinalWeight", FinalWeight);
        imgWindow->addImg("Thresholded Weight", *Temp2);
        imgWindow->refresh();

        Img* Overlay = new Img(width, height);
        for (int x=0; x<width; x++) {
            for (int y=0; y<height; y++) {
                if ( Temp2->get(x, y) == 1 )
                    Overlay->set(x, y, 1);
                else
                    Overlay->set(x, y, SS->get(x,y));
                }
            }
ostringstream oss;
oss << setfill('0') << setw(2);
oss << learnIndex;
oss << ".png";

Overlay->save("overlay" + oss.str());
FinalWeight.save("weight" + oss.str());
Temp2->save("thresholded" + oss.str());
    }

    // A negative image rotation (rotation to the left) corresponds
    // to a positive angle.  Hence the negative sign below.
    return -Angles::int2angle(bestShift, CV->getWidth());
}

float WeightedDIDCompass::probDensity( float x ) {
    float var = sigma*sigma;
    return (float) ( exp(-0.5*x*x/var) / 
                     sqrt(2*M_PI*var) );
}

void WeightedDIDCompass::computeMinShift( Img *SS, Img *CV ) {
    MinShift.setAll(0);
    MinShiftValue.setAll(FLT_MAX);

    for ( int shift=0; shift < width; shift++ ) {
        if ( shift != 0 ) {
            ImgOps::rotate(CV, shift, &Rotated);
            ImgOps::sqdDiff(SS, &Rotated, &SqdDiff);

            if (smoothRadius > 0)
                ImgOps::smooth(&SqdDiff, &SqdDiffSmoothed, smoothRadius);
            else
                SqdDiffSmoothed = SqdDiff;

            // Comparing each pixel of SqdDiff to the minimum value found so far
            // at that pixel location (stored in MinShiftValue).  If the new
            // value is smaller, set the current level of shift at the
            // corresponding pixel location in MinShift.
            for ( int y=0; y<height; y++ )
                for ( int x=0; x<width; x++ )
                    if (SqdDiffSmoothed.get(x,y) < MinShiftValue.get(x,y)) {
                        MinShiftValue.set(x, y, SqdDiffSmoothed.get(x,y));
                        MinShift.set(x, y, shift);
                    }
        }
    }
}
