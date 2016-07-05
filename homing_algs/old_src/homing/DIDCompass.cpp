#include <cmath>
#include "DIDCompass.h"

DIDCompass::DIDCompass( int width, int height )
    : SS(NULL),
      Rotated(NULL),
      SqdDiff(NULL)
{
    maxShiftFrac = 0.5f;
    maxShift = (int) ceil( maxShiftFrac*width );

    debug = false;
    if ( debug )
        imgWindow = new ImgWindow("DIDCompass");
}

DIDCompass::~DIDCompass() {
    if ( debug )
        delete imgWindow;
}

void DIDCompass::snapshot(Img* inSS) {
    SS = inSS;
}

double DIDCompass::currentView( Img* CV ) {
// Return the angle that the angle of rotation between the CV and SS by
// finding the rotation of CV with the minimum difference to SS.
//
    float lowestSSD = FLT_MAX;
    int bestShift = 0;
    int shift = -maxShift;
    float ssd;
    for ( ; shift <= maxShift; shift++ ) {
        ImgOps::rotate(CV, shift, Rotated);

        ImgOps::sqdDiff(SS, Rotated, SqdDiff);

        ssd = SqdDiff->getSum();

        if ( ssd < lowestSSD ) {
            lowestSSD = ssd;
            bestShift = shift;
        }

    }
    if ( debug ) {
        ImgOps::rotate(CV, bestShift, Rotated);
        imgWindow->clear();
        imgWindow->addImg("SS", *SS);
        imgWindow->addImg("CV", *CV);
        imgWindow->addImg("Rotated", *Rotated);
        imgWindow->refresh();
    }

    // A negative image rotation (rotation to the left) corresponds
    // to a positive angle.  Hence the negative sign below.
    return -Angles::int2angle(bestShift, CV->getWidth());
}
