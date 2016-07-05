/*
 * Compass algorithm based on simple Descent in Image Distances (DID).
 *
 * Andrew Vardy
 */
#ifndef DIDCOMPASS_H
#define DIDCOMPASS_H

#include "RotHomingAlg.h"
#include "ImgWindow.h"
#include "ImgOps.h"
#include "Angles.h"

class DIDCompass : public RotHomingAlg {
public:
    DIDCompass( int width, int height );
    ~DIDCompass();
    void snapshot(Img* inSS);
    double currentView( Img* CV );

private:
    Img *SS, *Rotated, *SqdDiff;
    float maxShiftFrac;
    int maxShift;
    bool debug;
    ImgWindow *imgWindow;
};

#endif
