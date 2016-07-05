/*
 * Uses a RotHomingAlg for rotation estimation and a TransHomingAlg for
 * translation.
 *
 * Andrew Vardy
 */

#ifndef TWOSTEPHOMINGALG_H
#define TWOSTEPHOMINGALG_H

#include "TotalHomingAlg.h"
#include "RotHomingAlg.h"
#include "TransHomingAlg.h"
#include "ImgWindow.h"
#include "ImgOps.h"
#include "Angles.h"

class TwoStepHomingAlg : public TotalHomingAlg {
public:
    TwoStepHomingAlg( int inWidth, int inHeight,
                      RotHomingAlg *inRotHomingAlg,
                      TransHomingAlg *inTransHomingAlg );
    ~TwoStepHomingAlg();
    virtual void snapshot( Img* inSS );
    virtual void getHomeAngle(Img* CV, double &homeAngle, double &similarity);
private:
    Img *SS;
    int width, height;
    RotHomingAlg *rotHomingAlg;
    TransHomingAlg *transHomingAlg;
    bool debug;
    Img *rotCV;
    ImgWindow *imgWindow;
};

#endif
