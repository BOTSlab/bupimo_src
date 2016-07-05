#include "TwoStepHomingAlg.h"

TwoStepHomingAlg::TwoStepHomingAlg( int inWidth, int inHeight,
                                    RotHomingAlg *inRotHomingAlg,
                                    TransHomingAlg *inTransHomingAlg )
    : SS(NULL),
      width(inWidth),
      height(inHeight),
      rotHomingAlg(inRotHomingAlg),
      transHomingAlg(inTransHomingAlg),
      debug(true),
      rotCV(NULL)
{
    if (debug)
        imgWindow = new ImgWindow("TwoStepHomingAlg");
}

TwoStepHomingAlg::~TwoStepHomingAlg() {
    if (debug)
        delete imgWindow;
    delete rotCV;
    delete rotHomingAlg;
    delete transHomingAlg;
}

void TwoStepHomingAlg::snapshot( Img* inSS ) {
    SS = inSS;

    if (debug) {
        imgWindow->clear();
        imgWindow->addImg("SS", *SS);
        imgWindow->refresh();
    }

    transHomingAlg->snapshot(SS);
    rotHomingAlg->snapshot(SS);
}

void TwoStepHomingAlg::getHomeAngle(Img* CV, double &homeAngle, double &similarity) {

    // Determine the amount of image rotation so as to align the CV with the SS.
    double rot = rotHomingAlg->currentView(CV);

    cout << "\tTwoStepHomingAlg: rotation: " << rot << endl;
    int colrot = Angles::angle2int(-rot, CV->getWidth());
    ImgOps::rotate(CV, colrot, rotCV);

    // Compute a vector pointing (hopefully) in the home direction.  Note that
    // this vector is in robot coordinates.
    Vec2 homeVec = transHomingAlg->currentView(rotCV);
    homeAngle = homeVec.ang - rot;
    similarity = 0;

    if (debug) {
        imgWindow->clear();
        imgWindow->addImg("SS", *SS);
        imgWindow->addImg("CV", *CV);
        imgWindow->addImg("Rotated CV", *rotCV);
        imgWindow->refresh();
    }
}
