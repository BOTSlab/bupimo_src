#include "WarpingAlg.h"

WarpingAlg::WarpingAlg( int inWidth, int inHeight )
    : warping(),
      SS(NULL),
      width(inWidth),
      height(inHeight),
      warpSS(width, 0),
      warpCV(width, 0)
{
    // Modify the default warping parameters.
    warping.warpingRhoMax = 0.95;
    warping.warpingRhoSteps = 36;
    warping.warpingAlphaSteps = 36;
    warping.warpingPsiSteps = 36;
    warping.warpingMatchMode = 1; // 0 == DOT, 1 == DIFF

}

WarpingAlg::~WarpingAlg() {
}

void WarpingAlg::extract( Img *img, WarpImage *warpImage ) {
// Average the middle rows of 'img' to generate 'warpImage'.  The number of
// rows included in this average is 2r + 1.
//    int horizon = height / 2;
int horizon = height / 4;
    int r = 5;
    for (int i=0; i<width; i++) {
    	float sum = 0;
    	for (int j=horizon-r; j<=horizon+r; j++)
    		sum += img->get(i, j);
		(*warpImage)[i] = sum / (2*r + 1);
    }
}

void WarpingAlg::snapshot( Img* inSS ) {
    SS = inSS;

    cout << "WarpingAlg: Processing snapshot..." << endl;

    extract(SS, &warpSS);

    // Set the snapshot image.  This step actually creates the warping table,
    // which may be costly to build.
    warping.setSnapshot(warpSS);
    cout << "WarpingAlg: Done" << endl;
}

void WarpingAlg::getHomeAngle(Img* CV, double &homeAngle, double &similarity) {
//Vec2 WarpingAlg::currentView( Img* CV ) {
    cout << "WarpingAlg: Processing current image..." << endl;

    extract(CV, &warpCV);

    // Calling 'setCurrentView' actually initiates the warping search.
    warping.setCurrentView(warpCV);

    // Get the computed home vector.
    FloatBlockVector homeVector = warping.matchWarping();

    // The vector returned by warping appears to be the direction from snapshot
    // to the current position.  Thus, we need to reverse it to obtain the home
    // vector from the current back to the snapshot position.
    double vx = homeVector[0];
    double vy = homeVector[1];

    // This home vector is scaled by the value of rho for which the match
    // between SS and CV is minimized.  Obtain the components of the home vector
    // in polar coordinates.
/*
cout << "choice 1: " << atan2(vy, vx) << endl;
cout << "choice 2: " << Angles::constrainAngle(atan2(vy, vx) - M_PI) << endl;
cout << "choice 3: " << Angles::constrainAngle(atan2(vy, vx) - M_PI/2.0) << endl;
cout << "choice 4: " << Angles::constrainAngle(atan2(vy, vx) + M_PI/2.0) << endl;
cout << "choice 5: " << atan2(vx, vy) << endl;
cout << "choice 6: " << Angles::constrainAngle(atan2(vx, vy) - M_PI) << endl;
cout << "choice 7: " << Angles::constrainAngle(atan2(vx, vy) - M_PI/2.0) << endl;
cout << "choice 8: " << Angles::constrainAngle(atan2(vx, vy) + M_PI/2.0) << endl << endl;
*/

// Choice 7 for Imgdb (Ralf's image format)
homeAngle = Angles::constrainAngle(atan2(vx, vy) - M_PI/2.0);
similarity = sqrt(vx*vx + vy*vy);

    cout << "WarpingAlg: Done" << endl;
}
