/**
 * The functions here serve as a bridge for a Python module that connects to
 * Ralf Moeller's warping code using SWIG.  The algorithm implemented comes from
 * the following paper:
 *
 *  Where did I take that snapshot? Scene-based homing by image matching
 *  Journal Biological Cybernetics
 *  Publisher   Springer Berlin / Heidelberg
 *  ISSN    0340-1200 (Print) 1432-0770 (Online)
 *  Issue   Volume 79, Number 3 / October, 1998
 *
 * @author Andrew Vardy
 */

#include "FloatArrays.h"
#include "Warping.h"
#include "Img.h"
#include <iostream>
using namespace std;

typedef FloatBlockVector WarpImage;

/**
 * Initialize the module (Python terminology).
 */
int init_moeller_warping() {
    // Instantiate the warping object.
    Warping warping;

    // Modify the default warping parameters (these are the ones I used in my
    // Doctoral experiments.
    warping.warpingRhoMax = 0.95;
    warping.warpingRhoSteps = 36;
    warping.warpingAlphaSteps = 36;
    warping.warpingPsiSteps = 36;
    warping.warpingMatchMode = 1; // 0 == DOT, 1 == DIFF
}

/**
 * Set the current snapshot image
 */
void set_snapshot() {
    // Load image files in current directory.
    Img imgSS("cv_5_8_bw0.40.pgm");
    Img imgCV("cv_7_8_bw0.40.pgm");

    // Extract the middle row of these images to apply warping.  (Other
    // strategies should probably be investigated).
    int width = imgSS.getWidth();
    int height = imgSS.getHeight();
    WarpImage SS(width, 0);
    WarpImage CV(width, 0);
    int horizon = height / 2;
    for (int i=0; i<width; i++) {
        SS[i] = imgSS.get(i, horizon);
        CV[i] = imgCV.get(i, horizon);
    }

    // Set the snapshot image.  This step actually creates the warping table,
    // which may be costly to build.
    warping.setSnapshot(SS);

    // Calling 'setCurrentView' actually initiates the warping search.
    warping.setCurrentView(CV);

    // Get the computed home vector.  
    FloatBlockVector homeVector = warping.matchWarping();

    // The vector returned by warping appears to be the direction from snapshot
    // to the current position.  Thus, we need to reverse it to obtain the home
    // vector from the current back to the snapshot position.
    homeVector[0] *= -1;
    homeVector[1] *= -1;
    cout << "homeVector: " << homeVector[0] << ", " << homeVector[1] << endl;

    // This home vector is scaled by the value of rho for which the match
    // between SS and CV is minimized.  If we only care about the angle, we can
    // extract it as follows.
    float alpha = atan2(homeVector[1], homeVector[0]);
    cout << "alpha: " << alpha << endl;

    return 0;
}
