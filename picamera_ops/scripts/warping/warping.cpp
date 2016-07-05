#include "warping.h"

#include "FloatArrays.h"
#include "Warping.h"
#include <iostream>
using namespace std;

typedef FloatBlockVector WarpImage;

std::vector <Warping> warping_vec;

// Stores the result from 'compute_home_vector'
FloatBlockVector homeVector;

void init(int n) {
    for (int i=0; i<n; i++) {
        Warping w;
        // Modify the default warping parameters (these are the ones I used in
        // my Doctoral experiments).
        w.warpingRhoMax = 0.95;
        w.warpingRhoSteps = 36;
        w.warpingAlphaSteps = 36;
        w.warpingPsiSteps = 36;
        w.warpingMatchMode = 1; // 0 == DOT, 1 == DIFF

        warping_vec.push_back(w);
    }
}

void set_snapshot(int i, std::vector < float > oneDimage) {
    int width = oneDimage.size();
    WarpImage SS(width, 0);
    for (int k=0; k<width; k++)
        SS[k] = oneDimage[k];

    // Set the snapshot image.  This step actually creates the warping table,
    // which may be costly to build.
    warping_vec[i].setSnapshot(SS);
}

void compute_home_vector(int i, std::vector < float > oneDimage) {
    int width = oneDimage.size();
    WarpImage CV(width, 0);
    for (int k=0; k<width; k++)
        CV[k] = oneDimage[k];

    // Calling 'setCurrentView' actually initiates the warping search.
    warping_vec[i].setCurrentView(CV);

    // Get the computed home vector.  
    homeVector = warping_vec[i].matchWarping();
}

float get_home_x() {
    return homeVector[0];
}

float get_home_y() {
    return homeVector[1];
}
