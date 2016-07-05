/**
 * Go through a stored ImgSequence and extract and filter its keypoints.
 *
 * @author Andrew Vardy
 */

#include "FeatureFilter.h"

int main( int argc, char ** argv ) {
    FeatureFilter filter("/data/seq/gallery2_1");
    //filter.angleFilter(1, 0.25);
    filter.nullFilter();
    return 0;
}
