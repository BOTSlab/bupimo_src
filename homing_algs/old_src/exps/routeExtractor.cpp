/**
 * Go through a stored ImgSequence and extract a sparse route.
 *
 * @author Andrew Vardy
 */

#include "RouteExtractor.h"

int main( int argc, char ** argv ) {
    RouteExtractor extractor("/data/seq/gallery2_1", "/data/seq/gallery2_1X",
                             0.2618, true); // 15 degrees
    extractor.extract();
    return 0;
}
