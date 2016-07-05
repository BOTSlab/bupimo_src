#include "SiftExtractor.h"

int main( int argc, char *argv[] ) {
    Img imgA(string("/data/db/preproc/original/4_8.pgm"));
    Img imgB(string("/data/db/preproc/original/8_8.pgm"));
    int width = imgA.getWidth();
    int height = imgA.getHeight();
    SiftExtractor extractor(width, height);
    vector<Keypoint*> keysA, keysB;

    int n = 10;
    // Extract keypoints from both images and then match them---n times.
    for (int i=0; i<n; i++) {
        extractor.clearKeys(keysA);
        extractor.clearKeys(keysB);
        extractor.extract(imgA, keysA);
        extractor.extract(imgB, keysB);

        vector<Match> matches;
        extractor.match(keysA, keysB, matches);
    }

    // Free keypoints
    extractor.clearKeys(keysA);
    extractor.clearKeys(keysB);

    return 0;
}
