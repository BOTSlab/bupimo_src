/**
 * Go through an ImgSequence or Imgdb and extract keypoints for all images.
 *
 * @author Andrew Vardy
 */

#include "ImgSequence.h"
#include "Imgdb.h"
#include "SiftExtractor.h"
#include <sstream>
#include <iomanip>

/**
 * Compute and save the keypoints for the ImgSequence at the given directory.
 */
void computeSeqKeys(string seqDir) {

    ImgSequence imgSequence(seqDir);

    // Create extractor and extract keypoints for all images in the sequence.
    SiftExtractor extractor(imgSequence.getImageWidth(),
                            imgSequence.getImageHeight());

    for (int i=0; i<imgSequence.getLength(); i++) {
        vector<Keypoint*> keys;
        extractor.extract(*(imgSequence.getImg(i)), keys);

        // Save the keys to a file.
        ostringstream oss;
        oss << seqDir << "/";
        oss << setfill('0') << setw(3);
        oss << i;
        oss << ".keys";
        SiftExtractor::saveKeys(keys, oss.str());
    }
}

/**
 * Compute and save the keypoints for the Imgdb at the given directory.
 */
void computeDBKeys(string dbDir, int width, int height) {

    Imgdb imgdb(dbDir, width, height);

    // Create extractor and extract keypoints for all images in the sequence.
    SiftExtractor extractor(imgdb.getImageWidth(), imgdb.getImageHeight());

    for (int x=0; x<width; x++) {
        for (int y=0; y<height; y++) {
            vector<Keypoint*> keys;
            extractor.extract(*(imgdb.getImg(x, y)), keys);

            // Save the keys to a file.
            ostringstream oss;
            oss << dbDir << "/" << x << "_" << y << ".keys";
            SiftExtractor::saveKeys(keys, oss.str());
        }
    }
}

int main(int argc, char ** argv) {
    //computeSeqKeys("/data/seq/inco4");
    computeDBKeys("/data/db/preproc/original", 10, 17);
    return 0;
}
