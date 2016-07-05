/**
 * Reads an image sequence an applies different methods to filter out features
 * and then save the output sequence of feature files (000.keys, 001.keys,...).
 *
 * @author Andrew Vardy
 */

#ifndef FEATUREFILTER_H
#define FEATUREFILTER_H

#include "ImgSequence.h"
#include "SiftExtractor.h"

class FeatureFilter {
public:
    FeatureFilter(string inSequenceName);

    virtual ~FeatureFilter();

    /**
     * Simply extracts and saves keypoints without applying any filtering.
     */
    void nullFilter();

    /**
     * Filters out keys from the bottom of the image.
     */
    void floorFilter();

    /**
     * Matches pairs of adjacent snapshots and filters out keys which move
     * (shift) excessively from one image to the other.  The shift is calculated
     * after counter-rotating one image to match the other.
     */
    void shiftFilter();

    /**
     * Attempts to match keypoints within the same image they were extracted
     * from.  Only those entries without matches are retained.
     */
    void matchFilter();

    /**
     * Pairs each image with neighbouring images and computes home angles for
     * each feature using the HISS method applied on an individual feature
     * basis.  Retains only those features whose angular error (w.r.t. odometry)
     * falls below a threshold.
     */
    void angleFilter(int maxSeparation, double aeThreshold);
private:
    void accumulateAngularError(int ssIndex, int cvIndex,
                                double *aae, int *used);

    // The input sequence and the number of images it contains.
    string imgSequenceDir;
    ImgSequence imgSequence;
    int n;

    SiftExtractor *extractor;

    // An array of vectors of features, one for each image in the input
    // sequence.
    vector<Keypoint*> *keysArray;

    int imgWidth, imgHeight;
};

#endif
