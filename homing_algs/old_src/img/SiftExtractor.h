/**
 * A filter for extracting SIFT keypoints from an input image and matching
 * pairs of previously extracted vectors of keypoints.  The same extractor
 * object can be re-used for any input image of the same size.
 *
 * Requires: VLFeat
 *
 * @author Andrew Vardy
 */

#ifndef SIFTEXTRACTOR_H
#define SIFTEXTRACTOR_H

#include "Keypoint.h"
#include "Img.h"
extern "C" {
#include <sift.h>
}
#include <vector>

using namespace std;

class Match {
public:
    Match(int inA, int inB) : a(inA), b(inB) {}
    int a, b;
};

class SiftExtractor {
public:
    /**
     * Constructs a SiftExtractor capable of extracting SIFT keypoints from
     * images of the given size.
     */
    SiftExtractor(int inWidth, int inHeight);

    /**
     * Destructor.
     */
    virtual ~SiftExtractor();

    /**
     * Extract keypoints from the given image and place them in the given
     * vector.  The caller is responsible with deallocating the Keypoints.
     */
    void extract(Img &img, vector<Keypoint*> &keypoints);

    /**
     * Find matches between the two vectors of input keypoints and place them
     * in 'matches'
     */
    void match(vector<Keypoint*> &keysA, vector<Keypoint*> &keysB,
               vector<Match> &matches);

    /**
     * Find matches using Lowe's method.
     */
    void ubcMatch(vector<Keypoint*> &keysA, vector<Keypoint*> &keysB,
                  vector<Match> &matches);

    /**
     * Find approximate matches using a fast KD-Forest approach.
     */
    void approxMatch(vector<Keypoint*> &keysA, vector<Keypoint*> &keysB,
                     vector<Match> &matches);

    /**
     * Filter matches between two panoramic images by considering the
     * horizontal component of the matches.  Firstly, each horizontal component
     * is taken as positive.  We then filter out matches with a horizontal
     * component that has a value that is 'stdevs' standard deviations away
     * from the mean value.
     */
    void deviationFilter(vector<Keypoint*> &keysA, vector<Keypoint*> &keysB,
                         vector<Match> &matches, double stdevs);

    /**
     * Compute the strength of matching between each keypoint in keys and all
     * of the others in keys.  The result for each keypoint is returned in
     * distRatios as the distance between the each keypoint and its best
     * match within keys, over the average distance.
     *
     * \pre distRatios is an array with keys.size entries.
     */
    static void matchWithin(vector<Keypoint*> &keys, double *distRatios);

    /**
     * Delete any keys in the given vector and reset the vector to zero size.
     */
    static void clearKeys(vector<Keypoint*> &keys);

    /**
     * Save the given vector of keypoints to a file.
     */
    static void saveKeys(vector<Keypoint*> &keys, string filename);

    /**
     * Load keypoints from the given file into the vector.
     */
    static void loadKeys(string filename, vector<Keypoint*> &keys);

    // The following two are the similarity measures described in my 2010 IROS
    // paper.

    double percentMatched(vector<Keypoint*> &saKeys, vector<Keypoint*> &sbKeys,
                          vector<Match> &matchesA, vector<Match> &matchesB);

    double scaleDiff(vector<Keypoint*> &saKeys, vector<Keypoint*> &sbKeys,
                     vector<Keypoint*> &cKeys,
                     vector<Match> &matchesA, vector<Match> &matchesB);

private:
    // Helper for filterMatches below which determines the smallest horizontal
    // displacement of a match between two keypoints with x-coordinates ax
    // and bx.
    double panoramicDX(double ax, double bx);


    // Dimensions of the image for which this extractor can be applied.
    int width, height;

    bool verbose;

    // The VLFEAT filter.
    VlSiftFilt *filt;

    // Buffers to hold the contents of the most recently extracted descriptor.
    vl_sift_pix floatBuffer[128];
    unsigned char ucharBuffer[128];

    // Parameters.
    bool useApprox;
    double approxCompareFraction;
};

#endif
