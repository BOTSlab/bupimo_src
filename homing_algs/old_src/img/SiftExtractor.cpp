#include "SiftExtractor.h"
#include "SingleConfig.h"
#include <fstream>
#include <cassert>
#include <limits.h>
#include <glog/logging.h>

// For the k-d forest stuff
extern "C" {
    #include <kdtree.h>
}

SiftExtractor::SiftExtractor(int inWidth, int inHeight) :
    width(inWidth), height(inHeight), verbose(false)
{
    if (verbose)
        cout << "SiftExtractor: Constructing with width: " << width
             << ", height: " << height << endl;
    
    SingleConfig *config = SingleConfig::getInstance(verbose);

    // Create a SIFT filter
    filt = vl_sift_new(width, height, 
                       config->getInt("SiftExtractor.numberOfOctaves", -1),
                       config->getInt("SiftExtractor.levelsPerOctave", 3),
                       config->getInt("SiftExtractor.firstOctaveIndex", -1));

    // Modify the filter's parameters.
    vl_sift_set_peak_thresh(filt,
                            config->getDouble("SiftExtractor.peakThresh", 0));
    vl_sift_set_edge_thresh(filt,
                            config->getDouble("SiftExtractor.edgeThresh", 10));
    vl_sift_set_window_size(filt,
                            config->getDouble("SiftExtractor.windowSize", 2));
    vl_sift_set_magnif(filt,
                       config->getDouble("SiftExtractor.magnifFactor", 3));

    // Matching parameters.
    useApprox = config->getBool("SiftExtractor.useApprox", false);
    approxCompareFraction = config->getDouble(
                                "SiftExtractor.approxCompareFraction", 0);
}

SiftExtractor::~SiftExtractor() {
    vl_sift_delete(filt);
}

void SiftExtractor::extract(Img &img, vector<Keypoint*> &keypoints) {
    LOG(INFO) << "extract START";

    assert(img.getWidth() == width);
    assert(img.getHeight() == height);

    //
    // Loop over all octaves.
    //
    bool first = true;
    while (true) {
        int err, nkeys = 0;
        VlSiftKeypoint const *keys  = 0;

        if (verbose)
            cout << "SiftExtractor: processing octave "
                 << vl_sift_get_octave_index(filt) << endl;

        // Process the next octave.
        if (first) {
            err = vl_sift_process_first_octave(filt, img.getData());
            first = false;
        } else {
            err = vl_sift_process_next_octave(filt);
        }        

        if (err)
            break;

        // Now detect keypoints
        vl_sift_detect(filt);
        keys = vl_sift_get_keypoints(filt);
        nkeys = vl_sift_get_nkeypoints(filt);

        if (verbose)
            cout << "SiftExtractor: detected " << nkeys
                 << " (unoriented) keypoints" << endl;

        // Process each keypoint to obtain its orientation.  Then for each
        // orientation we will calculate a descriptor.  VLFeat's notion of a
        // keypoint is thus slightly different from ours since a VLFeat
        // keypoint can generate up to four descriptors and therefore four
        // 'Keypoint's.
        for (int i=0; i < nkeys ; i++) {
            double angles[4];
            int nangles;
            VlSiftKeypoint const *k = keys + i;

            // Obtain keypoint orientations
            nangles = vl_sift_calc_keypoint_orientations(filt, angles, k);            
            // For each orientation...
            for (int q = 0; q < nangles ; q++) {
                // Compute descriptor
                vl_sift_calc_keypoint_descriptor(filt, floatBuffer, k,
                                                 angles[q]);

                Keypoint *key = new Keypoint(k->x, k->y, k->sigma,
                                            angles[q]);

                // Turn the keypoint descriptor into an array of integers
                // in the range [0 - 255].
                for (int j = 0 ; j < 128 ; j++) {
                    double x = 512.0 * floatBuffer[j];
                    x = (x < 255.0) ? x : 255.0 ;
                    ucharBuffer[j] = (unsigned char) x;
                }

                // Copy the descriptor into the Keypoint and add it to
                // our vector.
                key->copyDescrip(ucharBuffer);
                keypoints.push_back(key);
            }
        }
    }

    if (verbose)
        cout << "SiftExtractor: found " << keypoints.size()
             << " keypoints" << endl;

    LOG(INFO) << "extract END";
}

void SiftExtractor::match(vector<Keypoint*> &keysA,
                          vector<Keypoint*> &keysB,
                          vector<Match> &matches) {
    if (useApprox)
        approxMatch(keysA, keysB, matches);
    else
        ubcMatch(keysA, keysB, matches);
}

void SiftExtractor::ubcMatch(vector<Keypoint*> &keysA,
                             vector<Keypoint*> &keysB,
                             vector<Match> &matches) {
    LOG(INFO) << "ubcMatch START";


    int nA = keysA.size();
    int nB = keysB.size();

    // Find a match for each keypoint in keysA
    for (int a=0; a<nA; a++) {
        Keypoint *keyA = keysA[a];
        
        int closest = INT_MAX;
        int secondClosest = INT_MAX;
        int closestB = 0;
        for (int b=0; b<nB; b++) {
            // Compute the sum of squared differences between the descriptors
            int ssd = 0;
            for (int i=0; i<128; i++) {
                int diff = keyA->descrip[i] - keysB[b]->descrip[i];
                ssd += diff*diff;
            }

            // Determine if the distance measure between these two keypoints
            // is the closest (smallest) or second closest found thus far.
            if (ssd < closest) {
                secondClosest = closest;
                closest = ssd;
                closestB = b;
            } else if (ssd < secondClosest) {
                secondClosest = ssd;
            }
        }

        // Utilize Lowe's criteria that the ratio of the closest distance
        // to the second should be less than 0.8.  Since we are dealing with
        // sum-of-squared differences, the threshold becomes 0.64.  Instead
        // of carrying out the division we actually cross-multiply for speed.
        if (1.5625 * closest < secondClosest) {
            Keypoint *keyB = keysB[closestB];
            keyA->match = keyB;
            keyB->match = keyA;
            matches.push_back(Match(a, closestB));
        }
    }

    LOG(INFO) << "ubcMatch END";
}

void SiftExtractor::approxMatch(vector<Keypoint*> &keysA,
                                vector<Keypoint*> &keysB,
                                vector<Match> &matches) {
    LOG(INFO) << "approxMatch START";

    int nA = keysA.size();
    int nB = keysB.size();

    // Create a temporary array holding all of the descriptor vectors from
    // keysA, stacked end-to-end.  The type of this array is float to match
    // with the vlfeat code below.
    float *dataA = new float[128*nA];
    for (int a=0; a<nA; a++) {
        Keypoint *keyA = keysA[a];
        for (int i=0; i<128; i++)
            dataA[a*128 + i] = keyA->descrip[i];
    }

    // Similarly, create a float array to hold the query descriptor (from B).
    float *dataB = new float[128];

    // Create and k-d forest.
    VlKDForest *forest = vl_kdforest_new(VL_TYPE_FLOAT, // Type of data
                                         128, // Dimensionality of descriptors
                                         4,  // Number of trees in forest
                                         VlDistanceL1);
    vl_kdforest_build(forest, nA, dataA);

    // Set some parameters.
    vl_kdforest_set_max_num_comparisons(forest, 
                                        (int) (nA * approxCompareFraction));

    VlKDForestNeighbor neighbours[2];
    for (int b=0; b<nB; b++) {
        Keypoint *keyB = keysB[b];
        for (int i=0; i<128; i++)
            dataB[i] = keyB->descrip[i];

        vl_kdforest_query(forest, neighbours, 2, dataB);

        // Utilize Lowe's criteria that the ratio of the closest distance
        // to the second should be less than 0.8.  Since we are dealing with
        // sum-of-squared differences, the threshold becomes 0.64.  Instead
        // of carrying out the division we actually cross-multiply for speed.
        if (1.5625 * neighbours[0].distance < neighbours[1].distance) {
            Keypoint *keyA = keysA[neighbours[0].index];
            keyA->match = keyB;
            keyB->match = keyA;
            matches.push_back(Match(neighbours[0].index, b));
        } 
    }

    // Delete the forest.
    vl_kdforest_delete(forest);

    delete [] dataA;
    delete [] dataB;

    LOG(INFO) << "approxMatch END";
}

double SiftExtractor::panoramicDX(double ax, double bx) {
    if (ax == bx)
        return 0;
    else if (bx > ax) {
        double dx_left = bx - ax; // +ve
        double dx_right = width - bx + ax; // +ve
        if (dx_left < dx_right)
            return dx_left;
        else
            return dx_right;
    } else {
        double dx_left = ax - bx; // +ve
        double dx_right = width - ax + bx; // +ve
        if (dx_left < dx_right)
            return dx_left;
        else
            return dx_right;
    }
}

void SiftExtractor::deviationFilter(vector<Keypoint*> &keysA,
                                    vector<Keypoint*> &keysB,
                                    vector<Match> &matches, double stdevs) {
    // Consider the horizontal component of each match vector.  Find the
    // average value of this quantity.  Then go through an remove all matches
    // whose horizontal component is greater than 'stdevs' standard deviations
    // from the mean.
    
    int n = matches.size();
    double sumDX = 0;
    double sumSquaredDX = 0;
    for (int m=0; m<matches.size(); m++) {
        Keypoint *keyA = keysA[matches[m].a];
        Keypoint *keyB = keysB[matches[m].b];
        double dx = panoramicDX(keyA->x, keyB->x);
        
        sumDX += dx;
        sumSquaredDX += dx*dx;
    }


    // The threshold for an acceptable deviation.
    double allowableDeviation = sqrt((sumSquaredDX - sumDX*sumDX/n)/n) * stdevs;
cout << "allowableDeviation: " << allowableDeviation << endl;
    int threshold = max((int)(0.01 * width), (int) allowableDeviation);
    sumDX /= n;

    vector<Match>::iterator it = matches.begin();    
    for (; it != matches.end();) {
        Keypoint *keyA = keysA[it->a];
        Keypoint *keyB = keysB[it->b];
        double dx = panoramicDX(keyA->x, keyB->x);

        if (fabs(dx - sumDX) > threshold)
            matches.erase(it);
        else
            it++;
    }
}

void SiftExtractor::matchWithin(vector<Keypoint*> &keys, double *distRatios) {

    // Find a match for each keypoint
    int n = keys.size();
    for (int a=0; a<n; a++) {
        Keypoint *keyA = keys[a];
        
        double smallestDist = DBL_MAX;
        double averageDist = 0;
        for (int b=0; b<n; b++) {
            if (a == b)
                continue;

            // Compute the sum of squared differences between the descriptors
            double dist = 0;
            for (int i=0; i<128; i++) {
                int diff = keyA->descrip[i] - keys[b]->descrip[i];
                dist += diff*diff;
            }
            dist = sqrt(dist);
            averageDist += dist;
            if (dist < smallestDist) smallestDist = dist;
        }
        averageDist /= (n-1);
        distRatios[a] = smallestDist / averageDist;
    }
}

void SiftExtractor::clearKeys(vector<Keypoint*> &keys) {
    // Clear out any old features.
    int nOld = keys.size();
    if (nOld > 0) {
        for (int i=0; i<nOld; i++)
            delete keys[i];
        keys.clear();
    }
}

void SiftExtractor::saveKeys(vector<Keypoint*> &keys, string filename) {
    ofstream keysFile(filename.c_str());

    // First write the total number of keypoints and the size of each
    // descriptor 128.
    keysFile << keys.size() << " 128" << endl;

    // Each keypoint will be written as a group of 8 lines.
    for (int k=0; k<keys.size(); k++) {

        Keypoint *key = keys[k];

        // The first line for each keypoint consists of the keypoint's x, y,
        // sigma, and angle separated by spaces.
        keysFile << key->x << " " << key->y << " " << key->sigma << " "
                 << key->angle << endl;

        // The rest of the descriptor will be written on subsequent lines as a
        // list of 128 integers in the range [0, 255].  We will write 20
        // entries per line (therefore requiring 7 lines per descriptor).
        int descripIndex = 0;
        for (int l=0; l<7; l++) {
            for (int e=0; e<20 && descripIndex < 128; e++) {
                keysFile << " " << (int) key->descrip[descripIndex];
                descripIndex++;
            }
            keysFile << endl;
        }

    }
}

void SiftExtractor::loadKeys(string filename, vector<Keypoint*> &keys) {

//cout << "Loading from " << filename << endl;
    ifstream keysFile(filename.c_str());

    int n, descripLength;
    keysFile >> n;
    keysFile >> descripLength;
    assert(descripLength == 128);

    for (int k=0; k<n; k++) {
        double x, y, sigma, angle;
        keysFile >> x;
        keysFile >> y;
        keysFile >> sigma;
        keysFile >> angle;

        Keypoint *key = new Keypoint(x, y, sigma, angle);

        for (int d=0; d<128; d++) {
            int value;
            keysFile >> value;
            key->descrip[d] = (unsigned char) value;
        }
        
        keys.push_back(key);
    }
}

double SiftExtractor::percentMatched(vector<Keypoint*> &saKeys,
                                     vector<Keypoint*> &sbKeys,
                                     vector<Match> &matchesA,
                                     vector<Match> &matchesB)
{
    return 0.5*( matchesA.size() / (double) saKeys.size() +
                 matchesB.size() / (double) sbKeys.size() );
}

double SiftExtractor::scaleDiff(vector<Keypoint*> &saKeys,
                                vector<Keypoint*> &sbKeys,
                                vector<Keypoint*> &cKeys,
                                vector<Match> &matchesA,
                                vector<Match> &matchesB)
{
    double frontUpperLimit = width/4.0;
    double frontLowerLimit = 3*width/4.0;

    double backLowerLimit = width/4.0;
    double backUpperLimit = 3*width/4.0;

    double scaleDiffThreshold = 0;

    // First determine the number of snapshot features in the front or back.
    int nFrontA = 0, nBackA = 0,
        nFrontB = 0, nBackB = 0;
    for (int k=0; k<saKeys.size(); k++) {
        Keypoint *key = saKeys[k];
        if (key->x < frontUpperLimit || key->x > frontLowerLimit)
            nFrontA++;
        else if (key->x > backLowerLimit && key->x < backUpperLimit)
            nBackA++;
    }
    for (int k=0; k<sbKeys.size(); k++) {
        Keypoint *key = sbKeys[k];
        if (key->x < frontUpperLimit || key->x > frontLowerLimit)
            nFrontB++;
        else if (key->x > backLowerLimit && key->x < backUpperLimit)
            nBackB++;
    }

    int nFrontExpA = 0, nBackConA = 0,
        nFrontConB = 0, nBackExpB = 0;
    for (int m=0; m<matchesA.size(); m++) {
        Keypoint *ssKey = saKeys[matchesA[m].a];
        Keypoint *cvKey = cKeys[matchesA[m].b];
        if (fabs(cvKey->sigma - ssKey->sigma) < scaleDiffThreshold)
            continue;

        if ((ssKey->x < frontUpperLimit || ssKey->x > frontLowerLimit) &&
                cvKey->sigma >= ssKey->sigma)
            nFrontExpA++;

        else if ((ssKey->x > backLowerLimit && ssKey->x < backUpperLimit) &&
                 cvKey->sigma <= ssKey->sigma)
            nBackConA++;
    }
    for (int m=0; m<matchesB.size(); m++) {
        Keypoint *ssKey = sbKeys[matchesB[m].a];
        Keypoint *cvKey = cKeys[matchesB[m].b];
        if (fabs(cvKey->sigma - ssKey->sigma) < scaleDiffThreshold)
            continue;

        if ((ssKey->x < frontUpperLimit || ssKey->x > frontLowerLimit) &&
                cvKey->sigma <= ssKey->sigma)
            nFrontConB++;

        else if ((ssKey->x > backLowerLimit && ssKey->x < backUpperLimit) &&
                 cvKey->sigma >= ssKey->sigma)
            nBackExpB++;
    }

    /*
    cout << "nFrontExpA / nFrontA: " << nFrontExpA / (double) nFrontA << endl;
    cout << "nBackConA / nBackA: " << nBackConA / (double) nBackA << endl;
    cout << "nFrontConB / nFrontB: " << nFrontConB / (double) nFrontB << endl;
    cout << "nBackExpB / nBackB: " << nBackExpB / (double) nBackB << endl;
    */

//    return 0.25*( nFrontExpA / (double) nFrontA + nBackConA / (double) nBackA +
//                  nFrontConB / (double) nFrontB + nBackExpB / (double) nBackB );

    // Version used in submitted and final IROS 2010 paper.
    return (nFrontExpA / (double) nFrontA) * (nBackConA / (double) nBackA) *
           (nFrontConB / (double) nFrontB) * (nBackExpB / (double) nBackB);
}
