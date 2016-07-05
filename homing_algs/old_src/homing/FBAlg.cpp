#include "FBAlg.h"
#include "Angles.h"
#include "Arrays.h"
#include "SingleConfig.h"
#include "SubsetSummer.h"
#include <cassert>
#include <iomanip>
#include <glog/logging.h>

// This is a bit of a hack.  We define a width variable that will be set to the
// same value as the instance variable width.  The only purpose is to access
// the image width in 'compareFrontKeys' which is used to perform a
// left-to-right sort of a set of keypoints in the front part of the SS image.
int FBAlg::staticWidth = 0;
int FBAlg::staticStartX = 0;
vector<Keypoint*> FBAlg::staticCVKeys;

FBAlg::FBAlg(int inWidth, int inHeight) :
    ssImg(NULL), cvImg(NULL), width(inWidth), height(inHeight),
    extractor(width, height)
{
    SingleConfig *config = SingleConfig::getInstance();
    includeAngle = PI_OVER_180 * config->getDouble("FBAlg.includeAngle", 180);
    deviationFilter = config->getBool("FBAlg.deviationFilter", false);
    stdevs = config->getDouble("FBAlg.stdevs", 1.0);
    orderFilter = config->getBool("FBAlg.orderFilter", false);
    applySubsets = config->getBool("FBAlg.applySubsets", false);
    resolution = config->getInt("FBAlg.resolution", 10);
    forwardContracted = config->getBool("FBAlg.forwardContracted", false);    
    backwardExpanded = config->getBool("FBAlg.backwardExpanded", false);    
    outputZhang = config->getBool("FBAlg.outputZhang", false);    
    outputF = config->getBool("FBAlg.outputF", false);    
    outputB = config->getBool("FBAlg.outputB", false);    
    outputR = config->getBool("FBAlg.outputR", false);    
    outputMix = config->getBool("FBAlg.outputMix", false);
    debug = config->getBool("FBAlg.debug", false);    

    staticWidth = width;

    if (debug)
        window = new ImgWindow("FBAlg");
}

FBAlg::~FBAlg() {
    if (debug)
        delete window;
}

void FBAlg::snapshot(Img* SS) {
    ssImg = SS;

    if (!SS->hasKeypoints())
        extractor.extract(*SS, SS->getKeypoints());
    vector<Keypoint*> &ssKeys = SS->getKeypoints();
    int n = ssKeys.size();

    // Separate out the keypoints from the snapshot image into ssFrontKeys and
    // ssBackKeys.
    ssFrontKeys.clear();
    ssBackKeys.clear();

    double includeWidth = width * ((includeAngle/2.0) / TWO_PI);
//    int frontLeftBound = (int) max(0.75*width, width - 1 - includeWidth);
//    int frontRightBound = (int) min(0.25*width, includeWidth);
//    int backLeftBound = (int) (width/2 - includeWidth);
//    int backRightBound = (int) (width/2 + includeWidth);
    int frontLeftBound = (int) (width - 1 - includeWidth);
    int frontRightBound = (int) (includeWidth);
    int backLeftBound = (int) (width/2 - includeWidth);
    int backRightBound = (int) (width/2 + includeWidth);

    for (int i=0; i<n; i++) {
        Keypoint *key = ssKeys[i];
        double sx = key->x;
        if (sx > frontLeftBound || sx < frontRightBound)
            ssFrontKeys.push_back(key);
        else if (sx > backLeftBound && sx < backRightBound)
            ssBackKeys.push_back(key);
    }
    //LOG(INFO) << "ssFrontKeys.size(): " << ssFrontKeys.size() << endl;
    //LOG(INFO) << "ssBackKeys.size(): " << ssBackKeys.size() << endl;
}

void FBAlg::getHomeAngle(Img* CV, double &homeAngle, double &similarity) {
    cvImg = CV;

    LOG(INFO) << "getHomeAngle START" << endl;

    if (debug)
        window->clear();

    // Extract keypoints from CV, if they haven't already been extracted.
    if (!CV->hasKeypoints())
        extractor.extract(*CV, CV->getKeypoints());
    vector<Keypoint*> &cvKeys = CV->getKeypoints();
    //LOG(INFO) << "cvKeys.size(): " << cvKeys.size() << endl;

    if (outputZhang) {
        TwoVec f, ssF;
        int nMatches;
        computeForB(true, ssFrontKeys, cvKeys, f, ssF, nMatches);
        homeAngle = f.angle() - ssF.angle();
        similarity = nMatches / (double)(ssFrontKeys.size());
    } else if (outputF) {
        TwoVec f, ssF;
        int nMatches;
        computeForB(true, ssFrontKeys, cvKeys, f, ssF, nMatches);
        homeAngle = f.angle();
        similarity = nMatches / (double)(ssFrontKeys.size());
    } else if (outputB) {
        TwoVec b, ssB;
        int nMatches;
        computeForB(false, ssBackKeys, cvKeys, b, ssB, nMatches);
        homeAngle = b.angle();
        similarity = nMatches / (double)(ssBackKeys.size());
    } else if (outputR || outputMix) {
        TwoVec f, b, ssF, ssB;
        int nMatchesF, nMatchesB;
        computeForB(true, ssFrontKeys, cvKeys, f, ssF, nMatchesF);
        computeForB(false, ssBackKeys, cvKeys, b, ssB, nMatchesB);
        similarity = (nMatchesF + nMatchesB) / 
                     (double)(ssFrontKeys.size() + ssBackKeys.size());

        if (f.x == 0 && f.y == 0) {
            // If there are no front features then follow the opposite of b.
            homeAngle = b.angle() + PI;
            LOG(INFO) << "FBAlg: NO FRONT FEATURES!" << endl;
        } if (b.x == 0 && b.y == 0) {
            // If there are no back features then just follow f.
            homeAngle = f.angle();
            LOG(INFO) << "FBAlg: NO BACK FEATURES!" << endl;
        }

        // Vector r points towards the average of f and b.
        TwoVec r = (f + b)/2.0;
        r.normalize();

        if (outputR)
            homeAngle = r.angle();
        else if (outputMix) {
            // This weight gives the balance between movement along the route
            // (following f) and returning to the route (following r).
            double weight = 0.5*max(-f.dot(b), 0.0);

            TwoVec homeVec = weight*f + (1-weight)*r;
            homeAngle = homeVec.angle();
        }
    }

    if (debug) {
        // Draw the computed home angle.
        int homeAngleX = (int)(width * (1 - homeAngle / TWO_PI)) % width;
        window->drawVerticalBar(homeAngleX, 3, 0.0, 0.0, 1.0);
        window->interact();
    }

    LOG(INFO) << "getHomeAngle END" << endl;
}

void FBAlg::computeForB(bool computingF,
                        vector<Keypoint*> &ssKeys, vector<Keypoint*> &cvKeys,
                        TwoVec &v, TwoVec &ssV, int &nMatches) {
    /*
    if (computingF)
        LOG(INFO) << "Computing F: " << endl;
    else
        LOG(INFO) << "Computing B: " << endl;
    */

    vector<Match> matches;
    extractor.match(ssKeys, cvKeys, matches);

    LOG(INFO) << "Matches: " << matches.size() << endl;

    if (debug)
        window->addImagesWithMatches(*ssImg, *cvImg, ssKeys, cvKeys, matches,
                                    "Original matches");

    if (orderFilter && computingF) {
        frontOrderFilter(ssFrontKeys, cvKeys, matches);
        LOG(INFO) << "O-Filtered Matches: " << matches.size() << endl;
        if (debug)
//            window->addImagesWithMatches(*ssImg, *cvImg, ssKeys, cvKeys,
//                                         matches, "O-Filtered matches");
            window->addImagesWithMatches(*ssImg, *cvImg, ssFrontKeys, cvKeys,
                                         matches, "O-Filtered matches");
    }

    if (deviationFilter) {
        extractor.deviationFilter(ssKeys, cvKeys, matches, stdevs);
        LOG(INFO) << "D-Filtered Matches: " << matches.size() << endl;
        if (debug)
            window->addImagesWithMatches(*ssImg, *cvImg, ssKeys, cvKeys,
                                         matches, "D-Filtered matches");
    }

    if (applySubsets) {
        extractSubsetsDP(ssKeys, matches, !computingF);
        LOG(INFO) << "Subset-sum filtered matches: " << matches.size() << endl;
        if (debug)
            window->addImagesWithMatches(*ssImg, *cvImg, ssKeys, cvKeys,
                                         matches,"Subset-sum filtered matches");
    }

    if (computingF && forwardContracted) {
        // Remove any expanded matches.
        vector<Match>::iterator it = matches.begin();
        for (; it != matches.end();) {
            if (cvKeys[it->b]->sigma > ssKeys[it->a]->sigma)
                matches.erase(it);
            else
                it++;
        }
    }
    if (!computingF && backwardExpanded) {
        // Remove any contracted matches.
        vector<Match>::iterator it = matches.begin();
        for (; it != matches.end();) {
            if (cvKeys[it->b]->sigma < ssKeys[it->a]->sigma)
                matches.erase(it);
            else
                it++;
        }
    }

    LOG(INFO) << "Contract/expand filtered matches: " << matches.size() << endl;
    if (debug)
        window->addImagesWithMatches(*ssImg, *cvImg, ssKeys, cvKeys, matches,
                                    "Contract/expand filtered matches");

    // Compute the vector v, which could be either f or b.  Also compute
    // the value of this vector for the snapshot.
    const double PIXEL2ANGLE = TWO_PI / width;
    int n = matches.size();
    for (int m=0; m<n; m++) {
        double angle = PIXEL2ANGLE * (width - 1 - cvKeys[matches[m].b]->x);
        v.x += cos(angle);
        v.y += sin(angle);
        double ssAngle = PIXEL2ANGLE * (width - 1 - ssKeys[matches[m].a]->x);
        ssV.x += cos(ssAngle);
        ssV.y += sin(ssAngle);
    }

    v.normalize();
    ssV.normalize();

    nMatches = matches.size();
}

bool FBAlg::compareFrontKeys(const Keypoint *key1, const Keypoint *key2) {
    // If either keypoint is in the left quarter of the image, subtract the
    // width of the image from its x-value so that we have a straightforward
    // comparison of x-coordinates.
    int threeQuartersWidth = (int)(0.75*staticWidth);
    int x1 = key1->x;
    int x2 = key2->x;
    if (x1 >= threeQuartersWidth)
        x1 -= staticWidth;
    if (x2 >= threeQuartersWidth)
        x2 -= staticWidth;

    return x1 < x2;
}

bool FBAlg::compareFrontMatches(const Match m1, const Match m2) {
    int x1 = staticCVKeys[m1.b]->x;
    int x2 = staticCVKeys[m2.b]->x;
    if (x1 >= staticStartX)
        x1 -= staticWidth;
    if (x2 >= staticStartX)
        x2 -= staticWidth;

    return x1 < x2;
}

void FBAlg::frontOrderFilter(vector<Keypoint*> &ssKeys,
                             vector<Keypoint*> &cvKeys,
                             vector<Match> &matches) {

    sort(ssFrontKeys.begin(), ssFrontKeys.end(), FBAlg::compareFrontKeys);

matches.clear();
extractor.match(ssFrontKeys, cvKeys, matches);

    // We first need to sort matches according to the order in which keypoints
    // appear in the current image.  This requires defining a start point or
    // origin to establish this ordering.  Since we are just matching features
    // from the front of the snapshot image, the matching features in the
    // current image should generally be clustered in one region of the image
    // (a region whose width is less than half of the image width).  We compute
    // the mean direction of the features in the current image and use the
    // opposite direction as the start point for the ordering.
    const double PIXEL2ANGLE = TWO_PI / width;
    int n = matches.size();
    TwoVec v;
    for (int m=0; m<n; m++) {
        double angle = PIXEL2ANGLE * (width - 1 - cvKeys[matches[m].b]->x);
        v.x += cos(angle);
        v.y += sin(angle);
    }
    staticStartX = (int)(width * (1.0 - (v.angle() + M_PI) / TWO_PI));
    staticCVKeys = cvKeys;
    sort(matches.begin(), matches.end(), FBAlg::compareFrontMatches);

    // Fill a vector with the indices into ssKeys of each matched keypoint.
    vector<int> indices(n);
    for (int m=0; m<n; m++)
        indices[m] = matches[m].a;

    vector<int> filteredIndicesIntoM;
    Arrays::longestIncSub(indices, filteredIndicesIntoM);

    int nF = filteredIndicesIntoM.size();
    vector<Match> filteredMatches;
    for (int i=0; i<nF; i++)
        filteredMatches.push_back( matches[filteredIndicesIntoM[i]] );

    matches.clear();
    matches = filteredMatches;
}

// Remove entries from matches such that the remaining entries correspond to
// the subset of keypoints in ssKeys, whose sine components have a sum as close
// to 0 as possible.  If 'addPi' is true then add PI to the keypoint position
// in radians.  Utilize the dynamic programming approach.
void FBAlg::extractSubsetsDP(vector<Keypoint*> &ssKeys, vector<Match> &matches,
                             bool addPi) {
    int n = (int) matches.size();
    if (n == 0)
        return;

    // Create arrays containing the sine components of the features from the
    // list.  Since the subsetSum procedure expects an array of ints we will
    // further multiply by 'resolution'.  Note that the speed of subsetSum will
    // be governed by the 'resolution' parameter (the coarser the faster).
    int *sines = new int[n];
    for (int m=0; m<n; m++) {
        int x = ssKeys[matches[m].a]->x;
        // The line below does the following from the inside out:
        //  x-coordinate -> radians -> angle in [-pi, pi] -> 
        //  angle in degrees [-180, 180] -> integer angle in degrees
        if (!addPi)
            sines[m] = (int)(resolution * sin(TWO_PI * x / width));
        else
            sines[m] = (int)(resolution * sin(TWO_PI * x / width - PI));
        //LOG(INFO) << "sines[" << m << "]: " << sines[m] << endl;
    }

    // We also need arrays to store results (after subsetSum runs they will 
    // just contain 0's or 1's indicating whether each feature is part of the
    // chosen subset).
    bool *subset = new bool[n];

    SubsetSummer summer(sines, n);

    // Determine the smallest possible sum that can be produced.
    int smallestSum = 0;
    if (!summer.isSumPossible(0)) {
        while (true) {
            smallestSum++;
            if (summer.isSumPossible(smallestSum))
                break;
            if (summer.isSumPossible(-smallestSum)) {
                smallestSum = -smallestSum;
                break;
            }
        }
    }

    LOG(INFO) << "smallest possible sum: " << smallestSum << endl;
    summer.buildBiggestSolutionsTable();
    bool subsetFound = summer.getBiggestSolutionForSum(smallestSum, subset);
    assert(subsetFound);

/*
double verifySum = 0;
for (int i=0; i<n; i++) {
    if (subset[i]) {
LOG(INFO) << "sines[" << i << "]: " << sines[i] << endl;
        verifySum += sines[i];
    }
}
LOG(INFO) << "verifySum: " << verifySum << endl;
*/

/*
LOG(INFO) << "angles: " << endl;
for (int i=0; i<n; i++)
    if (subset[i] == 1)
        LOG(INFO) << bearings[i] << " ";
LOG(INFO) << endl;
*/

    // Remove those matches which were not part of the chosen subset.
    for (int i=n-1; i >= 0; i--)
        if (subset[i] == 0)
            matches.erase(matches.begin()+i);
    
    delete [] sines;
    delete [] subset;
}

// As above, only the method is to select subsets at random until we have a
// below-threshold sum or some number of iterations has elapsed.
void FBAlg::extractSubsetsRandom(vector<Keypoint*> &ssKeys,
                                 vector<Match> &matches, bool addPi) {
    int n = (int) matches.size();
    if (n == 0)
        return;

    // The following arrays represent permutations of the original matches.
    // 'perm' is the current permutation.  'bestPerm' is the best permutation
    // found, 'bestSum' is its sum.
    int *perm = new int[n];
    int *bestPerm = new int[n];
    double bestSum = DBL_MAX;

    for (int attempt = 0; attempt<100000; attempt++) {
        // Generate a random permutation that uses (on average) 50% or more
        // of the keys.
        double fraction = rng.uniform(0.5, 1);
        for (int i=0; i<n; i++) {
            if (rng.uniform(0, 1) < fraction)
                perm[i] = 1;
            else
                perm[i] = 0;
        }

        // Determine the corresponding sum.
        double sum = 0;
        for (int i=0; i<n; i++) {
            int x = ssKeys[matches[i].a]->x;
            if (perm[i] == 1) {
                if (!addPi)
                    sum += sin(TWO_PI * x / width);
                else
                    sum += sin(TWO_PI * x / width - PI);

                // Break if sum is higher than the best found so far.
                if (sum > bestSum)
                    break;
            }
        }

        if (sum < bestSum) {
            // Copy 'perm' into 'bestPerm'
            for (int i=0; i<n; i++)
                bestPerm[i] = perm[i];
            bestSum = sum;
        }
    }
    //LOG(INFO) << "bestSum: " << bestSum << endl;

    // Remove those features which were not part of the best permutation.
    for (int i=n-1; i >= 0; i--)
        if (bestPerm[i] == 0)
            matches.erase(matches.begin()+i);
    
    delete [] perm;
    delete [] bestPerm;
}
