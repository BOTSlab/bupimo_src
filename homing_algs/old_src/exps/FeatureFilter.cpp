#include "FeatureFilter.h"
#include "Angles.h"
#include "Pose.h"
#include <sstream>
#include <iomanip>
#include <fstream>

FeatureFilter::FeatureFilter(string inSequenceDir)
    : imgSequenceDir(inSequenceDir),
      imgSequence(imgSequenceDir),
      n(imgSequence.getLength()),
      imgWidth(imgSequence.getImg(0)->getWidth()),
      imgHeight(imgSequence.getImg(0)->getHeight())
{
    // Create extractor and extract keypoints for all images in the sequence.
    extractor = new SiftExtractor(imgWidth, imgHeight);
    keysArray = new vector<Keypoint*>[n];
    for (int i=0; i<n; i++)
        extractor->extract(*(imgSequence.getImg(i)), keysArray[i]);
}

FeatureFilter::~FeatureFilter() {
    for (int i=0; i<n; i++)
        SiftExtractor::clearKeys(keysArray[i]);

    delete extractor;
}

void FeatureFilter::nullFilter() {

    for (int ssIndex = 0; ssIndex < n; ssIndex++) {
        // Save the raw keys to a file.
        ostringstream oss;
        oss << imgSequenceDir << "/";
        oss << setfill('0') << setw(3);
        oss << ssIndex;
        oss << ".keys";
        SiftExtractor::saveKeys(keysArray[ssIndex], oss.str());
    }
}

void FeatureFilter::floorFilter() {

    int floorHeight = (int) (0.6 * imgHeight);

    for (int c = 0; c < n; c++) {
        vector<Keypoint*> filteredKeys;
        int n = keysArray[c].size();
        for (int k=0; k<n; k++) {
            Keypoint *key = keysArray[c][k];
            if (key->y < floorHeight)
                filteredKeys.push_back(key);
        }

        // Save the filtered keys to a file
        ostringstream oss;
        oss << imgSequenceDir << "/" << setfill('0') << setw(3) << c << ".keys";
        SiftExtractor::saveKeys(filteredKeys, oss.str());
    }
}

void FeatureFilter::shiftFilter() {

    int shiftThreshold = imgWidth / 10;

    for (int c = 0; c < n-1; c++) {

        int s = c + 1;

        // Determine the rotation between the image at c and the one at s.
        Pose ssPose = imgSequence.getOdoPose(s);
        Pose cvPose = imgSequence.getOdoPose(c);
        double rotation = Angles::constrainAngle(ssPose.theta - cvPose.theta);
        int rotInPixels = -imgWidth * rotation / TWO_PI;

        vector<Match> matches;
        SiftExtractor::match(keysArray[s], keysArray[c], matches);

        vector<Keypoint*> filteredKeys;
        for (int m=0; m<matches.size(); m++) {
            Keypoint *keyS = keysArray[s][matches[m].a];
            Keypoint *keyC = keysArray[c][matches[m].b];

            // Account for wrap-around by finding the shortest distance between
            // the horizontal coordinates of the two matched features.
            double sx = keyS->x + rotInPixels;
            double cx = keyC->x;
            double dx = 0;
            if (sx > cx)
                dx = min(sx - cx, imgWidth - sx + cx);
            else if (sx < cx)
                dx = min(cx - sx, imgWidth-1 - cx + sx);

            double dy = keyS->y - keyC->y;
            double shift = sqrt(dx*dx + dy*dy);
            if (shift < shiftThreshold)
                filteredKeys.push_back(keyC);
        }
        cout << "\tfiltered features: " << filteredKeys.size() << " / " 
             << keysArray[c].size() << endl;

        // Save the filtered keys to a file
        ostringstream oss;
        oss << imgSequenceDir << "/" << setfill('0') << setw(3) << c << ".keys";
        SiftExtractor::saveKeys(filteredKeys, oss.str());
    }

    // The last one is unfiltered.
    ostringstream oss;
    oss << imgSequenceDir << "/" << setfill('0') << setw(3) <<n-1<< ".keys";
    SiftExtractor::saveKeys(keysArray[n-1], oss.str());
}


void FeatureFilter::matchFilter() {

    for (int ssIndex = 0; ssIndex < n; ssIndex++) {

        int nKeys = keysArray[ssIndex].size();
        double ssdRatios[nKeys];
        SiftExtractor::matchWithin(keysArray[ssIndex], ssdRatios);

        vector<Keypoint*> filteredKeys;
        for (int k=0; k<nKeys; k++) {
cout << " " << ssdRatios[k] << endl;
            if (ssdRatios[k] > 250)
                filteredKeys.push_back(keysArray[ssIndex][k]);
        }
//        cout << "\tfiltered features: " << filteredKeys.size() << " / " 
//             << nKeys << endl;

        // Save the filtered keys to a file
        ostringstream oss;
        oss << imgSequenceDir << "/";
        oss << setfill('0') << setw(3);
        oss << ssIndex;
        oss << ".keys";
        SiftExtractor::saveKeys(filteredKeys, oss.str());
    }
}

void FeatureFilter::angleFilter(int maxSeparation, double aeThreshold) {
    //
    // TBD Handle filtering of features in first snapshot image.
    //

    for (int ssIndex = maxSeparation; ssIndex < n; ssIndex++) {

        // Create an array to store a quality measure for each feature in the
        // snapshot at index 'ssIndex'.  The quality measure used will be
        // average angular error (so lower is better).  We also create a
        // corresponding array which gives the number of times an angular error
        // has been computed for each particular feature.
        int nSSFeatures = keysArray[ssIndex].size();
        double aae[nSSFeatures];
        int used[nSSFeatures];
        for (int i=0; i<nSSFeatures; i++) {
            aae[i] = 0;
            used[i] = 0;
        }
        cout << "ssIndex: " << ssIndex << endl;
        cout << "\traw features: " << nSSFeatures << endl;

        for (int cvIndex = ssIndex-maxSeparation; cvIndex < ssIndex; cvIndex++)
            accumulateAngularError(ssIndex, cvIndex, aae, used);

        // Extract those keys whose accumulated AAE falls below the threshold.
        vector<Keypoint*> filteredKeys;
        for (int i=0; i<nSSFeatures; i++) {
            if (used[i] > 0) {
                if (aae[i] / used[i] < aeThreshold)
                    filteredKeys.push_back(keysArray[ssIndex][i]);
            }
        }
        cout << "\tfiltered features: " << filteredKeys.size() << endl;

        // Save the filtered keys to a file
        ostringstream oss;
        oss << imgSequenceDir << "/";
        oss << setfill('0') << setw(3);
        oss << ssIndex;
        oss << ".keys";
        SiftExtractor::saveKeys(filteredKeys, oss.str());
    }

    // Snapshot 0 is left unfiltered for the time being.
    ostringstream oss;
    oss << imgSequenceDir << "/000.keys";
    SiftExtractor::saveKeys(keysArray[0], oss.str());
}

void FeatureFilter::accumulateAngularError(int ssIndex, int cvIndex,
                                           double *aae, int *used) {
    cout << "\tcvIndex: " << cvIndex << endl;

    // Compute the home angle from odometry.
    Pose cvPose = imgSequence.getOdoPose(cvIndex);
    Pose ssPose = imgSequence.getOdoPose(ssIndex);
    double odoHomeAngle = PI - cvPose.theta - atan2(
                                               ssPose.y - cvPose.y,
                                               cvPose.x - ssPose.x);

    // Compute the matches from SS to CV
    vector<Match> matches;
    cout << "\tFeatureFilter: Computing matches..." << endl;
    SiftExtractor::match(keysArray[ssIndex], keysArray[cvIndex], matches);
    cout << "\tFeatureFilter: Done." << endl;

    // Now consider each match in turn.  This is essentially running
    // HISS on a per match basis.
    for (int m=0; m<matches.size(); m++) {

        Keypoint *keyInSS = keysArray[ssIndex][matches[m].a];
        Keypoint *keyInCV = keysArray[cvIndex][matches[m].b];

        double homeAngle;
        if (keyInCV->sigma > keyInSS->sigma)
            // Feature has expanded.  Computed home direction is away
            // from it.
            homeAngle = TWO_PI * (1.0 - keyInCV->x / imgWidth) + PI;
        else if (keyInCV->sigma < keyInSS->sigma)
            // Feature has contracted.  Computed home direction is
            // towards it.
            homeAngle = TWO_PI * (1.0 - keyInCV->x / imgWidth);
        else
            // Feature has not changed in scale.  However, we assume it
            // should have so we assign an angle 180 degrees opposite
            // to odoHomeAngle, which will then yield an angular error
            // of 180 degrees.
            homeAngle = odoHomeAngle + PI;

        // Compare the two angles.
        double diff = Angles::getAngularDifference(homeAngle, odoHomeAngle);

        aae[matches[m].a] += diff;
        used[matches[m].a]++;
    }
}
