/**
 * Processes adjacent pairs of images in an ImgSequence and determines the home
 * angle computed by odometry and by visual homing.  Displays the corresponding
 * vectors in a SeqWindow.
 *
 * \author Andrew Vardy
 */


#include "ImgSequence.h"

#include "SiftExtractor.h"
//#include "FeatureHissAlg.h"
#include "FBAlg.h"

//#include "WarpingAlg.h"

//#include "TwoStepHomingAlg.h"
//#include "OpticFlowHomingAlg.h"
//#include "DIDCompass.h"

#include "SeqWindow.h"
#include "Angles.h"
#include "Pose.h"
#include <sstream>
#include <iomanip>
#include <fstream>

int main() {

    // Parameters
    string seqName("inco4");
    ostringstream seqPath;
    seqPath << "/data/seq/" << seqName;
    string sequenceDir(seqPath.str());
    bool useRawKeys = true;

    ImgSequence imgSequence(sequenceDir);
    int n = imgSequence.getLength();
    int w = imgSequence.getImageWidth();
    int h = imgSequence.getImageHeight();

    // Instantiate homing algorithm.
    //FeatureHissAlg homingAlg(w);
    FBAlg homingAlg(w);

    //WarpingAlg homingAlg(w, h);

    //DIDCompass compassAlg(w, h);
    //OpticFlowHomingAlg transAlg(w, h);
    //TwoStepHomingAlg homingAlg(w, h, &compassAlg, &transAlg);

    // Create window to display sequence data.
    SeqWindow seqWindow(seqName, imgSequence, true);

    // Create extractor and extract keypoints for all images in the .
    vector<Keypoint*> keysArray[n];
    SiftExtractor extractor(imgSequence.getImageWidth(),
                            imgSequence.getImageHeight());
    for (int i=0; i<n; i++) {
        ostringstream oss;
        if (useRawKeys)
            oss << sequenceDir << "/rawKeys/";
        else
            oss << sequenceDir << "/";
        oss << setfill('0') << setw(3) << i << ".keys";
        SiftExtractor::loadKeys(oss.str(), keysArray[i]);
    }

    // Process adjacent pairs of images.
    for (int cvIndex = 0; cvIndex < n-1; cvIndex++) {

        int ssIndex = cvIndex + 1;
        cout << endl << "Comparing " << cvIndex << " to " << ssIndex <<endl;

        // Compute the home angle from CV to SS
        vector<Match> matches;
        SiftExtractor::match(keysArray[ssIndex], keysArray[cvIndex], matches);
        double homeAngle = homingAlg.getHomeAngle(
                           keysArray[ssIndex], keysArray[cvIndex], matches);

        /*
        Img *CV = imgSequence.getImg(cvIndex);
        Img *SS = imgSequence.getImg(ssIndex);
        homingAlg.snapshot(SS);
        double homeAngle, similarity;
        homingAlg.getHomeAngle(CV, homeAngle, similarity);
        */

        // Compute the home angle from odometry.
        Pose cvPose = imgSequence.getOdoPose(cvIndex);
        Pose ssPose = imgSequence.getOdoPose(ssIndex);
        double odoAngle = atan2(ssPose.y - cvPose.y, ssPose.x - cvPose.x)
                          - cvPose.theta;

        // Compare the two angles.
        double diff = Angles::getAngularDifference(homeAngle, odoAngle);
        cout << "\todoAngle: " << odoAngle << endl;
        cout << "\thomeAngle: " << homeAngle << endl;
        cout << "\tdiff: " << diff << endl;
        cout << "\trotation: " << (ssPose.theta - cvPose.theta) << endl;

        float vecLength = Pose::distanceBetween(cvPose, ssPose);
        seqWindow.addPoint(cvIndex, 1);
        seqWindow.addVector(cvIndex, 0.5f * vecLength,
                            odoAngle + cvPose.theta, 0.75f);
        seqWindow.addVector(cvIndex, 0.75f * vecLength, 
                            homeAngle + cvPose.theta, 0.0f);
    }

    return 0;
}
