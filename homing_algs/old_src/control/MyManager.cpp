#include "MyManager.h"
#include "SingleConfig.h"
#include <iomanip>
#include <fstream>
#include <glog/logging.h>

MyManager::MyManager(DriverAdaptor &inAdaptor)
    : adaptor(inAdaptor),
      posIndex(0),
      CV(NULL),
      trialIndex(-1)
{
    SingleConfig* config = SingleConfig::getInstance();
    string snapshotDirName =
            config->getString("MyManager.snapshotDirName");
    useBayes = config->getBool("MyManager.useBayes", false);
    scaleMeasure = config->getBool("MyManager.scaleMeasure", false);
    advanceSnapshot = config->getBool("MyManager.advanceSnapshot", false);
    meanMovement = config->getDouble("MyManager.meanMovement", 0.5);
    stdevMovement = config->getDouble("MyManager.stdevMovement", 1.0);
    measModelRadius = config->getInt("MyManager.measModelRadius", 1);

    sSeq = new ImgSequence(snapshotDirName);
    nSnapshots = sSeq->getLength();
    nPositions = nSnapshots - 1;

    // Create extractor and make sure that keypoints are extracted for all
    // snapshot images.  This is necessary because these keypoints are used not
    // just by the homingAlg, but also for localization.
    adaptor.oss() << "Extracting route keypoints...";
    LOG(INFO) << adaptor.oss().str();
    adaptor.printOss();

    extractor = new SiftExtractor(sSeq->getImageWidth(),
                                  sSeq->getImageHeight());
    for (int s=0; s<nSnapshots; s++) {
        Img *S = sSeq->getImg(s);
        if (!S->hasKeypoints())
            extractor->extract(*S, S->getKeypoints());
    }
    adaptor.oss() << "Done.";
    LOG(INFO) << adaptor.oss().str();
    adaptor.printOss();

    homingAlg = TotalHomingAlg::createFromConfig(sSeq->getImageWidth(),
												 sSeq->getImageHeight());
    Img *S1;
    if (advanceSnapshot)
        // Start homing towards image 2.
        S1 = sSeq->getImg(2);
    else
        // Start homing towards image 1.
        S1 = sSeq->getImg(1);
    homingAlg->snapshot(S1);

    // Create arrays to represent the current belief (bel), previous belief
    // (lastBel), predicted belief (belBar), and probability of the current
    // observation (probZ).  updateBeliefLocalMax uses only probZ.
    if (useBayes) {
        bel = new double[nPositions];
        lastBel = new double[nPositions];
        belBar = new double[nPositions];
    }
    probZ = new double[nPositions];

    // Initialize belief to the initial position (only lastBel needs to be
    // initialized).
    if (useBayes) {
        lastBel[0] = 1;
        for (int p=1; p<nPositions; p++)
            lastBel[p] = 0;
    }

    // Parameters used in storeAndVisualize.
    pointSize = 0.2;
    radiusConstant = 0.6;
    plusCrossSize = 3.0;
}

MyManager::~MyManager() {
    delete sSeq;
    delete extractor;
    if (useBayes) {
        delete bel;
        delete lastBel;
        delete belBar;
    }
    delete probZ;
}

double MyManager::getHomeAngle(double displacement, 
                               bool &arrived, bool &routeComplete) {
    trialIndex++;
    adaptor.oss() << "trialIndex: " << trialIndex;
    LOG(INFO) << adaptor.oss().str();
    adaptor.printOss();

    arrived = false;
    routeComplete = false;

    adaptor.getImg(CV);

    // Save the current image and also capture a picture of the robot from the
    // observation camera (or of the Stage window if we are doing a
    // simulation).
    saveCV();

    // Compute home angle.
    double homeAngle, similarity;
    homingAlg->getHomeAngle(CV, homeAngle, similarity);

    adaptor.oss() << "homeAngle: " << homeAngle;
    LOG(INFO) << adaptor.oss().str();
    adaptor.printOss();

    if (useBayes)
        updateBeliefBayes(displacement, arrived);
    else
        updateBeliefLocalMax(arrived);

    // If arrived is true then posIndex has been advanced (possibly by more
    // than one unit).  The consequences are that we may have completed the
    // route.  If not then we at least have to choose a new snapshot image to
    // home to.
    if (arrived) {
        if (posIndex == nPositions-1) {
            adaptor.oss() << "Route complete!";
            LOG(INFO) << adaptor.oss().str();
            routeComplete = true;
        } else {

            // Set the next snapshot to home towards.
            Img *S;
            if (advanceSnapshot)
                S = sSeq->getImg(posIndex+2);
            else
                S = sSeq->getImg(posIndex+1);
            homingAlg->snapshot(S);
        }
    }
    if (!routeComplete) {
        adaptor.oss() << "posIndex: " << posIndex;
        LOG(INFO) << adaptor.oss().str();
        adaptor.printOss();
    }

    return homeAngle;
}

void MyManager::saveCV() {
    LOG(INFO) << "saveCV START";

    ostringstream baseName;
    baseName << setfill('0') << setw(3) << trialIndex;

    // Save the CV image to the current directory.
    ostringstream cvImageName;
    cvImageName << "cv_" << baseName.str() << ".png";
    CV->save(cvImageName.str());

    // Capture a picture of the robot from the observation camera (or of the
    // Stage window if we are doing a simulation).
    ostringstream filename;
    filename << "ob_" << baseName.str() << ".jpg";
    adaptor.captureFromObservationCamera(filename.str());

    LOG(INFO) << "saveCV END";
}

double MyManager::similarity(int pIndex, 
                             vector<Match> &matchesA, vector<Match> &matchesB) {
    vector<Keypoint*> &cvKeys = CV->getKeypoints();

    if (scaleMeasure)
        return extractor->scaleDiff(sSeq->getImg(pIndex)->getKeypoints(),
                                    sSeq->getImg(pIndex+1)->getKeypoints(),
                                    cvKeys, matchesA, matchesB);
    else
        return extractor->percentMatched(sSeq->getImg(pIndex)->getKeypoints(),
                                         sSeq->getImg(pIndex+1)->getKeypoints(),
                                         matchesA, matchesB);
}

void MyManager::updateBeliefLocalMax(bool &arrived) {

    // Apply the measurement model to fill in the three adjacent values around
    // posIndex in probZ.
    applyMeasModel();

    checkArrived(probZ, arrived);

    storeAndVisualize(probZ, "localMax");
}

void MyManager::updateBeliefBayes(double displacement, bool &arrived) {

    LOG(INFO) << "updateBeliefBayes START";

    // Initialize belief.
    for (int i=0; i<nPositions; i++)
        bel[i] = 0;

    // Apply prediction step of Bayes filter.
    LOG(INFO) << "computing prediction START";
    for (int p=0; p<nPositions; p++) {
        belBar[p] = 0;
        for (int lastP=0; lastP<nPositions; lastP++) {
            belBar[p] += motionModel(lastP, p) * lastBel[lastP];
        }
    }
    LOG(INFO) << "computing prediction END";

    storeAndVisualize(belBar, "belBar");
        
    // Apply the measurement model to update z for p-positions corresponding
    // to curGoalG
    applyMeasModel();
    storeAndVisualize(probZ, "probZ");

    // Now incorporate the results of the measurement model.
    double sumOfBel = 0;
    for (int p=0; p<nPositions; p++) {
        bel[p] = probZ[p] * belBar[p];
        sumOfBel += bel[p];
    }

    // Normalize bel.
    for (int p=0; p<nPositions; p++)
        bel[p] /= sumOfBel;

    storeAndVisualize(bel, "bel");

    checkArrived(bel, arrived);

    // Exchange bel and lastBel.
    for (int p=0; p<nPositions; p++)
        lastBel[p] = bel[p];

    LOG(INFO) << "updateBeliefBayes END";
}

double MyManager::motionModel(int lastP, int p) {

    // movement.
    double dp = p - lastP - meanMovement;

    return //(1.0/(stdevMovement*sqrt(2.0*M_PI))) *
           exp(-0.5*dp*dp/(stdevMovement*stdevMovement));
}

/*
void MyManager::applyMeasModel() {

    adaptor.print("applyMeasModel START");

    // Initialize probZ.
    for (int i=0; i<nPositions; i++)
        probZ[i] = 0;

    // Compute matches from the current snapshot to the current image.
    // BAD: The homing algorithm already does this!
    vector<Keypoint*> &cvKeys = CV->getKeypoints();
    vector<Match> matchesCur;
    extractor->match(sSeq->getImg(posIndex+1)->getKeypoints(), cvKeys,
                    matchesCur);

    vector<Match> matchesA;
    extractor->match(sSeq->getImg(posIndex)->getKeypoints(), cvKeys, matchesA);
    probZ[posIndex] = similarity(posIndex, matchesA, matchesCur);

    if (posIndex == 0) {
        // Compute similarity of the 2nd position.
        vector<Match> matchesB;
        extractor->match(sSeq->getImg(2)->getKeypoints(), cvKeys, matchesB);
        probZ[1] = similarity(1, matchesCur, matchesB);
    } else if (posIndex == nPositions-1) {
        // We're done already!
    } else {
        // Update belief at posIndex-1.
        vector<Match> matches;
        extractor->match(sSeq->getImg(posIndex-1)->getKeypoints(), cvKeys,
                        matches);
        probZ[posIndex-1] = similarity(posIndex-1, matches, matchesA);

        matches.clear();
        extractor->match(sSeq->getImg(posIndex+2)->getKeypoints(), cvKeys,
                        matches);
        probZ[posIndex+1] = similarity(posIndex+1, matchesCur, matches);
    }

    adaptor.print("applyMeasModel END");
}

void MyManager::checkArrived(double *belief, bool &arrived) {
    if (posIndex == 0) {
        if (belief[1] > belief[0])
            arrived = true;
    } else if (posIndex == nPositions-1) {
        // We're done already!
        arrived = true;
    } else {
        if (belief[posIndex+1] > belief[posIndex] &&
            belief[posIndex+1] > belief[posIndex-1])
            arrived = true;
    }
}
*/

void MyManager::applyMeasModel() {

    LOG(INFO) << "applyMeasModel START";

    // Initialize probZ.
    for (int i=0; i<nPositions; i++)
        probZ[i] = 0;

    int firstP = std::max(0, posIndex-measModelRadius);
    int lastP = std::min(nPositions-1, posIndex+measModelRadius);
    vector<Keypoint*> &cvKeys = CV->getKeypoints();

    vector<Match> matchesA, matchesB;
    extractor->match(sSeq->getImg(firstP)->getKeypoints(), cvKeys, matchesA);
    extractor->match(sSeq->getImg(firstP+1)->getKeypoints(), cvKeys, matchesB);
    probZ[firstP] = similarity(firstP, matchesA, matchesB);
    matchesA = matchesB;

    for (int p=firstP+1; p<=lastP; p++) {
        
        // On the previous iteration (or in the initialization above the loop)
        // we have already computed the matches for snapshot image p.  They are
        // stored in matchesA.  We just need to compute the matches for
        // position 'p+1'.  We will put these in matchesB.
        
        matchesB.clear();
        extractor->match(sSeq->getImg(p+1)->getKeypoints(), cvKeys, matchesB);

        probZ[p] = similarity(p, matchesA, matchesB);

        // Copy the matches from B into A for the next iteration.
        if (p != lastP)
            matchesA = matchesB;
    }

    LOG(INFO) << "applyMeasModel END";
}

void MyManager::checkArrived(double *belief, bool &arrived) {

    // Determine the position of maximum belief.
    double maxP = 0;
    double biggestValue = belief[0];
    for (int p=1; p<nPositions; p++)
        if (belief[p] > biggestValue) {
            maxP = p;
            biggestValue = belief[p];
        }

    // Three strategies:
    //
    // (1) If maxP is ahead of posIndex then we assume we have
    //     arrived at posIndex + 1.  Note that this strategy does not allow
    //     positions to be skipped.
    // if (maxP > posIndex) {
    //     posIndex + 1;
    //     arrived = true;
    // }

    // (2) Whatever maxP is, we assume it is right so we immediately set
    //     posIndex to this value.
    //if (posIndex != maxP) {
    //    posIndex = maxP;
    //    arrived = true;
    //}

    // (3) Like (2) but we only consider advancing posIndex.  With (2) it is
    //     possible for the belief to oscillate, causing the robot to oscillate
    //     and make no progress.
    if (posIndex < maxP) {
        posIndex = maxP;
        arrived = true;
    }
}


void MyManager::storeAndVisualize(double *inDist, string title) {
    // Get a string that combines the title and the trialIndex.
    ostringstream oss;
    oss << title << "_" << setfill('0') << setw(3) << trialIndex;
    string titleOut(oss.str());


    // Save the contents of the array to disk.
    oss << ".dat";
    ofstream outFile(oss.str().c_str());
    for (int p=0; p<nPositions; p++) {
        outFile << inDist[p] << " ";
    }
    outFile << endl;
    outFile.close();

    // Make a copy of the input distribution and normalize it for display
    // purposes.
    double dist[nPositions];
    double maxDensity = 0;
    for (int p=0; p<nPositions; p++) {
        dist[p] = inDist[p];
        if (dist[p] > maxDensity) maxDensity = dist[p];
    }
    if (maxDensity > 1e-5)
        for (int p=0; p<nPositions; p++)
            dist[p] /= maxDensity;
    
    // Create a window for display
    double minX = sSeq->getMinX();
    double minY = sSeq->getMinY();
    double maxX = sSeq->getMaxX();
    double maxY = sSeq->getMaxY();
    double width = maxX - minX;
    double margin = width * 0.2;

    SimWindow window(titleOut, minX - margin, minY - margin,
                     maxX + margin, maxY + margin, true);

    // Determine the s-position of maximum belief.
    double maxValue = 0;
    int maxS = 0;
    for (int s=0; s<nPositions; s++) {
        if (dist[s] > maxValue) {
            maxValue = dist[s];
            maxS = s;
        }
    }

    // Draw circles sized according to the probability of the given distribution
    Pose firstPose = sSeq->getTruePose(0);
    Pose secondPose = sSeq->getTruePose(1);
    double dx = firstPose.x - secondPose.x;
    double dy = firstPose.y - secondPose.y;
    double radiusFactor = radiusConstant*sqrt(dx*dx + dy*dy);
    for (int s=0; s<nPositions; s++) {
        Pose currentPose = sSeq->getTruePose(s);
        Pose nextPose = sSeq->getTruePose(s+1);

        double radius = radiusFactor * dist[s];

        window.plotEllipseFromTo(currentPose.x, currentPose.y,
                                 nextPose.x, nextPose.y, radius);

        // Draw a plus at the position of maximum belief.
        if (s == maxS) {
            double x = 0.5*(currentPose.x + nextPose.x);
            double y = 0.5*(currentPose.y + nextPose.y);
            window.addPoint(x, y, 2, plusCrossSize);
        }
    }

//    ImgWindow::pauseWindow();
}
