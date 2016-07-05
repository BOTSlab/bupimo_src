#include "RouteManager.h"
#include "SingleConfig.h"
#include "FeatureHissAlg.h"
#include <iomanip>

RouteManager::RouteManager(DriverAdaptor &inAdaptor)
    : adaptor(inAdaptor),
      curGoalG(1),
      method(LOCAL_MAX),
      CV(NULL),
      trialIndex(-1),
      maxBelP(0),
      SIMILARITY_THRESHOLD_VALUE(0.29),
      Z_SEGMENTS_RADIUS(2)
{
    SingleConfig* config = SingleConfig::getInstance();
    string snapshotDirName = config->getString("RouteManager.snapshotDirName");
    sSequence = new ImgSequence(snapshotDirName);
    numberSnapshots = sSequence->getLength();

    ostringstream oss;
    oss << "numberSnapshots: " << numberSnapshots;
    adaptor.print(oss.str());

    // Set the current snapshot image to the first in the sequence.
    Img *SS0 = sSequence->getImg(curGoalG);

    // Create extractor and extract keypoints for all snapshot images.
    adaptor.print("RouteManager: Loading keypoints for stored route...");
    extractor = new SiftExtractor(SS0->getWidth(), SS0->getHeight());
    ssKeysArray = new vector<Keypoint*>[numberSnapshots];
    for (int s=0; s<numberSnapshots; s++) {
        ostringstream oss;
        oss << snapshotDirName << "/" << setfill('0') << setw(3) << s <<".keys";
        extractor->loadKeys(oss.str(), ssKeysArray[s]);
    }
    adaptor.print("RouteManager: Done.");

    homingAlg = new FeatureHissAlg(SS0->getWidth());

    //
    // Initialization specific to Bayes filter.
    //

    PNG = config->getBool("RouteManager.png");
    STDEV_EXPECTED_MOVEMENT = 
            config->getDouble("RouteManager.stdevExpectedMovement");

    if (method == MEAS_MODEL || method == BAYES_FILTER) {

        bfRoute = new BFRoute(snapshotDirName);

        nP = bfRoute->nP;
        lastBel = new double[nP];
        bel = new double[nP];
        belBar = new double[nP];
        z = new double[nP];

        // Initialize last belief to position 0.
        lastBel[0] = 1;
        for (int p=1; p<nP; p++)
            lastBel[p] = 0;

        // Initialize the other arrays.
        for (int p=0; p<nP; p++) {
            bel[p] = 0;
            belBar[p] = 0;
            z[p] = 0;
        }

        /*
        // Initialize a SeqWindow for visualizing bel while the robot is
        // homing (we also use SeqWindows's for storing the current belief
        // in checkArrivedFromBel). 
        */
        pSequence = new ImgSequence(bfRoute->parentSeqDir);
    }
}

RouteManager::~RouteManager() {
    delete sSequence;
    delete extractor;
    for (int s=0; s<numberSnapshots; s++) {
        for (int k=0; k<ssKeysArray[s].size(); k++)
            delete ssKeysArray[s][k];
    }

    if (method == MEAS_MODEL || method == BAYES_FILTER) {
        delete bfRoute;
        delete [] lastBel;
        delete [] bel;
        delete [] belBar;
        delete [] z;
        delete pSequence;
    }
}

double RouteManager::getHomeAngle(double displacement, 
                                  bool &arrived, bool &routeComplete) {
    trialIndex++;
    arrived = false;
    routeComplete = false;

    adaptor.getImg(CV);
    saveCV();

    // Extract keys from CV and compute home angle.
    extractor->clearKeys(cvKeys);
    extractor->extract(*CV, cvKeys);
    vector<Match> matchesCur;
    extractor->match(ssKeysArray[curGoalG], cvKeys, matchesCur);
    double homeAngle = homingAlg->getHomeAngle(ssKeysArray[curGoalG], cvKeys,
                                               matchesCur);
    ostringstream oss;
    oss << "RouteManager: homeAngle: " << homeAngle;
    adaptor.print(oss.str());

    //
    // Now use one of a variety of methods to detect arrival...
    //

    // Simple similarity threshold.
    if (method == SIMILARITY_THRESHOLD) {
        methodSimilarityThreshold(cvKeys, matchesCur, arrived);

    // Current similarity should be greater than both the predecessor and the
    // successor snapshots (better than the successor for the first node and
    // better than the predecessor for the last).
    } else if (method == LOCAL_MAX) {
        methodLocalMax(cvKeys, matchesCur, arrived);

    // Apply the measurement model half of Bayes filter only.  This method is
    // intended for test purposes and is applied for all possible positions.
    // Therefore it is quite slow.
    } else if (method == MEAS_MODEL) {
        methodMeasModel(cvKeys, matchesCur, arrived);

    // Apply the full Bayes filter.
    } else if (method == BAYES_FILTER) {
        methodBayesFilter(displacement, cvKeys, matchesCur, arrived);
    }

    if (arrived) {
        if (curGoalG == numberSnapshots-1) {
            adaptor.print("RouteManager: Route complete!");
            routeComplete = true;
        } else {
            curGoalG++;
        }
    }
    if (!routeComplete) {
        ostringstream oss;
        oss << "RouteManager: Homing to " << curGoalG;
        adaptor.print(oss.str());
    }

    return homeAngle;
}

void RouteManager::saveCV() {
    // Save the CV image to the current directory.
    ostringstream oss;
    oss << setfill('0') << setw(3) << trialIndex << ".bmp";
    CV->save(oss.str());
}

void RouteManager::methodSimilarityThreshold(vector<Keypoint*> &cvKeys,
                                             vector<Match> &matches,
                                             bool &arrived)
{
    double similarity = matches.size() / (double) ssKeysArray[curGoalG].size();

    if (similarity > SIMILARITY_THRESHOLD_VALUE)
        arrived = true;
}

void RouteManager::methodLocalMax(vector<Keypoint*> &cvKeys,
                                  vector<Match> &matches,
                                  bool &arrived)
{
    double similarity = matches.size() / (double) ssKeysArray[curGoalG].size();

    double prevSimilarity, nextSimilarity;
    ostringstream oss;
    oss << "sim: " << similarity << " ";

    if (curGoalG == 0) {
        // Compare similarity to that of the second snapshot.
        matches.clear();
        extractor->match(ssKeysArray[1], cvKeys, matches);
        nextSimilarity = matches.size() / (double) ssKeysArray[1].size();
        oss << "next: " << nextSimilarity;
        if (similarity > nextSimilarity)
            arrived = true;
    } else if (curGoalG == numberSnapshots-1) {
        // Compare similarity to that of the second-last snapshot.
        matches.clear();
        extractor->match(ssKeysArray[numberSnapshots-2], cvKeys, matches);
        prevSimilarity = matches.size() / 
                         (double) ssKeysArray[numberSnapshots-2].size();
        oss << "prev: " << prevSimilarity;
        if (similarity > prevSimilarity)
            arrived = true;
    } else {
        matches.clear();
        extractor->match(ssKeysArray[curGoalG-1], cvKeys, matches);
        prevSimilarity = matches.size() /
                         (double) ssKeysArray[curGoalG-1].size();
        matches.clear();
        extractor->match(ssKeysArray[curGoalG+1], cvKeys, matches);
        nextSimilarity = matches.size() /
                         (double) ssKeysArray[curGoalG+1].size();
        oss << "prev: " << prevSimilarity
            << ", next: " << nextSimilarity;
        if (similarity > prevSimilarity && similarity > nextSimilarity)
            arrived = true;
    }
    adaptor.print(oss.str());
}

void RouteManager::methodMeasModel(vector<Keypoint*> &cvKeys,
                                   vector<Match> &matchesCur,
                                   bool &arrived)
{
    // Apply the measurement model to update z for p-positions corresponding
    // to curGoalG
    applyMeasModel(cvKeys, matchesCur);

    // Now copy from z into bel.
    for (int p=0; p<nP; p++)
        bel[p] = z[p];
    visualize(bel, "bel");

    checkArrivedFromBel(arrived);
}

void RouteManager::checkArrivedFromBel(bool &arrived) {
    // Find the maximum entry in bel (hopefully this is where we are).
    double maxBel = -1;
    maxBelP = -1;
    for (int p=0; p<nP; p++) {
        //ostringstream oss;
        //oss << "bel[" << p << "]: " << bel[p];
        //adaptor.print(oss.str());
        if (bel[p] > maxBel) {
            maxBel = bel[p];
            maxBelP = p;
        }
    }

    ostringstream oss;
    oss << "RouteManager: maxBelP: " << maxBelP;
    adaptor.print(oss.str());

    // If the maximum corresponds to the position before the current
    // snapshot, or a later position, assume we have arrived.
//    if (maxBelP >= bfRoute->s2p[curGoalG] - 1)
//        arrived = true;
    if (maxBelP == bfRoute->g2p[curGoalG]) {
        arrived = true;
        adaptor.print("RouteManager: arrived!");
    }
}

void RouteManager::methodBayesFilter(double displacement,
                                     vector<Keypoint*> &cvKeys,
                                     vector<Match> &matchesCur,
                                     bool &arrived)
{
    // Apply prediction step of Bayes filter.
    for (int p=0; p<nP; p++) {
        belBar[p] = 0;
        for (int lastP=0; lastP<nP; lastP++) {
            belBar[p] += motionModel(displacement, lastP, p) * lastBel[lastP];
        }
    }
    visualize(belBar, "belBar");
        
    // Apply the measurement model to update z for p-positions corresponding
    // to curGoalG
    applyMeasModel(cvKeys, matchesCur);
    visualize(z, "z");

    // Now incorporate the results of the measurement model.
    double sumOfBel = 0;
    for (int p=0; p<nP; p++) {
        bel[p] = z[p] * belBar[p];
        sumOfBel += bel[p];
    }

    // Normalize bel.
    for (int p=0; p<nP; p++)
        bel[p] /= sumOfBel;

    visualize(bel, "bel");

    checkArrivedFromBel(arrived);

    // Exchange bel and lastBel.
    for (int p=0; p<nP; p++)
        lastBel[p] = bel[p];
}

void RouteManager::applyMeasModel(vector<Keypoint*> &cvKeys,
                                  vector<Match> &matchesCur)
{
    for (int p=0; p<nP; p++)
        z[p] = 0;

    // It would be prohibitively expensive to update all entries in z.  Instead
    // we update them in segments.  Each segment begins and ends with a g-node.
    // If the two g-nodes are adjacent then only those nodes are updated.  If
    // there are p-nodes between then these will also be updated.  Note that
    // updating these in-between p-nodes is a relatively low-cost operation
    // since no new matching has to be performed.  The constant
    // Z_SEGMENTS_RADIUS gives the radius of the range of additional segments
    // to update.  The segment from (curGoalG-1) to curGoalG is always updated
    // and a further 2*Z_SEGMENTS_RADIUS will be updated.
    //

    int gNextStart = std::max(1, curGoalG - Z_SEGMENTS_RADIUS);
    int gNextEnd = std::min(bfRoute->nS-1, curGoalG + Z_SEGMENTS_RADIUS);

    // Compute all necessary matches.  While we're at it, compute z for the g
    // nodes. 
    vector< vector<Match> > bigMatches;
    for (int g = gNextStart-1; g <=gNextEnd; g++) {

        vector<Match> matches;
        if (g == curGoalG)
            // We have already computed these matches.
            matches = matchesCur;
        else
            extractor->match(ssKeysArray[g], cvKeys, matches);
        bigMatches.push_back(matches);

        computeZForGNode(g, matches);
    }

    for (int gNext = gNextStart; gNext <=gNextEnd; gNext++) {
        int gPrev = gNext - 1;

        vector<Match> &matchesPrev = bigMatches[gNext - gNextStart];
        vector<Match> &matchesNext = bigMatches[gNext - gNextStart + 1];

        ostringstream oss, oss2;
        int pPrev = bfRoute->g2p[gPrev];
        adaptor.print(oss.str());

        // Now iterate through the intervening p-nodes (if any).
        int pStart = bfRoute->g2p[gPrev] + 1;
        int pEnd = bfRoute->g2p[gNext] - 1;
        if (pStart <= pEnd)
            computeZForPNodes(pStart, pEnd, gPrev, gNext,
                              matchesPrev, matchesNext);

        int pNext = bfRoute->g2p[gNext];
    }
}

void RouteManager::computeZForGNode(int g, vector<Match> &matches) {
    // The similarity is simply the fraction of matched features.
    int p = bfRoute->g2p[g];
    z[p] = matches.size() / (double)ssKeysArray[g].size();
    ostringstream oss;
    oss << "\tz[" << p << "]: " << z[p];
    adaptor.print(oss.str());
}

void RouteManager::computeZForPNodes(int pStart, int pEnd, int gPrev, int gNext,
                                     vector<Match> &matchesPrev,
                                     vector<Match> &matchesNext) {

    int nKeysPrev = ssKeysArray[gPrev].size();
    int nKeysNext = ssKeysArray[gNext].size();

    // Allocate and initialize the two match tables for the current image.
    int Cprev[nKeysPrev];
    int Cnext[nKeysNext];
    for (int k=0; k<nKeysPrev; k++)
        Cprev[k] = 0;
    for (int k=0; k<nKeysNext; k++)
        Cnext[k] = 0;

    // Fill in each table from the matches computed above.
    for (int q=0; q<matchesPrev.size(); q++)
        Cprev[matchesPrev[q].a]++;
    for (int q=0; q<matchesNext.size(); q++)
        Cnext[matchesNext[q].a]++;

    // The loop below will update z for the given section of p values.
    for (int p = pStart; p<=pEnd; p++) {

        // Find the number of matches recorded during training.
        // BAD: Pre-compute these.
        double trainMatchesPrev = 0, trainMatchesNext = 0;
        for (int k=0; k<nKeysPrev; k++)
            trainMatchesPrev += bfRoute->Mprev[p][k];
        for (int k=0; k<nKeysNext; k++)
            trainMatchesNext += bfRoute->Mnext[p][k];

        // Compute the similarity.  First form the dot products
        // between Mprev and Cprev and between Mcur and Ccur.
        double dotProductPrev = 0, dotProductNext = 0;
        for (int k=0; k<nKeysPrev; k++)
            dotProductPrev += bfRoute->Mprev[p][k] * Cprev[k];
        for (int k=0; k<nKeysNext; k++)
            dotProductNext += bfRoute->Mnext[p][k] * Cnext[k];

        z[p] = 0.5*(dotProductPrev/trainMatchesPrev +
                    dotProductNext/trainMatchesNext);

        ostringstream oss2;
        oss2 << "\tz[" << p << "]: " << z[p];
        adaptor.print(oss2.str());

        /*
        ostringstream oss;
        oss << "p: " << p << dotProductPrev << " / " <<trainMatchesPrev << " = "
            << (dotProductPrev/trainMatchesPrev) << ", " 
            << dotProductNext << " / " << trainMatchesNext << " = " 
            << (dotProductNext/trainMatchesNext);
        adaptor.print(oss.str());
        */

        // The following possibility would not require the match tables to be
        // stored.  However, it doesn't work as well.
        //z[p] = 0.5*(matchesPrev.size()/trainMatchesPrev +
        //            matchesCur.size()/trainMatchesCur);

        // The Zhang & Kleeman inspired interpolation approach.
//
//      if (curPIndex == prevPIndex + 1) {
//            z[p] = matchesCur.size() / (double) nKeysCur;
//        } else {
//            double curFactor = (p - prevPIndex) / 
//                               (double)(curPIndex - 1 - prevPIndex);
//            double prevFactor = 1.0 - curFactor;
//            z[p] = prevFactor * matchesPrev.size() / (double) nKeysPrev +
//                   curFactor * matchesCur.size() / (double) nKeysCur;
//
//            ostringstream oss;
//            oss << "\tp: " << p << ", prevFactor: " << prevFactor
//                << ", curFactor: " << curFactor;
//            adaptor.print(oss.str());
//        }
    }
}

double RouteManager::motionModel(double displacement, int lastP, int p) {

    // The displacement is p - lastP.  We subtract from this the observed
    // movement.
    double x = p - lastP - displacement;

    double stdev = STDEV_EXPECTED_MOVEMENT;
    return (1.0/(stdev*sqrt(2.0*M_PI))) * exp(-0.5*x*x/(stdev*stdev));
}

void RouteManager::visualize(double *inDist, string name) {
    // Make a copy of the input distribution and normalize it for display
    // purposes.
    double dist[nP];
    double maxDensity = 0;
    for (int p=0; p<nP; p++) {
        dist[p] = inDist[p];
        if (dist[p] > maxDensity) maxDensity = dist[p];
    }
    if (maxDensity > 1e-5)
        for (int p=0; p<nP; p++)
            dist[p] /= maxDensity;
    
    // Create a window for display
    ostringstream oss;
    oss << name << setfill('0') << setw(3) << trialIndex;
    double minX = sSequence->getMinX();
    double minY = sSequence->getMinY();
    double maxX = sSequence->getMaxX();
    double maxY = sSequence->getMaxY();
    double maxDim = maxX - minX;
    if (maxY - minY > maxDim) maxDim = maxY - minY;
    double margin = maxDim * 0.15;
    SimWindow window(oss.str(), minX-margin, minY-margin,
                     maxX+margin, maxY+margin, PNG);

    // Fill in marks at snapshot positions
    double lineL = maxDim * 0.005;
    for (int s=0; s<bfRoute->nS; s++) {
        Pose sPose = sSequence->getTruePose(s);
        window.addLine(sPose.x-lineL, sPose.y, sPose.x+lineL, sPose.y);
        window.addLine(sPose.x, sPose.y-lineL, sPose.x, sPose.y+lineL);
    }

    // Draw circles sized according to the probability of the given distribution
    double radiusFactor = maxDim * 0.015;
    for (int p=0; p<nP; p++) {
        //ostringstream oss;
        //oss << "\tdist[" << p << "]: " << dist[p];
        //adaptor.print(oss.str());

        Pose currentPose = pSequence->getTruePose(p);
        double radius = radiusFactor * dist[p];
        window.plotCircle(currentPose.x, currentPose.y, radius);
    }

    // Draw the robot's estimated position from odometry.
//    window.addPoint(adaptor.getX(), adaptor.getY(), 5);
}
