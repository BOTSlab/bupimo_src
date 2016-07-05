/**
 * RouteManager that uses either the local maximum or discrete Bayes filter
 * (depending on 'useBayes' as localization filter.
 *
 * @Andrew Vardy
 */

#ifndef MYMANAGER_H
#define MYMANAGER_H

#include "RouteManager.h"
#include "DriverAdaptor.h"
#include "Img.h"
#include "SiftExtractor.h"
#include "ImgSequence.h"
#include "TotalHomingAlg.h"
#include "SimWindow.h"

class MyManager : public RouteManager {
public:
    MyManager(DriverAdaptor &inAdaptor);

    virtual ~MyManager();

    virtual double getHomeAngle(double displacement,
                                bool &arrived, bool &routeComplete);

private:
    void saveCV();
    double similarity(int i, vector<Match> &matchesA, vector<Match> &matchesB);
    void updateBeliefLocalMax(bool &arrived);
    void updateBeliefBayes(double displacement, bool &arrived);
    void checkArrived(double *belief, bool &arrived);
    double motionModel(int lastP, int p);
    void applyMeasModel();
    void storeAndVisualize(double *inDist, string title);

    DriverAdaptor &adaptor;

    // Current position of the robot along the route.  Note that this is an
    // index of the space between snapshots along the route.  Thus, when
    // posIndex == 0 we are actually homing to snapshot 1.
    int posIndex;

    Img *CV;
    int trialIndex;

    // Configurable parameters.
    bool useBayes, scaleMeasure, advanceSnapshot;
    double meanMovement, stdevMovement;
    int measModelRadius;

    int nSnapshots, nPositions;
    ImgSequence *sSeq;

    // SiftExtractor and vector of features for the current view image.
    SiftExtractor *extractor;

    TotalHomingAlg *homingAlg;

    // Data structures used to represent our belief in the robot's position
    // (explained in the .cpp file).
    double *lastBel, *bel, *belBar, *probZ;

    double pointSize, radiusConstant, plusCrossSize;
};

#endif
