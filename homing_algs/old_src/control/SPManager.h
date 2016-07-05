/**
 * A RouteManager selects the snapshot image to home to for
 * VisualHomingController.
 *
 * @Andrew Vardy
 */

#ifndef ROUTEMANAGER_H
#define ROUTEMANAGER_H

#include "DriverAdaptor.h"
#include "Img.h"
#include "ImgSequence.h"
#include "FeatureHomingAlg.h"
#include "BFRoute.h"
#include "SimWindow.h"

class RouteManager {
public:
    RouteManager(DriverAdaptor &inAdaptor);

    virtual ~RouteManager();

    /**
     * Compute and return the home angle.  Also, determine if the robot has
     * arrived at its current snapshot position and if the route is complete.
     */
    double getHomeAngle(double displacement,
                        bool &arrived, bool &routeComplete);

private:
    void saveCV();
    void methodSimilarityThreshold(vector<Keypoint*> &cvKeys,
                                   vector<Match> &matches, bool &arrived);
    void methodLocalMax(vector<Keypoint*> &cvKeys,
                        vector<Match> &matches, bool &arrived);
    void methodMeasModel(vector<Keypoint*> &cvKeys,
                         vector<Match> &matches, bool &arrived);
    void methodBayesFilter(double displacement, vector<Keypoint*> &cvKeys,
                           vector<Match> &matches, bool &arrived);
    // Helper methods specific to methodMeasModel and methodBayesFilter
    double motionModel(double displacement, int lastP, int p);
    void applyMeasModel(vector<Keypoint*> &cvKeys, vector<Match> &matches);
    void computeZForGNode(int g, vector<Match> &matches);
    void computeZForPNodes(int pStart, int pEnd, int gPrev, int gNext,
                           vector<Match> &matchesPrev,
                           vector<Match> &matchesNext);
    void checkArrivedFromBel(bool &arrived);
    void visualize(double *distribution, string name);

/*
    : adaptor(inAdaptor),
      curGoalG(1),
      method(BAYES_FILTER),
      CV(NULL),
      trialIndex(-1),
      maxBelP(0),
      SIMILARITY_THRESHOLD_VALUE(0.29),
      Z_SEGMENTS_RADIUS(1)
*/
    DriverAdaptor &adaptor;
    int curGoalG;
    enum MethodType { SIMILARITY_THRESHOLD, LOCAL_MAX, MEAS_MODEL,
                      BAYES_FILTER };
    MethodType method;
    Img *CV;
    int trialIndex, maxBelP;

    // If the similarity between the current and snapshot images exceeds this
    // threshold then we assume we have arrived.
    const double SIMILARITY_THRESHOLD_VALUE;

    int Z_SEGMENTS_RADIUS;
    bool PNG;
    double STDEV_EXPECTED_MOVEMENT;


    int numberSnapshots;
    ImgSequence *sSequence;

    // SiftExtractor and vector of features for the current view image.
    SiftExtractor *extractor;
    vector<Keypoint*> cvKeys;

    // A vector of features extracted from each snapshot image goes in each
    // entry of the following array. 
    vector<Keypoint*> *ssKeysArray;

    FeatureHomingAlg *homingAlg;

    BFRoute *bfRoute;
    int nP;

    // Data structures specific to Bayes filter.
    double *lastBel, *bel, *belBar, *z;

    ImgSequence *pSequence;
};


#endif
