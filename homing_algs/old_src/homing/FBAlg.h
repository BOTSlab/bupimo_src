/**
 * Route following method based on defining vectors F (front) and B (back).
 *
 * \author Andrew Vardy
 */

#ifndef FBALG_H
#define FBALG_H

#include "TotalHomingAlg.h"
#include "SiftExtractor.h"
#include "TwoVec.h"
#include "ImgWindow.h"
#include "rng.h"

class FBAlg : public TotalHomingAlg {
public:
    FBAlg(int inWidth, int inHeight);
    virtual ~FBAlg();
    virtual void snapshot(Img* inSS);
    virtual void getHomeAngle(Img* CV, double &homeAngle, double &similarity);

private:
    void computeForB(bool computingF,
                     vector<Keypoint*> &ssKeys, vector<Keypoint*> &cvKeys,
                     TwoVec &v, TwoVec &ssV, int &nMatches);
    void frontOrderFilter(vector<Keypoint*> &ssKeys, vector<Keypoint*> &cvKeys,
                          vector<Match> &matches);
    void extractSubsetsDP(vector<Keypoint*> &ssKeys, vector<Match> &matches,
                          bool addPi);
    void extractSubsetsRandom(vector<Keypoint*> &ssKeys, vector<Match> &matches,
                              bool addPi);
    static bool compareFrontKeys(const Keypoint *key1, const Keypoint *key2);
    static bool compareFrontMatches(const Match m1, const Match m2);

    Img *ssImg, *cvImg;
    int width, height;
    SiftExtractor extractor;

    vector<Keypoint*> ssFrontKeys, ssBackKeys;

    // Parameters.
    double includeAngle, stdevs;
    bool deviationFilter, orderFilter, subtractAngleAtS, applySubsets,
         forwardContracted, backwardExpanded,
         outputZhang, outputF, outputB, outputR, outputMix, debug;
    int resolution;

    RNG rng;

    ImgWindow *window;

    static int staticWidth;
    static int staticStartX;
    static vector<Keypoint*> staticCVKeys;
};

#endif
