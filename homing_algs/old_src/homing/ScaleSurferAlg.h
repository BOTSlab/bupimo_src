/**
 * EXPERIMENTAL
 *
 * Requirements: VLFeat
 *
 * @author Andrew Vardy
 */

#ifndef SCALESURFERALG_H
#define SCALESURFERALG_H

#include "TotalHomingAlg.h"
#include "SiftExtractor.h"
#include "Angles.h"

class ScaleSurferAlg : public TotalHomingAlg {
public:
    ScaleSurferAlg(int inWidth, int height);
    virtual ~ScaleSurferAlg();
    virtual void snapshot(Img* SS);
    virtual void getHomeAngle(Img* CV, double &homeAngle, double &similarity);
    virtual double getSimilarityCVTo(Img* B);
    virtual double getSimilarity(Img* A, Img* B);
private:
    void clearFeatureVector(vector<Keypoint*> &features);
    void cutBackFeatures(vector<Keypoint*> &features);

    int width;
    SiftExtractor extractor;
    vector<Keypoint*> ssFeatures;
    vector<Keypoint*> cvFeatures;

    // Used for getSimilarity.
    vector<Keypoint*> aFeatures;
    vector<Keypoint*> bFeatures;
};

#endif
