#include "FeatureHissAlg.h"
#include "Angles.h"
#include "SingleConfig.h"

FeatureHissAlg::FeatureHissAlg(int inWidth) : width(inWidth)
{
    SingleConfig *config = SingleConfig::getInstance();

    useCon = config->getBool("FeatureHissAlg.useCon", true);
    useExp = config->getBool("FeatureHissAlg.useExp", true);
}

double FeatureHissAlg::getHomeAngle(vector<Keypoint*> &ssKeys,
                                    vector<Keypoint*> &cvKeys,
                                    vector<Match> &matches) {

    // For each match, classify the keypoint in the current image as either
    // contracted or expanded.
    vector<Keypoint*> contracted;
    vector<Keypoint*> expanded;
    for (int m=0; m<matches.size(); m++) {
        Keypoint *keyInSS = ssKeys[matches[m].a];
        Keypoint *keyInCV = cvKeys[matches[m].b];
        if (keyInCV->sigma > keyInSS->sigma)
            expanded.push_back(keyInCV);
        else if (keyInCV->sigma != keyInSS->sigma)
            contracted.push_back(keyInCV);
    }
    int nCon = (int) contracted.size(); 
    int nExp = (int) expanded.size(); 

    // Compute the angular mean of these two feature sets.
    float cosSum = 0;
    float sinSum = 0;
    for (int i=0; i<nCon; i++) {
        Keypoint *key = contracted[i];
        double theta = TWO_PI * (1.0 - key->x / width);
        cosSum += cos(theta);
        sinSum += sin(theta);
    }
    float bearingCon = atan2(sinSum, cosSum);
    sinSum = 0;
    cosSum = 0;
    for (int i=0; i<nExp; i++) {
        Keypoint *key = expanded[i];
        double theta = TWO_PI * (1.0 - key->x / width);
        cosSum += cos(theta);
        sinSum += sin(theta);
    }
    float bearingExp = atan2(sinSum, cosSum);

    // Finally compute the weighted average.
    float s, c;
    if (useCon && useExp) {
        s = nCon * sin(bearingCon) + nExp * sin(bearingExp + M_PI);
        c = nCon * cos(bearingCon) + nExp * cos(bearingExp + M_PI);
    } else if (!useCon && useExp) {
        s = sin(bearingExp + M_PI);
        c = cos(bearingExp + M_PI);
    } else if (useCon && !useExp) {
        s = sin(bearingCon);
        c = cos(bearingCon);
    } else {
        assert(false);
    }
    return atan2(s, c);
}
