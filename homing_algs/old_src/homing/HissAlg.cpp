#include "HissAlg.h"
#include "SingleConfig.h"

HissAlg::HissAlg(int inWidth, int height) 
    : SS(NULL), width(inWidth), extractor(width, height)
{
    SingleConfig *config = SingleConfig::getInstance();

    useCon = config->getBool("HissAlg.useCon", true);
    useExp = config->getBool("HissAlg.useExp", true);
    deviationFilter = config->getBool("HissAlg.deviationFilter", false);
    stdevs = config->getDouble("HissAlg.stdevs", 1.0);
}

HissAlg::~HissAlg() {
}

void HissAlg::snapshot(Img* inSS) {
    SS = inSS;
    if (!SS->hasKeypoints())
        extractor.extract(*SS, SS->getKeypoints());
}

void HissAlg::getHomeAngle(Img* CV, double &homeAngle, double &similarity) {

    // Extract keypoints from CV, if they haven't already been extracted.
    if (!CV->hasKeypoints())
        extractor.extract(*CV, CV->getKeypoints());

    vector<Keypoint*> &ssKeys = SS->getKeypoints();
    vector<Keypoint*> &cvKeys = CV->getKeypoints();

    cout << "ssKeys.size(): " << ssKeys.size() << endl;
    cout << "cvKeys.size(): " << cvKeys.size() << endl;

    vector<Match> matches;
    extractor.match(ssKeys, cvKeys, matches);

    if (deviationFilter)
        extractor.deviationFilter(ssKeys, cvKeys, matches, stdevs);
    
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
    homeAngle = atan2(s, c);

    // Determine the similarity.
    if (nCon == 0 && nExp == 0)
        similarity = 0;
    else
        similarity = (double) matches.size() / cvKeys.size();
}
