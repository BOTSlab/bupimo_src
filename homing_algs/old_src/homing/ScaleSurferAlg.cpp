#include "ScaleSurferAlg.h"

ScaleSurferAlg::ScaleSurferAlg(int inWidth, int height) 
    : width(inWidth), extractor(width, height)
{
}

ScaleSurferAlg::~ScaleSurferAlg() {
    clearFeatureVector(ssFeatures);
    clearFeatureVector(cvFeatures);
}

void ScaleSurferAlg::clearFeatureVector(vector<Keypoint*> &features) {
    // Clear out any old features.
    int nOld = features.size();
    if (nOld > 0) {
        for (int i=0; i<nOld; i++)
            delete features[i];
        features.clear();
    }
}
    
void ScaleSurferAlg::snapshot(Img* SS) {
    clearFeatureVector(ssFeatures);

    // We wish to extract features lying in the front hemisphere of the image
    // from [-width/4 to width/4].  We do this here by extracting features
    // from the whole snapshot image and then discarding those that lie outside
    // this region (it would be good to consider extracting from part of the
    // image directly).
    extractor.extract(*SS, ssFeatures);

    cutBackFeatures(ssFeatures);
}

void ScaleSurferAlg::cutBackFeatures(vector<Keypoint*> &features) {
    double leftLimit = 3.0 * width / 4.0;
    double rightLimit = width / 4.0;
    
    vector<Keypoint*>::iterator it = features.begin();
    while (it != features.end()) {
        Keypoint *key = *it;
        if (!(key->x >= leftLimit || key->x <= rightLimit)) {
            it = features.erase(it);
            delete key;
            features.begin();
        } else {
            it++;
        }
    }
}

void ScaleSurferAlg::getHomeAngle(Img* CV, double &homeAngle, double &similarity) {

    clearFeatureVector(cvFeatures);
    extractor.extract(*CV, cvFeatures);
    //cout << "ScaleSurferAlg: keypoints in CV: " << cvFeatures.size() << endl;

    // Find matches from the current to the snapshot image.
    vector<Match> matches;
    extractor.match(cvFeatures, ssFeatures, matches); 
    //cout << "ScaleSurferAlg: matches: " << matches.size() << endl;

    // For each match, classify the keypoint in the current image as either
    // contracted or expanded.
    vector<Keypoint*> contracted;
    vector<Keypoint*> expandedRight, expandedLeft;
    for (int m=0; m<matches.size(); m++) {
        Keypoint *keyInCV = cvFeatures[matches[m].a];
        Keypoint *keyInSS = ssFeatures[matches[m].b];
        if (keyInCV->sigma > keyInSS->sigma) {
            if (keyInSS->x <= width/4.0)
                expandedRight.push_back(keyInCV);
            else
                expandedLeft.push_back(keyInCV);
        } else if (keyInCV->sigma != keyInSS->sigma)
            contracted.push_back(keyInCV);
    }
    int nCon = (int) contracted.size(); 
    int nExpLeft = (int) expandedLeft.size(); 
    int nExpRight = (int) expandedRight.size(); 

    // Compute the angular mean of these two feature sets.
    float cosSum = 0;
    float sinSum = 0;
    for (int i=0; i<nCon; i++) {
        Keypoint *key = contracted[i];
        double theta = TWO_PI * (1 - key->x / width);
        cosSum += cos(theta);
        sinSum += sin(theta);
    }

    // Expanded left
    for (int i=0; i<nExpLeft; i++) {
        Keypoint *key = expandedLeft[i];
        double theta = TWO_PI * (1 - key->x / width);
        cosSum += cos(theta - PI_OVER_2);
        sinSum += sin(theta - PI_OVER_2);
    }

    for (int i=0; i<nExpRight; i++) {
        Keypoint *key = expandedRight[i];
        double theta = TWO_PI * (1 - key->x / width);
        cosSum += cos(theta + PI_OVER_2);
        sinSum += sin(theta + PI_OVER_2);
    }

    homeAngle = atan2(sinSum, cosSum);

    // Determine the similarity.
    if (nCon == 0 && nExpLeft == 0 && nExpRight == 0)
        similarity = 0;
    else
        similarity = (double) matches.size() / cvFeatures.size();
}

double ScaleSurferAlg::getSimilarityCVTo(Img* B) {
    clearFeatureVector(bFeatures);
    extractor.extract(*B, bFeatures);
    cutBackFeatures(bFeatures);

    // Find matches from CV to B
    vector<Match> matches;
    extractor.match(cvFeatures, bFeatures, matches); 

    return matches.size() / (double) cvFeatures.size();
}

double ScaleSurferAlg::getSimilarity(Img* A, Img* B) {
    clearFeatureVector(aFeatures);
    extractor.extract(*A, aFeatures);
    if (aFeatures.size() == 0)
        return 0;

    clearFeatureVector(bFeatures);
    extractor.extract(*B, bFeatures);

    // Find matches from A to B
    vector<Match> matches;
    extractor.match(aFeatures, bFeatures, matches); 

    return matches.size() / (double) aFeatures.size();
}
