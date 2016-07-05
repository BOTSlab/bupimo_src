#include "OldHissAlg.h"

OldHissAlg::OldHissAlg(int config) {
    siftHoming.setConfig(config);
}

OldHissAlg::~OldHissAlg() {
}

void OldHissAlg::snapshot(Img* SS) {
	siftHoming.SetSS(SS);
}

void OldHissAlg::getHomeAngle(Img* CV, double &homeAngle, double &similarity) {
	siftHoming.SetCV(CV);

    int homePixel;
    siftHoming.getHomePixel(homePixel, similarity);
	homeAngle = Angles::constrainAngle(
            TWO_PI * (1 - homePixel / (float) CV->getWidth()) - PI);
}

double OldHissAlg::getSimilarity(Img* A, Img* B) {
    return siftHoming.getMatchFraction(A, B);
}
