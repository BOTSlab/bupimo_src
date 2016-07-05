#include <sstream>
#include <vector>
#include "key.h"
#include "Img.h"

#ifndef SIFTHOMING_H
#define SIFTHOMING_H

using namespace std;

typedef struct MatchPairSt {
	struct KKeypointSt *CV;
	struct KKeypointSt *SS;
	struct MatchPairSt *next;
} * MatchPair;

class SIFTHoming {

public:
	SIFTHoming();
	SIFTHoming(string CV, string SS);
	SIFTHoming(KKeypoint CVkeys, KKeypoint SSkeys, int w, int h);
	SIFTHoming(string CV, KKeypoint SSkeys);
	SIFTHoming(float ** CVpixels, float ** SSpixels, int w, int h);

	// Added by AV
	void setConfig(int config);

	void SetSS(float ** SSpixels, int w, int h);

	// Added by AV
	void SetSS(Img *SS);

	void SetCV(float ** CVpixels, int w, int h);

	// Added by AV
	void SetCV(Img *CV);

	/**
     * Obtain the horizontal image pixel position which corresponds to the home
     * direction.  Also places the percentage of keypoints matched from CV to
     * SS in matchFraction.  This function operates on the SS and CV images
     * previously set by SetSS and SetCV.
     */
    void getHomePixel(int &pixel, double &matchFraction);

    /**
     * Obtain the percentage of matched keypoints from image A to image B.
     */
    double getMatchFraction(Img *A, Img *B);

	~SIFTHoming();

private:

	int imageWidth, imageHeight;
	KKeypoint SSkeys, CVkeys;
	MatchPair matches;

	vector< vector<float> > CVpix;
	vector< vector<float> > SSpix;

    // The number of keypoints found for the last CV or SS image.
    int nSSkeys, nCVkeys;

    // Configuration parameter.  0 indicates that both expanded and contracted
    // features be used.  1 indicates only contracted features are used.
    // 2 indicates only expanded features.
    int config;

	int GetScore(KKeypoint k1, KKeypoint k2);
	int CountKeys(KKeypoint k);
	int CountMatches(MatchPair m);

	double correctAngle(double a);
	double angleDiff(double a1, double a2);

	MatchPair GetMatches(KKeypoint CV, KKeypoint SS);
	MatchPair MatchPair_Create();
	MatchPair MakePair(KKeypoint CV, KKeypoint SS, MatchPair m);

	vector<double> toCircle(double x);
	vector<double> GetMeans(MatchPair matches);

	void freeImage(Image im);
};

#endif
