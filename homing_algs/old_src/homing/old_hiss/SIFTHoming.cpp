// changed on
// Mon 24 Mar 2008 03:04:15 PM NDT

#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cassert>
#include <stdio.h>
#include <math.h>
#include "key.h"
#include "SIFTHoming.h"

// Blank constructor for SIFT Homing
// If this is used, setSS and setCV must be called before home()
SIFTHoming::SIFTHoming() {
    config = 0;
}

// Constructor based on PGM image filenames
SIFTHoming::SIFTHoming(string CV, string SS) {

	FILE * CVfile = fopen(CV.c_str(), "r");
	FILE * SSfile = fopen(SS.c_str(), "r");

	Image CVim = ReadPGM(CVfile);
	Image SSim = ReadPGM(SSfile);

	CVkeys = GetKeypoints(CVim);
	SSkeys = GetKeypoints(SSim);

	int nSSkeys = CountKeys(SSkeys);
	int nCVkeys = CountKeys(CVkeys);
	//cout << "There are " << nSSkeys << " SS keypoints." << endl;
	//cout << "There are " << nCVkeys << " CV keypoints." << endl;

	imageWidth = CVim->cols;
	imageHeight = CVim->rows;

	// This is Lowe's code which free's all created images
	// We no longer need them since we only work with the Keypoints
	FreeStoragePool(IMAGE_POOL);
    config = 0;
}

// Constructor based on two Keypoint structures, image width and height
// Useful for precomputed database homing
SIFTHoming::SIFTHoming(Keypoint CVk, Keypoint SSk, int w, int h) {

	SSkeys = SSk;
	CVkeys = CVk;
	imageWidth = w;
	imageHeight = h;
    config = 0;
}

// Constructed based on a CV image filename, and the SS Keypoints
// Useful for precomputed SS keypoints for database homing
SIFTHoming::SIFTHoming(string CV, Keypoint SSk) {

	FILE * CVfile = fopen(CV.c_str(), "r");

	Image CVim = ReadPGM(CVfile);

	CVkeys = GetKeypoints(CVim);
	SSkeys = SSk;

	imageWidth = CVim->cols;
	imageHeight = CVim->rows;

	FreeStoragePool(IMAGE_POOL);
    config = 0;
}

// Constructor based on float** arrays for both SS and CV
// Useful for performing on input for non-pgm image types, or from a camera
SIFTHoming::SIFTHoming(float ** CVpixels, float ** SSpixels, int w, int h) {

	Image CVim, SSim;

	CVim->cols = w;
	CVim->rows = h;
	SSim->cols = w;
	SSim->rows = h;

	CVim->pixels = CVpixels;
	SSim->pixels = SSpixels;

	CVkeys = GetKeypoints(CVim);
	SSkeys = GetKeypoints(SSim);

	imageWidth = CVim->cols;
	imageHeight = CVim->rows;

	FreeStoragePool(IMAGE_POOL);
    config = 0;
}

// Deconstructor
SIFTHoming::~SIFTHoming() {
	//cout << "SIFTHoming destructor." << endl;
}

void SIFTHoming::setConfig(int inConfig) {
    config = inConfig;
}

// Sets the CV image based on a float** array
void SIFTHoming::SetCV(float ** CVpixels, int w, int h) {

	CVpix.clear();
	Image CVim = CreateImage(h, w, IMAGE_POOL);
	imageWidth = w;
	imageHeight = h;

	for (int r=0; r<h; r++) {
		vector<float> f(w);
		CVpix.push_back(f);
		for (int c=0; c<w; c++) {
			CVim->pixels[r][c] = CVpixels[r][c];
			CVpix[r][c] = CVpixels[r][c];
		}
	}

	CVkeys = GetKeypoints(CVim);
	nCVkeys = CountKeys(CVkeys);
	//cout << "There are " << nCVkeys << " CV keypoints." << endl;
	FreeStoragePool(IMAGE_POOL);
}

void SIFTHoming::SetCV(Img *CV) {
	int w = CV->getWidth();
	int h = CV->getHeight();
	CVpix.clear();
	Image CVim = CreateImage(h, w, IMAGE_POOL);
	imageWidth = w;
	imageHeight = h;

	for (int r=0; r<h; r++) {
		vector<float> f(w);
		CVpix.push_back(f);
		for (int c=0; c<w; c++) {
			CVim->pixels[r][c] = CV->get(c, r);
			CVpix[r][c] = CV->get(c, r);
		}
	}

	CVkeys = GetKeypoints(CVim);
	nCVkeys = CountKeys(CVkeys);
	//cout << "There are " << nCVkeys << " CV keypoints." << endl;
	FreeStoragePool(IMAGE_POOL);
}

// Sets the SS image based on a float** array
void SIFTHoming::SetSS(float ** SSpixels, int w, int h) {

	//cout << w << "\t" << h << endl;

	SSpix.clear();
	Image SSim = CreateImage(h, w, IMAGE_POOL);
	imageWidth = w;
	imageHeight = h;

	for (int r=0; r<h; r++) {
		vector<float> f(w);
		SSpix.push_back(f);
	}

	for (int r=0; r<h; r++) {
		for (int c=0; c<w; c++) {
			SSim->pixels[r][c] = SSpixels[r][c];
			SSpix[r][c] = SSpixels[r][c];
		}
	}

	SSkeys = GetKeypoints(SSim);

	nSSkeys = CountKeys(SSkeys);
	//cout << "SIFTHoming:\t\tThere are " << nSSkeys << " SS keypoints." << endl;
	FreeStoragePool(IMAGE_POOL);
}

void SIFTHoming::SetSS(Img *SS) {
	int w = SS->getWidth();
	int h = SS->getHeight();

	SSpix.clear();
	Image SSim = CreateImage(h, w, IMAGE_POOL);
	imageWidth = w;
	imageHeight = h;

	for (int r=0; r<h; r++) {
		vector<float> f(w);
		SSpix.push_back(f);
	}

	for (int r=0; r<h; r++) {
		for (int c=0; c<w; c++) {
			SSim->pixels[r][c] = SS->get(c, r);
			SSpix[r][c] = SS->get(c, r);
		}
	}

	SSkeys = GetKeypoints(SSim);

	nSSkeys = CountKeys(SSkeys);
	//cout << "SIFTHoming:\t\tThere are " << nSSkeys << " SS keypoints." << endl;
	FreeStoragePool(IMAGE_POOL);
}

void SIFTHoming::getHomePixel(int &pixel, double &matchFraction) {

	//cout << "SIFTHoming:\t\tCalculating homing angles..." << endl;

	matches = GetMatches(CVkeys, SSkeys);

    // AV
    int nMatches = CountMatches(matches);
	//cout << "SIFTHoming:\t\tThere are " << nMatches << " matches." << endl;

    matchFraction = nMatches / (double) max(nCVkeys, nSSkeys);

	vector<double> means = GetMeans(matches);

	double green = means[0];
	double red = (int)(means[1] + imageWidth/2) % imageWidth;

    double wX = 0, wY = 0;
    if (config == 0) {
        // Both expanded and contracted features are used.  The home direction
        // is given by the weighted mean of angles red and green.
        vector<double> rp = toCircle(red);
        vector<double> gp = toCircle(green);
        wX = rp[0]*means[3] + gp[0]*means[2];
        wY = rp[1]*means[3] + gp[1]*means[2];
    } else if (config == 1) {
        // Only contracted features are used.
        vector<double> gp = toCircle(green);
        wX = gp[0];
        wY = gp[1];
    } else if (config == 2) {
        // Only expanded features are used.
        vector<double> rp = toCircle(red);
        wX = rp[0];
        wY = rp[1];
    } else {
        // Shouldn't get here.
        assert(false);
    }

	/* The angular mean of the data
	-PI is the left of the image, PI is the right of the image, 0 the center */
	double ret = correctAngle(atan2(wY,wX));

	/* We now convert the the angle to 0 for the left, 2PI for the right */
	if (ret < 0) ret += 2*PII;

	/* Find which pixel that is within the image*/
	pixel = (int)(( (ret) / (2*PII) ) * imageWidth);
}

double SIFTHoming::getMatchFraction(Img *A, Img *B) {
	int w = A->getWidth();
	int h = A->getHeight();

    // Copy both images into Lowe's Image structure.
    //cout << "\tcopying images" << endl;
	Image Aim = CreateImage(h, w, IMAGE_POOL);
	Image Bim = CreateImage(h, w, IMAGE_POOL);
	for (int r=0; r<h; r++)
		for (int c=0; c<w; c++) {
			Aim->pixels[r][c] = A->get(c, r);
			Bim->pixels[r][c] = B->get(c, r);
        }

    // Obtain keypoints for both
    //cout << "\tobtaining keypoints" << endl;
	Keypoint Akeys = GetKeypoints(Aim);
if (Akeys == NULL)
cout << "\tAkeys NULL!" << endl;
	Keypoint Bkeys = GetKeypoints(Bim);
if (Bkeys == NULL)
cout << "\tBkeys NULL!" << endl;
	int nAkeys = CountKeys(Akeys);
    //cout << "\tA keys: " << nAkeys << endl;
	int nBkeys = CountKeys(Bkeys);
    //cout << "\tB keys: " << nBkeys << endl;

    // Compute the matches.
    //cout << "\tcomputing matches" << endl;
	matches = GetMatches(Akeys, Bkeys);
    int nMatches = CountMatches(matches);
    //cout << "\tmatches: " << nMatches << endl;

	FreeStoragePool(IMAGE_POOL);

    return nMatches / (double) max(nAkeys, nBkeys);
}

// Gets the angular means of both contracted and expanded features from the matches
vector<double> SIFTHoming::GetMeans(MatchPair matches)
{

	double gX = 0, gY = 0, rX = 0, rY = 0, nG = 0, nR = 0;
	MatchPair m;
	for (m = matches; m->next != NULL; m=m->next) {

		double diff = m->CV->scale - m->SS->scale;

		if (diff > 0) {
			vector<double> p = toCircle(m->CV->col);
			gX += p[0]; gY += p[1];
			nG++;
		} else if (diff < 0) {
			vector<double> p = toCircle(m->CV->col);
			rX += p[0]; rY += p[1];
			nR++;
		}
	}

	/* Figure out the winning location of the green (contraction) heads */
	double gAngle = atan2(gY,gX);
	if (gAngle < 0) gAngle = PII*2 + gAngle;
	double gRatio = gAngle / (PII*2);
	double green = gRatio * imageWidth;

	double rAngle = atan2(rY,rX);
	if (rAngle < 0) rAngle = PII*2 + rAngle;
	double rRatio = rAngle / (PII*2);
	double red = rRatio * imageWidth;

	vector<double> ret(4);
	ret[0] = green; ret[1] = red; ret[2] = nG; ret[3] = nR;
	return ret;
}

// Returns the matches from two Keypoint structs
MatchPair SIFTHoming::GetMatches(Keypoint CV, Keypoint SS) {

	MatchPair matches = MatchPair_Create();
	Keypoint k, s;

	/* For each Keypoint in CV, find its match in SS */
	for (k = CV; k->next != NULL; k=k->next) {

		// max theoretical value for diff of k,s
		int min = 8323201, min2 = 8323201;
		Keypoint minkey = NULL;

		for (s=SS; s->next!=NULL; s=s->next) {

			int score = GetScore(k, s);
			if (score < min) {
				min2 = min;
				min = score;
				minkey = s;
			} else if (score < min2) {
				min2 = score;
			}
		}

		if (10 * 10 * min < 8 * 8 * min2) {
			// if it is still the head of the list, simply set it to a new one
			if (matches->CV == NULL)
				matches = MakePair(k,minkey,NULL);
			else
				matches = MakePair(k,minkey,matches);
		}
	}

	//cout << "Matches found:\t" << CountMatches(matches) << endl;
	return matches;
}

// Creates a MatchPair struct
MatchPair SIFTHoming::MatchPair_Create() {

	MatchPair matches = (struct MatchPairSt *) malloc(sizeof(struct MatchPairSt));
	matches->CV = NULL;
	matches->SS = NULL;
	matches->next = NULL;
	return matches;
}

// Creates a MatchPair from two keypoints
MatchPair SIFTHoming::MakePair(Keypoint CV, Keypoint SS, MatchPair m) {

	MatchPair match = MatchPair_Create();
	match->CV = CV;
	match->SS = SS;
	match->next = m;
	return match;
}

// Returns the SIFT match score of two Keypoints
int SIFTHoming::GetScore(Keypoint k1, Keypoint k2) {

	int i;
	int score = 0;
	int diff = 0;
	for (i=0; i<VecLength; i++) {
		diff = k1->ivec[i] - k2->ivec[i];
		score += diff * diff;
	}
	return score;
}

// Converts a pixel value into polar coordinates
vector<double> SIFTHoming::toCircle(double x) {

	vector<double> p(2);
	double angle = (x / imageWidth) * (PII * 2);
	p[0] = cos(angle);
	p[1] = sin(angle);

	return p;
}

// Counts the number of keys in a Keypoint linked list
int SIFTHoming::CountKeys(Keypoint keys)
{
	int num = 1;
	Keypoint k;
	for (k = keys; k->next != NULL; k=k->next) 
        num++;
	return num;
}

// Counts the number of matches in a MatchPair linked list
int SIFTHoming::CountMatches(MatchPair matches)
{
// AV
if (matches->next == NULL) {
//cout << "NO MATCHES" << endl;
return 0;
}
	int num = 1;
	MatchPair m;
	for (m = matches; m->next != NULL; m=m->next) num++;
	return num;
}

// Free's a Lowe Image
void SIFTHoming::freeImage(Image im) {

	for (int i=0; i<im->cols; i++)
		free(im->pixels[i]);
	free(im->pixels);
	free(im);
}

// Converts any angle to -PI to PI range
double SIFTHoming::correctAngle(double a) {
	while (a < -PII) a += 2*PII;
	while (a >  PII) a -= 2*PII;
	return a;
}

