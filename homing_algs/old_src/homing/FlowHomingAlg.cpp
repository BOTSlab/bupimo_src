/* A visual homing algorithm using optic flow
 * written by Robert Hamilton
 */

#include "FlowHomingAlg.h"
#include "Imgdb.h"
#include "ImgOps.h"
#include "ImgWindow.h"
#include "SingleConfig.h"
#include <cv.h>
#include <sstream>
#include <fstream>

int FlowHomingAlg::debugCount = 0;
const bool debug = false;
const int numTemps = 128;

FlowHomingAlg::FlowHomingAlg(int inWidth, int inHeight, double inMaxY):
	SS(NULL),
	bestScoreImage(NULL),
	bestAngleImage(NULL),
	//bestU(NULL),
	//bestV(NULL),
	bsi(false),
	width(inWidth),
	height(inHeight),
	level2Width(inWidth/2),
	level2Height(inHeight/2),
	level3Width(inWidth/4),
	level3Height(inHeight/4),
	//debug(true),
	rotCV(NULL),
	maxy(inMaxY),
	templates(new Img*[numTemps])
{
	/*if (debug)
		imgWindow = new ImgWindow("FlowHomingAlg");*/

	if (debug){
		bestU = new Img(level3Width, level3Height);
		bestV = new Img(level3Width, level3Height);
	}
	
	//Alocate the three-dimensional array for pixel weights
	weights = new double**[numTemps];
	for (int a = 0 ; a < numTemps ; a++){
		weights[a] = new double*[level3Width];
		for (int b = 0 ; b < level3Width ; b++){
			weights[a][b] = new double[level3Height];
		}
	}

	
	templates[0] = new Img(level3Width, level3Height);

	//Calculate the flow template and weights
	for (int i = 0 ; i < level3Width ; i++){
		for (int j = 0 ; j < level3Height ; j++){
			const double alpha = 0; //This might be able to change
			double beta = (level3Width - i)*((2.0*M_PI)/level3Width);
			double gamma = 
				((level3Height/2) - j)*(maxy/(level3Height/2));
			templates[0]->set(i, j, atan2(cos(alpha - beta)*sin(gamma),
			                   -1.0*sin(alpha - beta)*(1/cos(gamma))));
			weights[0][i][j] = sqrt((cos(alpha - beta)*sin(gamma))
			                           * (cos(alpha - beta)*sin(gamma)) +
			                   (-1.0*sin(alpha - beta)*(1/cos(gamma)))
			                   * (-1.0*sin(alpha - beta)*(1/cos(gamma))));
		}
	}
	//Set up the rest of the templates by rotating the existing one
	for (int k = 1 ; k < numTemps ; k++){
		templates[k] = new Img(level3Width, level3Height);
		ImgOps::rotate(templates[0], 
		(k*level3Width)/numTemps, templates[k]);
		for (int i = 0 ; i < level3Width ; i++){
			for (int j = 0 ; j < level3Height ; j++){
				int oldVal = i - level3Width/numTemps;
				if (oldVal < 0) oldVal = level3Width + oldVal;
				weights[k][i][j] = weights[k - 1][oldVal][j];
			}
		}
	}
}

FlowHomingAlg::~FlowHomingAlg() {
	/*if (debug){
		delete bestScoreImage;
		delete bestU;
		delete bestV;
	}*/
	delete [] templates;
	//delete SS;
	
    /*if (debug)
        delete imgWindow;*/
}

void FlowHomingAlg::snapshot(Img* inSS){
	SS = inSS;

	/*if (debug) {
		imgWindow->clear();
		imgWindow->addImg("SS", *SS);
		imgWindow->refresh();
	}*/
}

double FlowHomingAlg::getScore(Img* U, Img* V, int k, double best, Img* SI, Img* AI, bool isSecond){

	SingleConfig *config = SingleConfig::getInstance();
	bool useWeight = config->getBool("FlowHomingAlg.useWeightTemplate", false);

	int w = U->getWidth();
	int h = U->getHeight();

	int wt, ht;
	Img *currentTemplate;
	double tempWeight[level2Width][level2Height];
	if (!isSecond){
		//If this is the first set of comparisons,
		//We can use the premade templates
		wt = level3Width;
		ht = level3Height;
		currentTemplate = templates[k];
	}
	else{
		//Otherwise, we need to upscale the existing ones
		wt = level2Width;
		ht = level2Height;
		currentTemplate = new Img(level2Width, level2Height);
		for (int i = 0 ; i < level2Width ; i++){
			for (int j = 0 ; j < level2Height ; j++){
				const double alpha = 2*M_PI*k/numTemps;
				double beta = (level2Width - i)*((2.0*M_PI)/level2Width);
				double gamma = 
					((level2Height/2) - j)*(maxy/(level2Height/2));
				currentTemplate->set(i, j, atan2(cos(alpha - beta)*sin(gamma),
								-1.0*sin(alpha - beta)*(1/cos(gamma))));
				tempWeight[i][j] = sqrt((cos(alpha - beta)*sin(gamma))
			                           * (cos(alpha - beta)*sin(gamma)) +
			                   (-1.0*sin(alpha - beta)*(1/cos(gamma)))
			                   * (-1.0*sin(alpha - beta)*(1/cos(gamma))));
			}
		}
	}
				
	double score  = 0;
	
	int x, y;
	double weight;
	double iTheta, theta;
	
	int adjustedi;
	int adjustedj;
	
	//Compare each pixel's flow angle to the template angle,
	//Weighted according to the size of the ideal flow vector
	for (int i = 0 ; i < w ; i++){
		for (int j = 0 ; j < h ; j++){
			adjustedi = i + (wt - w)/2;
			adjustedj = j + (ht - h)/2;

			x = U->get(i, j);
			y = V->get(i, j);
			
			if (!useWeight){
				weight = sqrt((x*x)+(y*y));
			}
			else{
				if (!isSecond) weight = weights[k][adjustedi][adjustedj];
				else weight = tempWeight[adjustedi][adjustedj];
			}
			
			theta = atan2(y, x);
			

			score += Angles::getAngularDifference(currentTemplate->get(adjustedi,
			             adjustedj), theta) * weight; //abs(templates[k]->get(i + (wt - w), j + (ht - h)) - theta);
			SI->set(i, j, Angles::getAngularDifference(currentTemplate->get(adjustedi,
			             adjustedj), theta));
			AI->set(i, j, theta);
			//Stop if the current score is more than the best
			if (score >= best){
				i = w;
				j = h;
			}
		}
	}
	return score;
}

/* Returns the home angle in parameter &homeAngle */
/* The similarity may someday be stored in &similarity
 *  to help with halting */
void FlowHomingAlg::getHomeAngle
(Img* CV, double &homeAngle, double &similarity){
	SingleConfig *config = SingleConfig::getInstance();
	
	int besti = 0, bestk = 0;
	int step = config->getInt("FlowHomingAlg.firstLevelStep", 3);
	double temp, bestv = DBL_MAX;
	
	//We need the level 2 and 3 images, which are downsized by a factor
	//of 2 and 4, respectively
	Img *S = new Img(level3Width, level3Height);
	Img *C = new Img(level3Width, level3Height);
	Img *level2S = new Img(level2Width, level2Height);
	Img *level2C = new Img(level2Width, level2Height);
	cvPyrDown(SS->image, level2S->image);
	cvPyrDown(CV->image, level2C->image);
	cvPyrDown(level2S->image, S->image);
	cvPyrDown(level2C->image, C->image);
	
	//Score a series of rotations of the image and find the best fit
	for (int i = 0 ; i < level3Width ; i += step){
		Img *scoreImage = NULL;
		Img *angleImage = NULL;
		Img *T = NULL;
		Img *U = NULL;
		Img *V = NULL;
		ImgOps::rotate(C, i, T);
		ImgOps::blockMatch(*S, *T, U, V);

		scoreImage = new Img(U->getWidth(), U->getHeight());
		angleImage = new Img(U->getWidth(), U->getHeight());
		
		//in debug mode, initialize the score and flow images
		//so that they can be recorded later
		if (debug && !bsi){
			bestScoreImage = new Img(U->getWidth(), U->getHeight());
			bestAngleImage = new Img(U->getWidth(), U->getHeight());
			bsi = true;
		}

		//For each orientation of the snapshot, we try a number
		//of rotations of the template, each representing a different
		//home angle
		for (int k = 0 ; k < numTemps ; k++){
			temp = getScore(U, V, k, bestv, scoreImage, angleImage, false);
			if (temp < bestv){
				bestv = temp;
				besti = i;
				bestk = k;
				if (debug){
					*bestScoreImage = *scoreImage;
					*bestAngleImage = *angleImage;
					*bestU = *U;
					*bestV = *V;
				}
			}
		}
		delete scoreImage;
		delete angleImage;
		delete T;
		delete U;
		delete V;
	}

	//Now, look more closely for the best match in the larger image
	besti = besti*2; //To compensate for the doubling of snapshot size
	
	step = config->getInt("FlowHomingAlg.secondLevelStep", 3);
	bool useK = config->getBool("FlowHomingAlg.useK", true);
	bool useI = config->getBool("FlowHomingAlg.useI", true);
	int lower, upper;
	//for i
	if (useI){
		lower = besti - step;
		upper = besti + step;	
	}
	else{
		lower = besti;
		upper = besti;
	}
	
	//for k
	int lower2, upper2;
	if (useK){
		lower2 = bestk - step;
		if (lower2 < 0) lower = numTemps + lower;
		upper2 = bestk + step % numTemps;
	}
	else{
		lower2 = bestk;
		upper2 = bestk;
	}
	
	bestv = DBL_MAX;

//Now, try to obtain a more precise angle by testing nearby rotations
for (int i = lower ; i <= upper ; i += step){
	Img *scoreImage = NULL;
	Img *angleImage = NULL;
	Img *T = NULL;
	Img *U = NULL;
	Img *V = NULL;
	if (useI)
		ImgOps::rotate(C, i, T);
	else
		ImgOps::rotate(C, besti, T);
		
	ImgOps::blockMatch(*S, *T, U, V);

	scoreImage = new Img(U->getWidth(), U->getHeight());
	angleImage = new Img(U->getWidth(), U->getHeight());

	for (int k = lower2 ; k <= upper2 ; k++){
		temp = getScore(U, V, bestk, bestv, scoreImage, angleImage, true);
		if (temp < bestv){
			bestv = temp;
			besti = i;
			bestk = k;
			if (debug){
				*bestScoreImage = *scoreImage;
				*bestAngleImage = *angleImage;
				*bestU = *U;
				*bestV = *V;
			}
		}
	}
	delete scoreImage;
	delete angleImage;
	delete T;
	delete U;
	delete V;
}

	//Find the focus of contraction and use that to find alpha
	double foc = (level2Width*1.0)/numTemps * bestk - (besti);
	double alpha = (2*M_PI)*(1 - (foc/(level2Width*1.0)));
	homeAngle = alpha;

	/*
	//Experimental similarity idea:
	//Similarity is the inverse of the total best score image score
	//over the maximum total score ignoring weight.
	//Using the score image because it also ignores weight.
	double bestBaseValue = 0.0;
	for (int i = 0 ; i < bestScoreImage->getWidth() ; i++){
		for (int j = 0 ; j < bestScoreImage->getHeight() ; j++){
			bestBaseValue += bestScoreImage->get(i, j);
		}
	}

	//similarity = 1 - (bestBaseValue * 1.0)/
                       (M_PI * bestScoreImage->getWidth() * bestScoreImage->getHeight());

	//This does not work, because closeness to the source does not make for a better score.
	//Either the flow matches or it does not. The score does not take distance into account
	*/
	/*
	//Experimental similarity idea 2:
	//Similarity is the average magnitude of the angles
	//The angle images have been changed to weight for this experiment
	double totalMagnitude = 0.0;
	for (int i = 0 ; i < bestAngleImage->getWidth() ; i++){
		for (int j = 0 ; j < bestAngleImage->getHeight() ; j++){
			totalMagnitude += bestAngleImage->get(i, j);
		}
	}

	//similarity = (totalMagnitude/(bestAngleImage->getWidth() * bestAngleImage->getHeight()));
	*/

	similarity = 0;

	if (debug){
		int row = debugCount/10;
		int col = debugCount%10;
		
		//log the best rotation
		string outFileName = "./testing/log.txt";
		ofstream outFile(outFileName.c_str(), ios_base::app);
		//outFile << "At: " << row << "-" << col << endl;
		//outFile << "Rotation: " << besti << endl;
		outFile << "Number " << debugCount << endl;
		outFile << similarity << endl << endl;
		outFile.close();
		
		//Save the horizontal component of flow
		string fname1;
		ostringstream f1(fname1);
		f1 << "./testing/" << row << "_" << col << "u.grd";
		bestU->saveGrd(f1.str());
		//Save the vertical component of flow
		string fname2;
		ostringstream f2(fname2);
		f2 << "./testing/" << row << "_" << col << "v.grd";
		bestV->saveGrd(f2.str());
		//Save the score image
		string fname3;
		ostringstream f3(fname3);
		f3 << "./testing/" << row << "-" << col << "s.pgm";
		bestScoreImage->save(f3.str());
		
		//Save the angle image
		string fname4;
		ostringstream f4(fname4);
		f4 << "./testing/" << row << "-" << col << "a.pgm";
		bestAngleImage->save(f4.str());
		//Save the template
		string fname5;
		ostringstream f5(fname5);
		f5 << "./testing/" << row << "-" << col << "t.pgm";
		templates[bestk]->save(f5.str());
		
		//Save snapshot
		string fname6;
		ostringstream f6(fname6);
		f6 << "./testing/testS" << "-" << debugCount << ".pgm";
		SS->save(f6.str());
		//Save Current
		string fname7;
		ostringstream f7(fname7);
		f7 << "./testing/testC" << "-" << debugCount << ".pgm";
		CV->save(f7.str());
		
		debugCount++;
	}
	
	delete S;
	delete C;
	
/*	if (debug) {
		imgWindow->clear();
		imgWindow->addImg("SS", *SS);
		imgWindow->addImg("CV", *CV);
		imgWindow->addImg("Rotated CV", *rotCV);
		imgWindow->refresh();
	}*/
}
