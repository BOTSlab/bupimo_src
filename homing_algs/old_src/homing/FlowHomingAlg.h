#ifndef FLOWHOMINGALG_H
#define FLOWHOMINGALG_H

#include "TotalHomingAlg.h"
/* A visual homing algorithm using optic flow
 * written by Robert Hamilton
 */

#include "ImgWindow.h"
#include "ImgOps.h"
#include "Angles.h"

class FlowHomingAlg : public TotalHomingAlg {

public:
	FlowHomingAlg(int inWidth, int inHeight, double inMaxY);
	~FlowHomingAlg();
	virtual void snapshot(Img* inSS);
	virtual void getHomeAngle(Img* CV, double &homeAngle, double &similarity);
	
private:
	double getScore(Img* U, Img* V, int k, double best, Img* SI, Img *AI, bool isSecond);
	
	static int debugCount;
	Img **templates;
	double ***weights;
	Img *SS;
	Img *bestScoreImage;
	Img *bestAngleImage;
	Img *bestU;
	Img *bestV;
	bool bsi;
	const int width, height, level3Width, level3Height, level2Width, level2Height;
	double maxy;
	//static bool debug;
	Img *rotCV;
	ImgWindow *imgWindow;
};
#endif
