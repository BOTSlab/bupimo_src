typedef struct MatchPairSt {
	struct KKeypointSt *CV;
	struct KKeypointSt *SS;
	struct MatchPairSt *next;
} * MatchPair;

int GetScore(KKeypoint k1, KKeypoint k2);
MatchPair MakePair(KKeypoint CV, KKeypoint SS, MatchPair m);
MatchPair GetMatches(KKeypoint CV, KKeypoint SS);
MatchPair MatchPair_Create();
double ** Double_Create(int rows, int cols);
float ** Float_Create(int rows, int cols);
int ** Int_Create(int rows, int cols);
void Array_Free(void ** arr, int w);
void realAngles(double ** field, int cX, int cY);
double correctAngle(double a);
double angleDiff(double a1, double a2);
void DoRotations(int ** ActualRotations);
void shift(Image im, int s, int t);
void doHoming(int cX, int cY);
void printArray(double ** arr);
void printArrayNon(double ** arr);
void printArrayInt(int ** arr);
void DatabaseHoming();
double * toCircle(double x);
double toCircleX(double x);
double toCircleY(double x);
double * GetMeans(MatchPair matches, int width);
void ConvertToAngles(double ** HV);
double ConvertToAngle(double pixel);
int CountKeys(KKeypoint keys);
int CountMatches(MatchPair matches);
double GetHomeAngle(char * CV, char * SS);
void makeGNUPLOTfile(double ** winners);
void print_progress(int xx, int yy);
void InsertObject(Image im);
KKeypoint GetImageKeypoints(char * image, double rotation);
int HomeToGoal(int xx, int yy);
void CalculateReturnRatio();
void ResetPathMask();
void PrintPathMask();
void printArrayDouble(double ** arr);
