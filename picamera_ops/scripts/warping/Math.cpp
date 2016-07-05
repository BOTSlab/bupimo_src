/* 
 * ===========================================================================
 * 
 * Math.C --
 * 
 * 
 * Ralf Moeller <moeller@psy.mpg.de>
 * 
 *    Copyright (C) 2002
 *    Cognitive Robotics Group
 *    Max Planck Institute for Psychological Research
 *    Munich
 * 
 * 1.0 / 30. Sep 02 (rm)
 * - from unimath.C 1.21
 * 1.1 /  1. Oct 02 (rm)
 * - sqr from MathFuncs.H
 * 1.2 /  3. Oct 02 (rm)
 * - removed templates
 *
 * ===========================================================================
 */

/*==========================================================================*
 * INCLUDES 
 *==========================================================================*/

#include "Math.h"

/*==========================================================================*
 *  FUNCTIONS
 *==========================================================================*/

//----------------------------------------------------------------------------
// uceil
//----------------------------------------------------------------------------
double
uceil(double x)
{
  double y = ceil(x);
  // uceil(x) never equals x
  return (y == x) ? y + 1.0 : y;
}

//----------------------------------------------------------------------------
// kathetos
//----------------------------------------------------------------------------
double
kathetos(double hyp, double kat)
{
  return sqrt(hyp * hyp - kat * kat);
}

//----------------------------------------------------------------------------
// signum
//----------------------------------------------------------------------------
double
signum(double x){
  // from USDemo.Mod
  if (x >= 0) return 1;
  return -1;
}

//----------------------------------------------------------------------------
// min
//----------------------------------------------------------------------------
double
min(double x1, double x2){
  return x1 < x2 ? x1 : x2;
}

//----------------------------------------------------------------------------
// max
//----------------------------------------------------------------------------
double
max(double x1, double x2){
  return x1 > x2 ? x1 : x2;
}

//----------------------------------------------------------------------------
// imin
//----------------------------------------------------------------------------
int
imin(int x1, int x2){
  return x1 < x2 ? x1 : x2;
}

//----------------------------------------------------------------------------
// imax
//----------------------------------------------------------------------------
int
imax(int x1, int x2){
  return x1 > x2 ? x1 : x2;
}

//----------------------------------------------------------------------------
// modulo
//----------------------------------------------------------------------------
// modulo(-5,3) = 1
// modulo(-4,3) = 2
// modulo(-3,3) = 0
// modulo(-2,3) = 1
// modulo(-1,3) = 2
// modulo(0,3) = 0
// modulo(1,3) = 1
// modulo(2,3) = 2
// modulo(3,3) = 0
// modulo(4,3) = 1
// modulo(5,3) = 2     
// 
int
modulo(int x, int y)
{
  x = x % y;
  return (x < 0) ? x + y : x;
}

//----------------------------------------------------------------------------
// signum0
//----------------------------------------------------------------------------
double
signum0(double x){
  // as signum, but with 0
  if (x > 0) return 1;
  if (x == 0) return 0;
  return -1;
}

//----------------------------------------------------------------------------
// vecTransform
//----------------------------------------------------------------------------
void
vecTransform(float xref, float yref, float x, float y, 
	     float &xt, float &yt) {
  xt = xref * x - yref * y;
  yt = yref * x + xref * y;
}

void
vecTransform(double xref, double yref, double x, double y, 
	     double &xt, double &yt) {
  xt = xref * x - yref * y;
  yt = yref * x + xref * y;
}

//----------------------------------------------------------------------------
// transform
//----------------------------------------------------------------------------
void
transform(float x, float y, float& xt, float& yt, float beta) {
  // Koordinatensystemtransformation, (x,y) -> (xt, yt) bei Drehung um beta
  // (Drehung CW!)
  // float s, c;
  // s = sin(beta);
  // c = cos(beta);
  // xt = c * x - s * y;
  // yt = s * x + c * y;
  vecTransform(cos(beta), sin(beta), x, y, xt, yt);
}

void
transform(double x, double y, double& xt, double& yt, double beta) {
  // Koordinatensystemtransformation, (x,y) -> (xt, yt) bei Drehung um beta
  // (Drehung CW!)
  // double s, c;
  // s = sin(beta);
  // c = cos(beta);
  // xt = c * x - s * y;
  // yt = s * x + c * y;
  vecTransform(cos(beta), sin(beta), x, y, xt, yt);
}

//----------------------------------------------------------------------------
// circularAngle
//----------------------------------------------------------------------------
double
circularAngle(double angle) {
  // all angles are corrected onto 0..2pi
  if (angle < 0) {
    angle += ((int)(fabs(angle) / TWOPI) + 1) * TWOPI; // -2pi -> ??
    if (angle == TWOPI) angle = 0.0;
  }
  else if (angle >= TWOPI)
    angle -= ((int)(angle / TWOPI)) * TWOPI;
  return angle;
}

//----------------------------------------------------------------------------
// circularIndex (tested, is ok)
//----------------------------------------------------------------------------
int 
circularIndex(int i, int size) {
  // index is corrected to 0..size-1
  if (i < 0)
    i += (abs(i+1) / size + 1) * size;
  else if (i >= size)
    i = i % size;
  return i;
}

//----------------------------------------------------------------------------
// angle
//----------------------------------------------------------------------------
double
angle(int index, int size, double dAngle) {
  // compute a angle from an index
  return circularIndex(index, size) * dAngle;
}

//----------------------------------------------------------------------------
// index
//----------------------------------------------------------------------------
int 
index(double angle, int size, double dAngle) {
  // compute an index from an angle
  int i = (int)rint(circularAngle(angle) / dAngle);
  // that's was the erroneous old one:
  //   return (i == size) ? (size-1) : i;
  return (i == size) ? 0 : i;
}

//----------------------------------------------------------------------------
// angleDiffSgn
//----------------------------------------------------------------------------
int
angleDiffSgn(double angle1, double angle2) {
  // for the circular angle, compute, in which angular direction we should
  // move, if we are at angle2 and want to go into direction of angle1
  // example1:
  //  angle1 = 10deg, angle2 = 350deg
  //  angle1 - angle2 = -340deg
  //  circularAngle(angle1 - angle2) = 20deg
  //  returns: 1
  // example2:
  //  angle1 = 350deg, angle2 = 10deg
  //  angle1 - angle2 = 340deg
  //  circularAngle(angle1 - angle2) = 340deg
  //  returns: -1
  // example3:
  //  angle1 = 170deg, angle2 = 190deg
  //  angle1 - angle2 = -20deg
  //  circularAngle(angle1 - angle2) = 340deg
  //  returns: -1
  return circularAngle(angle1 - angle2) > PI ? -1 : 1;
}

//----------------------------------------------------------------------------
// adjustPi
//----------------------------------------------------------------------------
double
adjustPi(double angle) {
  // transforms an angle in the interval [0..360] to the interval [-180, 180]
  // 0 -> 0, 90 -> 90, 270 -> -90
  return angle > PI ? angle - TWOPI : angle;
}

//----------------------------------------------------------------------------
// circularAnglePi
//----------------------------------------------------------------------------
double
circularAnglePi(double angle) {
  // transforms arbitrary angle to -180..180
  return adjustPi(circularAngle(angle));
}

//----------------------------------------------------------------------------
// adjustSize2
//----------------------------------------------------------------------------
int
adjustSize2(int index, int size) {
  // adjust the index to -size/2 ... size2
  return index > size/2 ? index - size : index;
}

//----------------------------------------------------------------------------
// angleDiffPi
//----------------------------------------------------------------------------
double
angleDiffPi(double angle1, double angle2) {
  // compute the difference of two angles in the range [-180,180]
  return adjustPi(circularAngle(angle1 - angle2));
}

//----------------------------------------------------------------------------
// indexDiffSize2
//----------------------------------------------------------------------------
int
indexDiffSize2(int index1, int index2, int size) {
  // compute the difference of two circ. indices in [-size/2,size/2]
  return adjustSize2(circularIndex(index1 - index2, size), size);
}

//----------------------------------------------------------------------------
// angleMiddle
//----------------------------------------------------------------------------
double
angleMiddle(double angle1, double angle2) {
  return circularAngle(angle2 + angleDiffPi(angle1, angle2)/2.0);
}


//----------------------------------------------------------------------------
// indexMiddle
//----------------------------------------------------------------------------
int
indexMiddle(int index1, int index2, int size) {
  // examples: (size = 8)
  // index1  index2  indexDiffSize2  indexDiffSize2/2  result
  // 1       7       -6 -> 2 -> 2     1                 8 -> 0
  // 7       1        6 -> 6 -> -2    -1                0
  // 1       3        -2 -> 6 -> -2   -1                2
  // 3       1        2 -> 2 -> 2     1                 2

  return circularIndex(index2 + indexDiffSize2(index1, index2, size)/2, size);
}

//----------------------------------------------------------------------------
// randIndex
//----------------------------------------------------------------------------
int
randIndex(int size)
{
  // generates random numbers from 0..size-1 (see man srand and
  // "Numerical Recipes in C" ch. 7)
  return (int) ((double) size * rand() / (RAND_MAX + 1.0));
}

//----------------------------------------------------------------------------
// fermi
//----------------------------------------------------------------------------
double
fermi(double dE, double T) {
  // accept improvements of E (dE < 0) with higher probability
  if (T == 0.0)
    return (dE <= 0.0) ? 1.0 : 0.0;
  return 1.0 / (1.0 + exp(dE/T));
}

//----------------------------------------------------------------------------
// drand48_open
//----------------------------------------------------------------------------
double 
drand48_open()
{
  // generates random number in the open interval (0,1)
  double r;
  do
    r = drand48();
  while (r < 1E-30);
  return r;
}

//----------------------------------------------------------------------------
// gaussian
//----------------------------------------------------------------------------
double 
gaussian()
{
  // generates a gaussian distribution with E()=0 and D()=1
  double y1, y2;
  y1 = drand48_open();
  y2 = drand48_open();
  return sqrt(-2*log(y1)) * sin(2*M_PI*y2);
}

//----------------------------------------------------------------------------
// time_srand48
//----------------------------------------------------------------------------
void 
time_srand48()
{	
  long t;
  t = time(0);
  srand48(t);
}

//----------------------------------------------------------------------------
// sigmoid (via tanh)
//----------------------------------------------------------------------------

double
sigmoid(double x, double spread, double theta)
{
  return 0.5 * (tanh(0.5 * spread * (x - theta)) + 1.0);
}
