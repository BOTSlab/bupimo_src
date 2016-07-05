/* 
 * ===========================================================================
 * 
 * Math.H --
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
 * - from unimath.H 1.26
 * - removed all vector stuff, lists etc.
 * 1.1 /  3. Oct 02 (rm)
 * - templates here
 *
 * ===========================================================================
 */

// TODO:
// - some functions should be turned into templates later

#ifndef _MATH_H_
#define _MATH_H_

/*==========================================================================*
 * INCLUDES 
 *==========================================================================*/

#include <math.h>
#include <stdlib.h>
#include <time.h>

/*==========================================================================*
 *  TEMPLATES
 *==========================================================================*/

//--------------------------------------------------------------------------
// square
//--------------------------------------------------------------------------

template <class T> 
T sqr(T x) { return x * x; }

/*==========================================================================*
 *  DEFINES, MACROS
 *==========================================================================*/

// 15. Aug 99 (rm): PI and PI2 are no longer defined in math.h
// this was copied from the old math.h

#ifndef PI                      /* as in stroustrup */
#define PI  M_PI
#endif
#ifndef PI2
#define PI2  (M_PI + M_PI)
#endif

#define TWOPI PI2	        // 2 * pi
#define PIHALF M_PI_2		// pi / 2
#define PIQUARTER M_PI_4	// pi / 4
#define DEG_TO_RAD (PI / 180.0)	// pi/180
#define RAD_TO_DEG (180.0 / PI)	// 180/pi
#define DEG2RAD DEG_TO_RAD
#define RAD2DEG RAD_TO_DEG

/*==========================================================================*
 * FUNCTIONS 
 *==========================================================================*/

// floor and ceil of an integer are the same
//   floor(1.9) = 1.0    ceil(1.9) = 2.0
//   floor(2.0) = 2.0    ceil(2.0) = 2.0
//   floor(2.1) = 2.0    ceil(2.1) = 3.0
// in this version (uceil) it's like this
//   ufloor(1.9) = 1.0   uceil(1.9) = 2.0
//   ufloor(2.0) = 2.0   uceil(2.0) = 3.0
//   ufloor(2.1) = 2.0   uceil(2.1) = 3.0
extern double 
uceil(double x);

// kathetos (there's no English word for "Kathete" is seems, so we take the
// Greek one)
extern double
kathetos(double hyp, double kat);

// calculates the signum of x
extern double
signum(double x),
signum0(double x);

// calculates the minimum/maximum
extern double
min(double x1, double x2),
max(double x1, double x2);

// same for ints
extern int
imin(int x1, int x2),
imax(int x1, int x2);

// as x % y, but with different behavior for negative x
extern int
modulo(int x, int y);

// transformation of (x,y) -> (xt, yt) when turning in direction of unit
// vector (xref, yref)
extern void
vecTransform(float xref, float yref, float x, float y, 
	     float &xt, float &yt),
vecTransform(double xref, double yref, double x, double y, 
	     double &xt, double &yt);

// transformation of (x,y) -> (xt,yt) when turned about angle beta
extern void
transform(float x, float y, float& xt, float& yt, float beta),
transform(double x, double y, double& xt, double& yt, double beta);

// corrects the angle onto 0..2pi or -pi..pi, resp.
extern double
circularAngle(double angle),
circularAnglePi(double angle);

// index is corrected to 0..size-1
extern int 
circularIndex(int i, int size);

// compute a angle from an index
extern double
angle(int index, int size, double dAngle);

// compute an index from an angle
extern int 
index(double angle, int size, double dAngle);

// for the circular angle, compute, in which angular direction we should
// move, if we are at angle2 and want to go into direction of angle1
extern int
angleDiffSgn(double angle1, double angle2);

// transforms an angle in the interval [0..360] to the interval [-180, 180]
extern double
adjustPi(double angle);

// transforms an index in the interval [0..size] to the interval [-size/2,size]
extern int
adjustSize2(int index, int size);

// compute the difference of two angles in the range [-180,180]
extern double
angleDiffPi(double angle1, double angle2);

// compute the index difference in the range [-size/2,size/2]
extern int
indexDiffSize2(int index1, int index2, int size);

// compute the middle of two circular indices
extern int
indexMiddle(int index1, int index2, int size);

// compute the middle of two circular angles
extern double
angleMiddle(double angle1, double angle2);

// random integer index between 0 and size-1 (including both)
extern int
randIndex(int size);

// fermi function
extern double
fermi(double dE, double T);

// random numbers in (0,1)
extern double 
drand48_open();

// gaussian distribution with E()=0 and D()=1
extern double 
gaussian();

// init with time()
extern void 
time_srand48();

// sigmoid function (implemented with tanh to avoid overflow)
extern double
sigmoid(double x, double spread, double theta);

#endif







