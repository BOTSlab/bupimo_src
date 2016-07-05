/* 
 * ===========================================================================
 * 
 * Warping.H --
 * warping code for Andrew Vardy
 * 
 * Ralf Moeller <moeller@techfak.uni-bielefeld.de>
 * 
 *    Copyright (C) 2003
 *    Computer Engineering Group
 *    Faculty of Technology
 *    University of Bielefeld
 * 
 * 1.0 / 25. Aug 03 (rm)
 * - from scratch
 * 1.1 / 25. Sep 03 (rm)
 * - with correction (pol2Cart call) from Andrew
 * 1.2 / 2. Mar 04 (av)
 * - added 'getBestWarped'
 * - removed '#include <math.h>' to avoid confusing compiler
 * - also changed parameters, the most crucial of which seems to be a
 *   switch from using WARPING_DOTPRODUCT to WARPING_DIFFERENCE.
 * 
 * ===========================================================================
 */

#ifndef _WARPING_H_
#define _WARPING_H_

#include "FloatArrays.h"
#include "IntArrays.h"
#include "Math.h"
#include <stdio.h>
#include <float.h>

using namespace floatarrays;
using namespace intarrays;

// for loop starting from 0
#define FOR(VAR,END) for (VAR = 0; VAR < END; VAR++)

class
Warping 
{

public:
  static const int WARPING_DOTPRODUCT = 0; // use dot product in matching
  static const int WARPING_DIFFERENCE = 1; // use vector difference -,,-

  // warping parameter
  float warpingRhoMax;		// maximal value for rho
  int warpingRhoSteps;	   // rho varies in X steps (0..warpingRhoMax)
  int warpingAlphaSteps;	// alpha varies in X steps (0..2pi)
  int warpingPsiSteps;		// psi varies in X steps (0..2pi)
  int warpingMatchMode;	    // matching mode (dot prod. or difference)
  
  // constructor
  Warping()
  {
    // parameter initialization (defaults, can be overwritten after
    // construction)
	/*
    warpingRhoMax = 1.0;
    warpingRhoSteps = 20;
    warpingAlphaSteps = 20;
    warpingPsiSteps = 20;
    warpingMatchMode = WARPING_DOTPRODUCT;
	*/
    warpingRhoMax = 1.0;
    warpingRhoSteps = 20;
    warpingAlphaSteps = 36;
    warpingPsiSteps = 1;
    warpingMatchMode = WARPING_DIFFERENCE;
    // initialize
    init();
  }

  // destructor
  ~Warping()
  {
    // BlockXXX things destruct themselves
  }

  //--------------------------------------------------------------------------
  // setSnapshotWarping
  //--------------------------------------------------------------------------

  // ss should contain image, pixels in range [0,1]
  void
  setSnapshot(const FloatBlockVector &ss)
  {
    int iAlpha, iRho, iTheta, iPsi, iThetaNew;
    float alpha, rho, theta, psi, thetaNew;

    // store size of snapshot 25. Aug 03 (rm)
    size = ss.size();
    // compute angle difference between pixels 25. Aug 03 (rm)
    dAngle = TWOPI / size;
    // resize store of warped images
    warpTable.reset(warpingRhoSteps, warpingAlphaSteps, warpingPsiSteps,
		    size, 0);
    // compute step sizes (end points should be TWOPI and 1.0)
    warpingDAlpha = TWOPI / warpingAlphaSteps;
    warpingDPsi = TWOPI / warpingPsiSteps;
    // this one is not closed in a circle, therefore -1
    warpingDRho = warpingRhoMax / (warpingRhoSteps - 1);
    // create warped images
    rho = 0.0;
    FOR(iRho,warpingRhoSteps) {
      alpha = 0.0;
      FOR(iAlpha,warpingAlphaSteps) {
	psi = 0.0;
	FOR(iPsi,warpingPsiSteps) {
	  theta = 0.0;
	  FOR(iTheta,size) {
	    thetaNew = warpAngle(alpha, rho, psi, theta);
	    // index() contains a rounding operation
	    iThetaNew = index(thetaNew, size, dAngle);
	    // this is the table: for each pixel in the warped cv it specifies,
	    // which pixel in the original cv should be assigned
	    warpTable[iRho][iAlpha][iPsi][iTheta] = iThetaNew;
	    theta += dAngle;
	  }
	  psi += warpingDPsi;
	}
	alpha += warpingDAlpha;
      }
      rho += warpingDRho;
    }
    // memorize snapshot (to make access easier later)
    warpingSS = ss;
  }
  
  //--------------------------------------------------------------------------
  // setCurrentViewWarping
  //--------------------------------------------------------------------------
  
  // cv should contain current view, pixels in range [0,1]
  void
  setCurrentView(const FloatBlockVector &cv)
  {
    // note: the parameters could have changed, which could be desastrous...
    // 25. Aug 03 (rm) (I don't understand this comment, I just hope it's not)
    int iAlpha, iAlphaMin, iRho, iRhoMin, iPsi, iPsiMin;
    float m;			// match measure

    if (cv.size() != (unsigned int) size) {
      fprintf(stderr, "invalid cv size\n");
      exit(-1);
    }
    // resize different internal cv versions
    warpingCVOrig.resize(size);	// original cv
    warpingCVWarped.resize(size); // warped cv (just helper var.)
    warpingCVWarpedBest.resize(size); // best warped cv
    // assign current view for faster access
    warpingCVOrig = cv;
    // minimum search: if they are still -1 afterwards, something went wrong
    iAlphaMin = iRhoMin = iPsiMin = -1;
    warpingMatchMin = FLT_MAX;
    // warp the view in all possible ways and shift it
    FOR(iRho,warpingRhoSteps) {
      FOR(iAlpha,warpingAlphaSteps) {
	FOR(iPsi,warpingPsiSteps) {
	  warpView(warpingCVOrig, warpTable[iRho][iAlpha][iPsi], 
		   warpingCVWarped);
	  // determine match quality m
	  m = match(warpingSS, warpingCVWarped);
	  if (m < warpingMatchMin) {
	    iAlphaMin = iAlpha; iRhoMin = iRho; iPsiMin = iPsi;
	    warpingMatchMin = m;
	  }
	}
      }
    }
    // assign optimum parameters
    warpingAlphaMin = iAlphaMin * warpingDAlpha;
    warpingRhoMin = iRhoMin * warpingDRho;
    warpingPsiMin = iPsiMin * warpingDPsi;
    // assign optimal warped view
    warpView(warpingCVOrig, warpTable[iRhoMin][iAlphaMin][iPsiMin], 
	     warpingCVWarpedBest);
    // compute home direction (p.192, but p.195 says "beta = alpha + pi" -> ?)
    warpingBetaMin = circularAngle(warpingAlphaMin - warpingPsiMin + PI);
  }

  //--------------------------------------------------------------------------
  // matchWarping
  //--------------------------------------------------------------------------

  // returns home vector scaled with warpingRhoMin (25. Aug 03 (rm):?)
  FloatBlockVector
  matchWarping()
  {
    // 26. Nov 99 (rm): scaled with warpingRhoMin
    // return pol2Cart(warpingBetaMin, warpingRhoMin);
    return pol2Cart(warpingRhoMin, warpingBetaMin);
  }
  
  //--------------------------------------------------------------------------
  // getWarpTableEntry
  //--------------------------------------------------------------------------
  
  bool
  getWarpTableEntry(float rho, float alpha, float psi, IntBlockVector &w)
  {
    int iRho, iAlpha, iPsi;
    
    iRho = (int) rint(rho / warpingDRho);
    iAlpha = (int) rint(alpha / warpingDAlpha);
    iPsi = (int) rint(psi / warpingDPsi);
    if ((iRho < 0) || (iRho >= warpingRhoSteps) ||
	(iAlpha < 0) || (iAlpha >= warpingAlphaSteps) ||
	(iPsi < 0) || (iPsi >= warpingPsiSteps))
      return false;
    w = warpTable[iRho][iAlpha][iPsi];
    return true;
  }

  //--------------------------------------------------------------------------
  // getBestWarped
  //--------------------------------------------------------------------------

  FloatBlockVector
  *getBestWarped()
  {
	  return &warpingCVWarpedBest;
  }

protected:		    
  
  // warping variables
  float warpingBetaMin;		// optimal beta value (homeward angle)
  float warpingAlphaMin;	// optimal alpha value
  float warpingRhoMin;		// optimal rho value
  float warpingPsiMin;		// optimal psi value
  float warpingMatchMin;	// optimal match value
  float warpingDAlpha;		// step size for alpha
  float warpingDRho;		// step size for rho
  float warpingDPsi;		// step size for psi  
  IntBlockHyper4 warpTable; // warped views [rho][alpha][psi][pixel]
  FloatBlockVector warpingSS;	// snapshot
  FloatBlockVector warpingCVOrig; // original current view
  FloatBlockVector warpingCVWarped; // warped current view (jworking copy!)
  FloatBlockVector warpingCVWarpedBest; // optimal warped current view 
  int size;		        // size of binary snapshot
  float dAngle;			// angle of one pixel
 
  //--------------------------------------------------------------------------
  // initWarping
  //--------------------------------------------------------------------------

  void
  init()
  {
    // just to have defined values
    warpingDAlpha = warpingDRho = warpingDPsi = 0.0;
    warpingBetaMin = warpingAlphaMin = warpingRhoMin = warpingPsiMin = 0.0;
    warpingMatchMin = 0.0;    
  }
  
 //--------------------------------------------------------------------------
  // other methods
  //--------------------------------------------------------------------------
  
  // warp function without shift term psi (? 23. Feb 04 (rm))
  float
  warpAngle(float alpha, double rho, float psi, float theta)
  {
    float a = theta - alpha;
    return theta + atan2(rho * sin(a), 1.0 - rho * cos(a)) - psi;
  }
  
  // apply warp table entry to view -> warpedView
  void
  warpView(const FloatBlockVector &view, 
	   const IntBlockVector &warpTableEntry, 
	   FloatBlockVector &warpedView)
  {
    int i;
    FOR(i,size)
      warpedView[i] = view[warpTableEntry[i]];
  }
  
  // determine match between original snapshot and shifted warped current view
  float
  match(const FloatBlockVector &ssOrig, const FloatBlockVector cvWarped)
  {
    int i;
    float m = 0.0;
    
    switch (warpingMatchMode) {
    case WARPING_DOTPRODUCT:
      FOR(i,size)
	m -= ssOrig[i] * cvWarped[i];
      break;
    case WARPING_DIFFERENCE:
      FOR(i,size)
	m += sqr(ssOrig[i] - cvWarped[i]);
      break;
    default:
      fprintf(stderr, "invalid warping match mode %d\n", warpingMatchMode);
      exit(-1);
    }
    return m;
  }
  
  // 25. Aug 03 (rm)
  FloatBlockVector
  pol2Cart(float len, float angle) {
    // create a vector which has the length len and is pointing into dir angle
    // the vector is 2d
    FloatBlockVector v(2, 0.0);
    v[0] = cos(angle) * len;
    v[1] = sin(angle) * len;
    return v;
  }
  
};


#endif
