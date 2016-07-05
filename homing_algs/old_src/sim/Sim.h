/*
 * Simulation environment for a point robot operating in the plane.  This robot
 * is subject to realisitic error in its odometry.
 *
 * Andrew Vardy
 */
#ifndef SIM_H
#define SIM_H

#include <cmath>
#include "Img.h"
#include "Imgdb.h"
//#include "SimWindow.h"
#include "ImgWindow.h"
#include "Vec2.h"
#include "rng.h"
#include "Angles.h"

class Sim {
public:
    Sim(string title, Imgdb *inImgdb);
    virtual ~Sim();
    int getWidth();
    int getHeight();
    int getImageWidth();
    int getImageHeight();
    double getOdoX();
    double getOdoY();
    double getOdoTheta();
    void teleport( double newX, double newY, double newTheta, bool ss );
    void move( double translation, double rotation );
    void move( Vec2 &vec );
    Img* getSS();
    Img* getCV();
 //   void addVector( double magnitude, double angle );

// The use of any of these methods constitutes "cheating" in that a true robot
// would never have access to its actual position.  They should only be used
// for analytical purposes.
    double cheatGetTheta() {
        return theta;
    }
    double cheatGetAngleToGoal( double sx, double sy );
    double cheatGetDistanceToGoal( double sx, double sy );

protected:
//    void plotOdo();
    void getImg( Img *&rotatedImg );
    int inBoundsIndex( int i, int cap );

    Imgdb *imgdb;
//    SimWindow *window;
    bool showImages;
    double odoX, odoY, odoTheta;
    double lastOdoX, lastOdoY, lastOdoTheta;
    double lastTeleportX, lastTeleportY, lastTeleportTheta;
    double moveParam[4];
    double measParam[4];
    RNG random;

    Img *rotatedSS;
    Img *rotatedCV;

    ImgWindow *ssWindow, *cvWindow;
private:
    double x, y, theta;
    double lastX, lastY, lastTheta;
};

#endif
