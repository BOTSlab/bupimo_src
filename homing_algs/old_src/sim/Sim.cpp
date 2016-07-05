#include "Sim.h"
#include "ImgOps.h"
#include "Angles.h"
#include <iomanip>

Sim::Sim(string title, Imgdb* inImgdb)
   : imgdb(inImgdb), showImages(false), random(0)
{
    //window = new SimWindow(title, -1, -1,imgdb->getWidth(),imgdb->getHeight(),
    //                       false);

    x = 0;
    y = 0;
    theta = 0;
    lastX = x;
    lastY = y;
    lastTheta = theta;
    odoX = 0;
    odoY = 0;
    odoTheta = 0;

    moveParam[0] = 0.0;
    moveParam[1] = 0.0;
    moveParam[2] = 0.0;
    moveParam[3] = 0.0;

    measParam[0] = 0.1;
    measParam[1] = 0.001;
    measParam[2] = 0.01;
    measParam[3] = 0.1;

    rotatedSS = NULL;
    rotatedCV = NULL;

    if (showImages) {
        ssWindow = new ImgWindow("SS");
        cvWindow = new ImgWindow("CV");
    }
}

Sim::~Sim() {
    if ( rotatedSS != NULL ) delete rotatedSS;
    if ( rotatedCV != NULL ) delete rotatedCV;

    if (showImages) {
        delete ssWindow;
        delete cvWindow;
    }

    //delete window;

    //cout << "Sim: Destructor completed." << endl;
}

int Sim::getWidth() { return imgdb->getWidth(); }
int Sim::getHeight() { return imgdb->getHeight(); }
int Sim::getImageWidth() { return imgdb->getImageWidth(); }
int Sim::getImageHeight() { return imgdb->getImageHeight(); }

double Sim::getOdoX() { return odoX; }
double Sim::getOdoY() { return odoY; }
double Sim::getOdoTheta() { return odoTheta; }

void Sim::teleport( double newX, double newY, double newTheta, bool ss ) {
    x = newX;
    y = newY;
    theta = newTheta;
    lastTeleportX = x;
    lastTeleportY = y;
    lastTeleportTheta = theta;
    odoX = 0;
    odoY = 0;
    odoTheta = 0;

    /*
    if ( ss )
        window->addPoint(x, y, 115);
    else
        window->addPoint(x, y, 1);
    */
}

void Sim::move( double translation, double rotation ) {
// Displace the agent by rotating it by 'rotation' and then translating
// forward (along the positive x-axis) by 'translation'.
    lastX = x;
    lastY = y;
    lastTheta = theta;

    lastOdoX = odoX;
    lastOdoY = odoY;
    lastOdoTheta = odoTheta;

    // We use the method of Thrun et. al (2005)
    double deltaRot1 = rotation;
    double deltaTrans = translation;

    // The variance and standard deviations for these three quantities
    double rot1Sqd = deltaRot1 * deltaRot1;
    double transSqd = deltaTrans * deltaTrans;
    double std1 = sqrt(moveParam[0]*rot1Sqd + moveParam[1]*transSqd);
    double std2 = sqrt(moveParam[2]*transSqd + moveParam[3]*rot1Sqd);
    double std3 = sqrt(moveParam[1]*transSqd);

    // Sample from the three distributions (all Gaussian) and perturb the
    // three motion quantities
    double moveDeltaRot1 = deltaRot1 + random.normal(0, std1);
    double moveDeltaTrans = deltaTrans + random.normal(0, std2);
    double moveDeltaRot2 = random.normal(0, std3);

    // Update the robot's position
    x += moveDeltaTrans * cos(theta + moveDeltaRot1);
    y += moveDeltaTrans * sin(theta + moveDeltaRot1);
    theta += moveDeltaRot1 + moveDeltaRot2;
    theta = Angles::constrainAngle(theta);

    // Now we update the internal estimate of position (odoX,odoY,odoTheta).
    // For this, we need to repeat the same process as above, only using the
    // parameters for measurement error.

    rot1Sqd = moveDeltaRot1 * moveDeltaRot1;
    transSqd = moveDeltaTrans * moveDeltaTrans;
    double rot2Sqd = moveDeltaRot2 * moveDeltaRot2;
    std1 = sqrt(measParam[0]*rot1Sqd + measParam[1]*transSqd);
    std2 = sqrt(measParam[2]*transSqd + measParam[3]*(rot1Sqd + rot2Sqd));
    std3 = sqrt(measParam[0]*rot2Sqd + measParam[1]*transSqd);

    double measDeltaRot1 = moveDeltaRot1 + random.normal(0, std1);
    double measDeltaTrans = moveDeltaTrans + random.normal(0, std2);
    double measDeltaRot2 = moveDeltaRot2 + random.normal(0, std3);

    odoX += measDeltaTrans * cos(odoTheta + measDeltaRot1);
    odoY += measDeltaTrans * sin(odoTheta + measDeltaRot1);
    odoTheta += measDeltaRot1 + measDeltaRot2;
    odoTheta = Angles::constrainAngle(odoTheta);

    /*
    cout << setiosflags(ios::fixed) << setprecision(2) << "true:" << endl
         << "    x: " << x << ",    y: " << y << ",    theta: " << theta
         << endl << "odometry:" << endl
         << "    x: " << odoX << ", y: " << odoY << ", theta: " << odoTheta
         << endl;
    */

    //plotOdo();
    //window->addLine(lastX, lastY, x, y);
}

void Sim::move( Vec2 &vec ) {
    move(vec.mag, vec.ang);
}

/*
void Sim::plotOdo() {
    double phi = atan2(odoY, odoX);
    double r = sqrt(odoX*odoX + odoY*odoY);
    double px = lastTeleportX + r * cos(phi + lastTeleportTheta);
    double py = lastTeleportY + r * sin(phi + lastTeleportTheta);
    window->addPoint(px, py, 18);
}
*/

Img* Sim::getSS() {
    getImg(rotatedSS);
    if (showImages)
        ssWindow->setImg(*rotatedSS);
    return rotatedSS;
}

Img* Sim::getCV() {
    getImg(rotatedCV);
    if ( showImages )
        cvWindow->setImg(*rotatedCV);
    return rotatedCV;
}

void Sim::getImg( Img *&rotatedImg ) {
    int w = imgdb->getWidth();
    int h = imgdb->getHeight();
    int xb = inBoundsIndex( (int)(x + 0.5), w );
    int yb = inBoundsIndex( (int)(y + 0.5), h );
    Img* img = imgdb->getImg(xb, yb);

    // We could be inside a wall.  The current check for this is
    // to look at the image and make sure it is not mostly black (why
    // "mostly"?  because POV-Ray sometimes generates spurious non-black
    // pixels...  at least for my current models).
    /*
    if ( img->getSum() / (img->getWidth()*img->getHeight()) < 0.01 )
        return NULL;
    */

    // Rotate the image to match the robot's orientation.
    int colrot = Angles::angle2int(theta, img->getWidth());
    ImgOps::rotate(img, colrot, rotatedImg);
}

int Sim::inBoundsIndex( int i, int cap ) {
    if (i < 0)
        return 0;
    if (i > cap-1)
        return cap - 1;
    return i;
}

//void Sim::addVector( double magnitude, double angle ) {
//    double x1 = x + magnitude * cos(angle);
//    double y1 = y + magnitude * sin(angle);
//    window->addVector(x, y, x1, y1);
//}

//
// The "cheat" methods.  See comment in header file.
//

double Sim::cheatGetAngleToGoal( double sx, double sy ) {
    double dx = sx - x;
    double dy = sy - y;
    return Angles::constrainAngle(atan2(dy, dx) - theta);
}

double Sim::cheatGetDistanceToGoal( double sx, double sy ) {
    double dx = sx - x;
    double dy = sy - y;
    return sqrt(dx*dx + dy*dy);
}
