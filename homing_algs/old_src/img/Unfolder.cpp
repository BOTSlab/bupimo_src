#include "Unfolder.h"
//#include "ImgWindow.h"
#include <cmath>

Unfolder::Unfolder() {
    // For Bielefeld database images use 369, 283, 167
    //
    // For Pioneer-mounted camera (Dave Churchill's era) use 517, 414, 205
    //
    // The following are for the first iRobot Create mounted camera.
    //centreX = 638;
    //centreY = 518;
    //horizon = 260;

    // The following are for the mirror mounted on the Nikon Coolpix P6000
    //centreX = 2180;
    //centreY = 1610;
    //horizon = 840;

    // The following are for the Bubblescope
    //centreX = 1414;
    //centreY = 893;
    //horizon = 680;
    centreX = 1236;
    centreY = 978;
    horizon = 657;

    // Parameters of mirror
    // Accowle Mirror
    //a = 21.57;
    //b = 32.49;

    // Parameters for Bubblescope (FIRST ATTEMPT)
    a = 0.0303;
    b = 0.05307;

    // Parameters governing the sampling pattern from the input to the output
    // image.
    nRows = 20;
//    nTrim = 0;
    nTrim = 10;
    //maxGamma = 26.0 * M_PI/180.0; // i.e. 26 degrees
    maxGamma = 12.5 * M_PI/180.0; // i.e. 26 degrees

    // Derived parameters (shouldn't need to modify these).
    nRows_2 = nRows / 2;
    nCols = (int)(nRows * M_PI / maxGamma + 0.5);
    a_sqd = a*a;
    b_sqd = b*b;
    c_sqd = a_sqd + b_sqd;
    c = sqrt(c_sqd);
    f = c - b;
    phiHor = gamma2phi(0);
    s = horizon / (f * tan(phiHor));
}

void drawSquare(Img *Debug, int x, int y, int radius, int value) {
    for (int i=x-radius; i<=x+radius; i++)
        for (int j=y-radius; j<=y+radius; j++)
            if (i >= 0 && i < Debug->getWidth() && 
                j >= 0 && j < Debug->getHeight() )
                Debug->set(i, j, value);
}

Img* Unfolder::unfold( Img& Input, Img*& Output, bool debug ) {
    if (Output == 0)
        Output = new Img(nCols, nRows-nTrim);
    else
        assert(Output->getWidth() == nCols && Output->getHeight()==nRows-nTrim);

    Img *Debug = NULL;
    if (debug)
        Debug = new Img(Input);
        // Mark centre on debug image
        drawSquare(Debug, round(centreX), round(centreY), 10, 1);

    double gamma, phi, p, beta, ix, iy;
    for (int row = 0; row<nRows-nTrim; row++) {
        gamma = maxGamma * (nRows_2 - row) / nRows_2;

        phi = gamma2phi(gamma);

        p = s * f * tan(phi);

        for (int col=0; col<nCols; col++) {
//            beta = 2.0*M_PI*(1.0 - col / (double)nCols) - M_PI/2.0;
//          // For the Bubblescope (first attempt)
//            beta = 2.0*M_PI*(col / (double)nCols) - M_PI/2.0;
            beta = 2.0*M_PI*(col / (double)nCols);

            // (ix,iy) are the coordinates cooresponding to (beta,gamma) in the
            // input image.
            ix = centreX + p * cos(beta);
            iy = centreY + p * sin(beta);

            Output->set(col, row, interpolate(Input, ix, iy));

            if (debug) {
                if (gamma == 0)
                    drawSquare(Debug, round(ix), round(iy), 3, 1);
                else
                    drawSquare(Debug, round(ix), round(iy), 3, 0);
            }
        }
    }

    if (debug)
        return Debug;
    else
        return NULL;
}

double Unfolder::interpolate( Img &Input, double x, double y ) {
// Apply bilinear interpolation on the four nearest neighbours of (x,y)

    int flx = (int) floor(x);
    int fly = (int) floor(y);
    int clx = (int) ceil(x);
    int cly = (int) ceil(y);

    double dx = x - flx;
    double valA = Input.get(clx, cly)*dx + Input.get(flx, cly)*(1 - dx);
    double valB = Input.get(clx, fly)*dx + Input.get(flx, fly)*(1 - dx);
    double dy = y - fly;

    return valA*dy + valB*(1 - dy);
}

double Unfolder::gamma2phi( double gamma ) {
// phi is the angle that the reflected ray of light makes to the vertical
// at the mirror's other focus.  We assume that the camera system is properly
// focussed, meaning that the camera's centre of projection is at (0, -c)
// and the focal length is (c - b).

    // (xp, yp) is the intersection point of the ray of light with the 
    // mirror's surface.
    double tan_gamma = tan(gamma);
    double t = a_sqd * tan_gamma*tan_gamma - b_sqd;
    double u = 2 * a_sqd * c * tan_gamma;
    double v = a_sqd * (c_sqd - b_sqd);
    double xp1 = (-u - sqrt(u*u - 4 * t * v)) / (2 * t);
    double xp2 = (-u + sqrt(u*u - 4 * t * v)) / (2 * t);
    double xp = max(xp1, xp2);
    double yp = tan_gamma * xp + c;

    double dy_dx = b * xp / (a_sqd * sqrt(xp*xp / a_sqd + 1));
    double alpha = atan(dy_dx);

    double zeta = gamma - 2 * alpha + M_PI;

    return zeta - M_PI/2;
}

int Unfolder::round( double x ) {
    return (int)(x + 0.5);
}
