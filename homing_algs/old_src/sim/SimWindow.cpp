#include "SimWindow.h"
#include <cv.h>
#include <sstream>

SimWindow::SimWindow( string title,
                      float minX, float minY, float maxX, float maxY,
                      bool saveToFile ) {
    pls = new plstream;

    // Initialize the device type and window size
    int width;
    if (saveToFile) {
        pls->setopt("dev", "svgcairo");
        ostringstream oss;
        oss << title << ".svg";
        pls->setopt("o", oss.str().c_str());
        width = 1000;
    } else {
        pls->setopt("dev", "xcairo");
// pls->SetOpt("dev", "xwin");
        width = 500;
    }

    double aspect = (maxX - minX) / (maxY - minY);
    int height = (int)(width / aspect);

    maxDim = max(maxX - minX, maxY - minY);

    ostringstream oss;
    oss << width << "x" << height;
    pls->setopt("geometry", oss.str().c_str());

    // Make some changes to color map 0, which is used for the foreground,
    // background, and axes
    pls->scol0(0, 255, 255, 255); // Change background to white
    pls->scol0(1, 50, 50, 50);    // Change axes colour
    pls->scol0(15, 0, 0, 0);      // Change foreground (roughly) to black

    // Setup color map 1, used for plotting the data.  It is set here to a
    // grayscale color map, with 0 corresponding to black, and 1 to white.
    i[0] = 0;
    i[1] = 1;
    r[0] = 0;
    r[1] = 1;
    g[0] = 0;
    g[1] = 1;
    b[0] = 0;
    b[1] = 1;
    bool rev = false;
    pls->scmap1l(true, 2, i, r, g, b, &rev);

    // Setup the viewport and coordinates of the axes
    pls->init();
    pls->env(minX, maxX, minY, maxY, 1, 0);

    // Title of plot
//    pls->mtex("t", 1, 0.5, 0.5, title.c_str());

    pls->col0(15);

    // Line width
    pls->width(0.00025);

    // Character height
    pls->schr(2.5, 1);


    // Setup for drawing vectors
    pls->Alloc2dGrid(&grid.xg, 1, 1);
    pls->Alloc2dGrid(&grid.yg, 1, 1);
    pls->Alloc2dGrid(&u, 1, 1);
    pls->Alloc2dGrid(&v, 1, 1);
    grid.nx = 1;
    grid.ny = 1;
    // Setup arrow style
    PLFLT arrow_x[6] = {-0.5, 0.5, 0.3, 0.5, 0.3, 0.5};
    PLFLT arrow_y[6] = {0.0, 0.0, 0.2, 0.0, -0.2, 0.0};
    pls->svect(arrow_x, arrow_y, 6, 1);
}

SimWindow::~SimWindow() {
    pls->Free2dGrid(grid.xg, 1, 1);
    pls->Free2dGrid(grid.yg, 1, 1);
    pls->Free2dGrid(u, 1, 1);
    pls->Free2dGrid(v, 1, 1);
    delete pls;
}

void SimWindow::addPoint( double x, double y, int code, float size ) {
    // Symbol height
    pls->ssym(size, 1);

    pls->poin(1, &x, &y, code);
    pls->flush();
}

/**
 * Add the point (x, y) with code 'code' with respect to a new coordinate
 * system defined at (xi, yi, theta).
 */
void SimWindow::addPointWRT( double xi, double yi, double theta,
                             double x, double y, int code ) {
    double angle = atan2(y, x);
    double r = sqrt(x*x + y*y);
    double px = xi + r * cos(angle + theta);
    double py = yi + r * sin(angle + theta);
    addPoint(px, py, code);
}

void SimWindow::addLine( double x0, double y0,
                         double x1, double y1 ) {
    pls->join(x0, y0, x1, y1);
    pls->flush();
}

/*
 * Similar to addPointWRT, only now there are a pair of points to transform.
 */
void SimWindow::addLineWRT( double xi, double yi, double theta,
                            double x0, double y0, double x1, double y1 ) {
    double angle0 = atan2(y0, x0);
    double r0 = sqrt(x0*x0 + y0*y0);
    double px0 = xi + r0 * cos(angle0 + theta);
    double py0 = yi + r0 * sin(angle0 + theta);

    double angle1 = atan2(y1, x1);
    double r1 = sqrt(x1*x1 + y1*y1);
    double px1 = xi + r1 * cos(angle1 + theta);
    double py1 = yi + r1 * sin(angle1 + theta);
    addLine(px0, py0, px1, py1);
}

void SimWindow::addVector( double x0, double y0,
                           double x1, double y1,
                           double color ) {
    pls->col1(color);

    double vx = x1 - x0;
    double vy = y1 - y0;
    grid.xg[0][0] = x0 + vx/2.0;
    grid.yg[0][0] = y0 + vy/2.0;
    u[0][0] = vx;
    v[0][0] = vy;

    pls->width(2);
    plvect(u, v, 1, 1, 2, plstream::tr2, (void *)&grid);
    pls->flush();
    pls->width(0);
}

void SimWindow::plotCircle( double cx, double cy, double radius ) {
//    if (radius < 0.0005*maxDim) {
//        return;
//    }

    double deltaT = 0.1;

    double t = 0;
    double firstX = cx + radius;
    double firstY = cy;
    double lastX = firstX;
    double lastY = firstY;
    t = deltaT;
    for ( ; t<2*M_PI; t += deltaT ) {
        double x = cx + radius*cos(t);
        double y = cy + radius*sin(t);
        addLine(lastX, lastY, x, y);
        lastX = x;
        lastY = y;
    }
    addLine(lastX, lastY, firstX, firstY);
}

/*
 * Draw the 90% uncertainty ellipse for the upper 2x2 block of the 3x3 matrix
 * given in 'array'.
 */
void SimWindow::plotUncertaintyEllipse( double cx, double cy, double array[] ) {
    // Critical value for the Chi-squared distribution for
    // alpha = 0.9 and two dimensions.  Obtained from page 907
    // of "Applied Statistics for Engineers and Scientists", by
    // Petruccelli, Nandram, and Chen, 1999.
    double c = sqrt(4.605);

    CvMat SigmaClone, Evecs, Evals;
    double sigmaClone[] = { array[0], array[1], array[3], array[4] };
    double evecs[4];
    double evals[2];
    cvInitMatHeader(&SigmaClone,2,2,CV_64FC1, sigmaClone);
    cvInitMatHeader(&Evecs,2,2,CV_64FC1, evecs);
    cvInitMatHeader(&Evals,1,2,CV_64FC1, evals);

    cvEigenVV(&SigmaClone, &Evecs, &Evals);

    // The length of the minor axis is given by c / sqrt(lambda1).  We compute
    // this e-value from Sigma, as opposed to Sigma^-1, so the e-value is the
    // reciprocal.
    double minor = c * sqrt(evals[1]);

    // As above, only for the major axis
    double major = c * sqrt(evals[0]);

    // Angle of e-vector with larger e-value
    double phi = atan2(evecs[1], evecs[0]);

//    window.plotEllipseWRT(lastTeleportX, lastTeleportY, lastTeleportTheta,
    plotEllipse(cx, cy, major, minor, phi);
}

void SimWindow::plotEllipse( double cx, double cy, double major, double minor,
                             double phi ) {
    double deltaT = 0.2;

    double t = 0;
    double firstX = cx + major*cos(phi)*cos(t) - minor*sin(phi)*sin(t);
    double firstY = cy + major*sin(phi)*cos(t) + minor*cos(phi)*sin(t);
    double lastX = firstX;
    double lastY = firstY;
    t = deltaT;
    for ( ; t<2*M_PI; t += deltaT ) {
        double x = cx + major*cos(phi)*cos(t) - minor*sin(phi)*sin(t);
        double y = cy + major*sin(phi)*cos(t) + minor*cos(phi)*sin(t);
        addLine(lastX, lastY, x, y);
        lastX = x;
        lastY = y;
    }
    addLine(lastX, lastY, firstX, firstY);
}

void SimWindow::plotEllipseWRT( double xi, double yi, double theta, double cx,
                                double cy, double major, double minor,
                                double phi ) {
    double deltaT = 0.2;

    double t = 0;
    double firstX = cx + major*cos(phi)*cos(t) - minor*sin(phi)*sin(t);
    double firstY = cy + major*sin(phi)*cos(t) + minor*cos(phi)*sin(t);
    double lastX = firstX;
    double lastY = firstY;
    t = deltaT;
    for ( ; t<2*M_PI; t += deltaT ) {
        double x = cx + major*cos(phi)*cos(t) - minor*sin(phi)*sin(t);
        double y = cy + major*sin(phi)*cos(t) + minor*cos(phi)*sin(t);
        addLineWRT(xi, yi, theta, lastX, lastY, x, y);
        lastX = x;
        lastY = y;
    }
    addLineWRT(xi, yi, theta, lastX, lastY, firstX, firstY);
}

void SimWindow::plotEllipseFromTo(double x0, double y0, double x1, double y1,
                                  double width) {

    double cx = 0.5*(x0 + x1);
    double cy = 0.5*(y0 + y1);
    double major = 0.5*sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
    double phi = atan2(y1 - y0, x1 - x0);

    plotEllipse(cx, cy, major, width, phi);
}

void SimWindow::clear() {
    pls->clear();
}

void SimWindow::plotText(double x, double y, const char* text) {
    pls->ptex(x, y, 1, 0, 0, text);
}
