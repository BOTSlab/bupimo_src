/*
 * Visualization class for use with Sim.
 *
 * Andrew Vardy
 */

#ifndef SIMWINDOW_H
#define SIMWINDOW_H

#include <iostream>
#include <cmath>
#include <unistd.h>
#include "plstream.h"
using namespace std;

class SimWindow {
public:
    SimWindow( string title, float inMinX, float inMinY,
               float inMaxX, float inMaxY, bool saveToFile=false );
    ~SimWindow();
    void addPoint( double x, double y, int code, float size=0.0f );
    void addPointWRT( double xi, double yi, double theta,
                     double x, double y, int code );
    void addLine( double x0, double y0, double x1, double y1 );
    void addLineWRT( double xi, double yi, double theta,
                     double x0, double y0, double x1, double y1 );
    void addVector( double x0, double y0, double x1, double y1,
                    double color = 0 );
    void plotCircle( double cx, double cy, double radius );
    void plotUncertaintyEllipse( double cx, double cy, double array[] );
    void plotEllipse( double cx, double cy, double major, double minor,
                      double phi );
    void plotEllipseWRT( double xi, double yi, double theta, double cx,
                         double cy, double major, double minor, double phi );
    void plotEllipseFromTo(double x0, double y0, double x1, double y1,
                           double width);
    void clear();
    void plotText(double x, double y, const char* text);
private:
    plstream *pls;
    double maxDim;

    PLFLT i[2], r[2], g[2], b[2]; // arrays used for setting up color map 1
    PLFLT **u, **v;
    PLcGrid2 grid;
};

#endif
