/*
 * Represents a stored sequence of robot-captured images and associated robot
 * odometry data.
 *
 * Andrew Vardy
 */
#ifndef SEQWINDOW_H
#define SEQWINDOW_H

#include <cmath>
#include "ImgSequence.h"
#include "SimWindow.h"
#include "Vec2.h"
#include "Angles.h"

class SeqWindow {
public:
    SeqWindow(string title, ImgSequence &inSequence, bool saveToFile=false);
    ~SeqWindow();
    void addVector(double x, double y,
                   double magnitude, double angle, double color = 0);
    void addPoint(double x, double y, int code);
    void addCircle(double x, double y, double radius);

    void addVector(int index, double magnitude, double angle, double color = 0);
    void addPoint(int index, int code);
    void addCircle(int index, double radius);

    void clear();

protected:
    ImgSequence &imgSequence;
    SimWindow *window;
    double visibleVectorLength;
};

#endif
