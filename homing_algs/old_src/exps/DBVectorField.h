/**
 * Used by dbVectorField to draw the field of home vectors for a pair of image
 * databases, one providing the snapshot images the other providing current
 * images.
 *
 * \author Andrew Vardy
 */

#ifndef DBVECTORFIELD_H
#define DBVECTORFIELD_H

#include "Imgdb.h"
#include "SimWindow.h"
#include "TotalHomingAlg.h"

#include <cmath>
#include <cstdlib>

class Position {
public:
    Position(int inX, int inY) : x(inX), y(inY) {}
    static void setSnapshot(int x, int y) {
        sx = x; sy = y;
    }
    static bool closerToSnapshot(const Position &pos1, const Position &pos2) {
        double dx1 = pos1.x-sx;
        double dy1 = pos1.y-sy;
        double dx2 = pos2.x-sx;
        double dy2 = pos2.y-sy;
        return (dx1*dx1 + dy1*dy1 < dx2*dx2 + dy2*dy2);
    }
    int x, y;
    static int sx, sy;
};


class DBVectorField {
public:
    DBVectorField();
    ~DBVectorField();
    double homeToSnapshot(int sx, int sy);
    int getWidth() { return width; }
    int getHeight() { return height; }
private:
    double home(SimWindow &prevWindow, SimWindow &saveWindow,
                Img *SS, Img *CV, int cx, int cy, float rotAngle);

    string runName, ssDir, cvDir;
    int width, height;
    Imgdb *ssDB, *cvDB;

    bool sortClosest, randomRotation;
    double similarityThreshold;

    TotalHomingAlg *homingAlg;
};

#endif
