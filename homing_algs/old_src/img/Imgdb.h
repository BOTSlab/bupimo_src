/*
 * An image database class.  Essentially this is a matrix of images.
 *
 * Andrew Vardy
 */
#ifndef IMGDB_H
#define IMGDB_H

#include <iostream>
#include <cv.h>
#include "Img.h"
#include "ImgWindow.h"
using namespace std;

class Imgdb {
public:
    // Constructor and destructor
    Imgdb(string inDirName, int inWidth, int inHeight);
    ~Imgdb();

    // Accessors...
    Img* getImg( int i, int j );
    int getWidth();
    int getHeight();
    int getImageWidth();
    int getImageHeight();
    //string getFloorPlan();

private:
    void loadImg( int i, int j );

    int width, height;
    int imageWidth, imageHeight;
    string dirName;//, floorPlan;
    bool ralfDB;

    Img ***imgs;
};

#endif
