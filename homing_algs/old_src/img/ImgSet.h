/*
 * Loads and manages a set of images.  These images should all be found in the
 * same directory and should be named as follows:
 *
 *  PREFIX_X_Y.EXT
 *
 * PREFIX can be anything (but must be known).  (X, Y) represents the 2-D
 * coordinates of the image's capture position.  EXT must correspond to a format
 * that OpenCV understands (e.g. .jpg, .png).
 *
 * All images will be loaded upon construction.  The 'getImg' method is used
 * to access the image with the closest matching coordinates.
 *
 * Andrew Vardy
 */
#ifndef IMGSET_H
#define IMGSET_H

#include "Img.h"
#include "ImgWindow.h"
#include <cv.h>
#include <vector>
#include <iostream>
using namespace std;

class ImgRecord {
public:
    ImgRecord(Img *img, double x, double y) :
        img(img), x(x), y(y)
    {}
    Img *img;
    double x, y;
};

class ImgSet {
public:
    // Constructor and destructor
    ImgSet(string dirName, string prefix);
    ~ImgSet();

    // Accessors...

    /**
     * Obtains the image which was captured from the closest position to (x, y,
     * theta).  The pointer 'img' will be set to a newly created image if it is
     * NULL.  If the image pointed at by 'img' is already of the correct size
     * it is reused.  If the closest distance exceeds maxDistance then the 
     * return value is set to false (otherwise it is true).
     */
    bool getImg(double x, double y, double theta, Img *&img,
                double maxDistance=DBL_MAX);

    int getImageWidth();
    int getImageHeight();

private:
    void loadFiles(string dirName, string prefix);

    vector<ImgRecord> records;
    int imageWidth, imageHeight;
};

#endif
