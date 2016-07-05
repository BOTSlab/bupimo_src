/*
 * An image sequence class.  Store a sequence of images and associated data.
 * Currently, the associated data for each image are as follows:
 *  - The true position at which the image was captured (x, y, theta).
 *  - The position estimated from odometry (x, y, theta).
 *
 * The images themselves will be loaded as needed when 'getImg' is called.
 *
 * REMOVED:
 *  - The covariance matrix of the odometry estimate (3x3 matrix stored as
 *    length 9 array).
 *
 * Andrew Vardy
 */
#ifndef IMGSEQUENCE_H
#define IMGSEQUENCE_H

#include "ImgWindow.h"
#include "Pose.h"
#include <vector>
#include <iostream>
#include <cv.h>
using namespace std;

class ImgSequence {
public:
    // Constructor and destructor
    ImgSequence(string inDirName);
    ~ImgSequence();

    // Accessors...
    int getLength();
    int getImageWidth();
    int getImageHeight();
    Img* getImg( int i );

    Pose& getTruePose( int i );
    Pose& getOdoPose( int i );
//    double* getCovArray( int i ); // BAD: Caller could modify array.
    double getMinX();
    double getMinY();
    double getMaxX();
    double getMaxY();

private:
    void countImages();
    void readLine( string filename, ifstream &in, string &line );
    void loadAllPoses();
//    void loadAllCovArrays();
    void loadPoseFile(Pose* array, string filename);
    void loadImg( int i );

    string dirName;
    vector<Img*> imgs;
    int imageWidth, imageHeight;

    Pose *truePoses;
    Pose *odoPoses;
//    double **covArrays;

    // Bounds of the poses contained in truePoses.
    double minX, minY, maxX, maxY;
};

#endif
