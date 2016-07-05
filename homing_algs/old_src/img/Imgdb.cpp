#include "Imgdb.h"
#include <sstream>

Imgdb::Imgdb(string inDirName, int inWidth, int inHeight) {

    dirName = inDirName;
    width = inWidth;
    height = inHeight;

    ralfDB = false;

    //cout << "Image directory: " << dirName << endl;

    // Create the image array
    imgs = new Img**[width];
    for (int x=0; x<width; x++)
        imgs[x] = new Img*[height];

    // Initialize the image array with NULL's
    for (int x=0; x<width; x++)
        for (int y=0; y<height; y++)
            imgs[x][y] = NULL;

    imageWidth = -1;
    imageHeight = -1;
    loadImg(0, 0); // Load the first image so we can figure out its size.
}

Imgdb::~Imgdb() {
    for (int x=0; x<width; x++) {
        for (int y=0; y<height; y++)
            delete imgs[x][y];
        delete imgs[x];
    }
    delete imgs;
}

void Imgdb::loadImg( int i, int j ) {
    ostringstream oss;

    if (ralfDB)
        // For Ralf's calibrated database:
        oss << dirName << "/cv_" << (width-i-1) << "_" << j << "_bw0.19.pgm";
    else
        //oss << dirName << "/" << i << "_" << j << ".jpg";
        oss << dirName << "/" << i << "_" << j << ".pgm";

    cout << "Loading : " << oss.str() << endl;
    imgs[i][j] = new Img(oss.str());
    if ( imageWidth == -1 ) {
        imageWidth = imgs[i][j]->getWidth();
        imageHeight = imgs[i][j]->getHeight();
    }
}

int Imgdb::getWidth() { return width; }
int Imgdb::getHeight() { return height; }
int Imgdb::getImageWidth() { return imageWidth; }
int Imgdb::getImageHeight() { return imageHeight; }
//string Imgdb::getFloorPlan() { return floorPlan; }

Img* Imgdb::getImg( int i, int j ) {
    if ( imgs[i][j] == NULL )
        loadImg(i, j);
    return imgs[i][j];
}

