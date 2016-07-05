#include "ImgSequence.h"
#include <sstream>
#include <iomanip>
#include <cassert>

// Needed for 'countImages'
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

ImgSequence::ImgSequence(string inDirName)
    : dirName(inDirName)
{
    //cout << "Image directory: " << dirName << endl;

    countImages();

    loadAllPoses();

//    loadAllCovArrays();

    // Load the first image to get its width and height.
    getImg(0);
    imageWidth = imgs[0]->getWidth();
    imageHeight = imgs[0]->getHeight();

    minX = truePoses[0].x;
    minY = truePoses[0].y;
    maxX = truePoses[0].x;
    maxY = truePoses[0].y;
    for (int i=1; i<getLength(); i++) {
        if (truePoses[i].x < minX) minX = truePoses[i].x;
        if (truePoses[i].y < minY) minY = truePoses[i].y;
        if (truePoses[i].x > maxX) maxX = truePoses[i].x;
        if (truePoses[i].y > maxY) maxY = truePoses[i].y;
    }
}

ImgSequence::~ImgSequence() {
    for (int i=0; i<getLength(); i++)
        delete imgs[i];
/*
    for (int i=0; i<getLength(); i++)
        delete covArrays[i];
    delete covArrays;
*/
}

void ImgSequence::countImages() {

    // Open the given directory
    DIR *dp;
    struct dirent *dirp;
    if((dp = opendir(dirName.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dirName << endl;
        return;
    }

    // Count the appropriately named image files.
    int n = 0;
    while ((dirp = readdir(dp)) != NULL) {
        string fileName(dirp->d_name);
        //cout << "fileName: " << fileName << endl;

        int lastPeriod = fileName.find_last_of('.');

        string ext = fileName.substr(lastPeriod + 1);
        if (!(ext == "bmp")) {
            // Not an appropriate image file (but could be true.txt or odo.txt
            // or something else acceptable---so this is not an error!).
            continue;
        }
    
        n++;
    }
    closedir(dp);

    for (int i=0; i < n; i++)
        // A NULL entry implies the image has not been loaded yet.
        imgs.push_back(NULL);
}

void ImgSequence::loadAllPoses() {
    truePoses = new Pose[getLength()];
    odoPoses = new Pose[getLength()];
    
    loadPoseFile(truePoses, "true.txt");
    loadPoseFile(odoPoses, "odo.txt");
}

void ImgSequence::readLine( string filename, ifstream &in, string &line ) {
    if ( !getline(in, line) ) {
        cerr << "ImgSequence: Problem reading from file " << filename << endl;
        exit(-1);
    }
    if (line[0] == '#') {
        readLine(filename, in, line);
    }
}

void ImgSequence::loadPoseFile(Pose* array, string filename) {
    string fullFilename = dirName + "/" + filename;
    ifstream in(fullFilename.c_str());
    //cout << "Loading filename: " << fullFilename << endl;

    string line;
    for (int i=0; i<getLength(); i++) {
        readLine(fullFilename, in, line);

        stringstream lineStream(line);
        float x, y, theta;
        lineStream >> x;
        lineStream >> y;
        lineStream >> theta;
        //cout << "Reading: " << x << " " << y << " " << theta << endl;
        array[i].x = x;
        array[i].y = y;
        array[i].theta = theta;
    }
}

/*
void ImgSequence::loadAllCovArrays() {
    covArrays = new double*[getLength()];
    for (int i=0; i<getLength(); i++)
        covArrays[i] = new double[9];
    
    string fullFilename = dirName + "/cov.txt";
    ifstream in(fullFilename.c_str());
    cout << "Loading filename: " << fullFilename << endl;

    string line;
    for (int i=0; i<getLength(); i++) {
        readLine(fullFilename, in, line);

        stringstream lineStream(line);
        //cout << "Reading: " << endl;
        for (int j=0; j<9; j++) {
            double cov;
            lineStream >> cov;
            covArrays[i][j] = cov;
            //cout << cov << " ";
        }
        //cout << endl;
    }
}
*/

int ImgSequence::getLength() { return imgs.size(); }
int ImgSequence::getImageWidth() { return imageWidth; }
int ImgSequence::getImageHeight() { return imageHeight; }

Img* ImgSequence::getImg( int i ) {
    if (imgs[i] == NULL) {
        // Image has not been loaded before.  Load it and return it.
        ostringstream oss;
        oss << dirName << "/";
        oss << setfill('0') << setw(3);
        oss << i;
        oss << ".bmp";
        Img *img = new Img(oss.str());
        imgs[i] = img;
        return img;
    } else {
        return imgs[i];
    }
}

Pose& ImgSequence::getTruePose( int i ) {
    return truePoses[i];
}

Pose& ImgSequence::getOdoPose( int i ) {
    return odoPoses[i];
}

/*
double* ImgSequence::getCovArray( int i ) {
    return covArrays[i];
}
*/

double ImgSequence::getMinX() { return minX; }
double ImgSequence::getMinY() { return minY; }
double ImgSequence::getMaxX() { return maxX; }
double ImgSequence::getMaxY() { return maxY; }
