#include "ImgSet.h"
#include "ImgOps.h"
#include "Angles.h"
#include <sstream>
#include <cmath>
//#include <glog/logging.h>

// Needed for 'loadFiles'
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

ImgSet::ImgSet(string dirName, string prefix) {

    std::cout << "Loading from " << dirName << " prefix: "
              << prefix << endl;

    loadFiles(dirName, prefix);
    
    if (records.size() > 0) {
        imageWidth = records[0].img->getWidth();
        imageHeight = records[0].img->getHeight();
    } else {
        imageWidth = -1;
        imageHeight = -1;
    }

    std::cout << "Loaded " << records.size() << " records." << endl;
}

void ImgSet::loadFiles(string dirName, string prefix) {

    // Open the given directory
    DIR *dp;
    struct dirent *dirp;
    if((dp = opendir(dirName.c_str())) == NULL) {
        std::cout << "Error(" << errno << ") opening " << dirName << endl;
        return;
    }

    // Process all files, adding those with the correct prefix to 'records'.
    while ((dirp = readdir(dp)) != NULL) {
        string fileName(dirp->d_name);
        //cout << "fileName: " << fileName << endl;

        // There should be two underscore characters; one delimits the prefix
        // from the x-coordinate; the other the x- from the y-coordinates.
        // There may be one in the prefix so we search backwards to find the
        // two of interest here.
        int underScore2 = fileName.find_last_of('_');
        int underScore1 = fileName.find_last_of('_', underScore2-1);

        // The y-coordinate is delimited from the filename extension by the
        // final period.
        int lastPeriod = fileName.find_last_of('.');

        if (underScore1 == string::npos || underScore2 == string::npos ||
            lastPeriod == string::npos) {
            //cout << "Could not find delimiters: NOT a match" << endl;
            continue;
        }

        string prefixFound = fileName.substr(0, underScore1);
        if (prefixFound != prefix) {
            //cout << "Prefix could not be found: NOT a match" << endl;
            continue;
        }

        string ext = fileName.substr(lastPeriod + 1);
        if (!(ext == "pgm" || ext == "png" || ext == "jpg" || ext == "grd")) {
            //cout << "extension -->" << ext << "<--" << endl;
            //cout << "Bad extension: NOT a match" << endl;
            continue;
        }
    
        string xCoord = fileName.substr(underScore1+1, 
                                        underScore2 - underScore1 - 1);
        string yCoord = fileName.substr(underScore2+1, 
                                        lastPeriod - underScore2 - 1);

        istringstream xConvert(xCoord);
        istringstream yConvert(yCoord);
        double x, y;
        xConvert >> x;
        yConvert >> y;
        //cout << "(x, y): " << x << ", " << y << endl;
        //cout << "ImgSet: Loading " << dirName + "/" + fileName << endl;

        records.push_back(ImgRecord(new Img(dirName + "/" + fileName), x, y));
    }
    closedir(dp);
}

ImgSet::~ImgSet() {
    for (int i=0; i<records.size(); i++)
        delete records[i].img;
}

int ImgSet::getImageWidth() { return imageWidth; }
int ImgSet::getImageHeight() { return imageHeight; }

bool ImgSet::getImg(double x, double y, double theta, Img *&img,
                    double maxDistance) {
    double smallestDist = DBL_MAX;
    int closestI = -1;
    for (int i=0; i<records.size(); i++) {
        double dx = x - records[i].x;
        double dy = y - records[i].y;
        double dist = sqrt(dx*dx + dy*dy);
        if (dist < smallestDist && dist <= maxDistance) {
            smallestDist = dist;
            closestI = i;
        }
    }

    if (closestI == -1)
        return false;

    // Rotate image to match the given orientation.
    Img *unrotated = records[closestI].img;
    int shift = Angles::angle2int(theta, unrotated->getWidth());
    ImgOps::rotate(unrotated, shift, img);
    return true;
}
