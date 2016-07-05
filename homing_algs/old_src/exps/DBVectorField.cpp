#include "DBVectorField.h"
#include "ImgOps.h"
#include "Angles.h"
//#include "SiftExtractor.h"
#include "SingleConfig.h"
#include <vector>
//#include <glog/logging.h>

int Position::sx = 0;
int Position::sy = 0;

DBVectorField::DBVectorField() {
    SingleConfig *config = SingleConfig::getInstance();
    runName = config->getString("DBVectorField.runName");
    ssDir = config->getString("DBVectorField.ssDir");
    cvDir = config->getString("DBVectorField.cvDir");
    width = config->getInt("DBVectorField.width");
    height = config->getInt("DBVectorField.height");
    sortClosest = config->getBool("DBVectorField.sortClosest");
    randomRotation = config->getBool("DBVectorField.randomRotation");
    similarityThreshold = config->getDouble(
                                           "DBVectorField.similarityThreshold");

    ssDB = new Imgdb(ssDir, width, height);
    cvDB = new Imgdb(cvDir, width, height);

    homingAlg = TotalHomingAlg::createFromConfig(ssDB->getImageWidth(),
												 ssDB->getImageHeight());

    // Seed random-number generator with a fixed value.
    srand(1);
}

DBVectorField::~DBVectorField() {
    delete ssDB;
    delete cvDB;
    delete homingAlg;
}

double DBVectorField::home(SimWindow &prevWindow, SimWindow &saveWindow,
                           Img *SS, Img *CV, int cx, int cy, float rotAngle) {

    double homeAngle, similarity;
    homingAlg->getHomeAngle(CV, homeAngle, similarity);

    // Account for the rotation of the current position.
    homeAngle += rotAngle;

    double x1 = cx + 0.5*cos(homeAngle);
    double y1 = cy + 0.5*sin(homeAngle);
    prevWindow.addVector(cx, cy, x1, y1);
    saveWindow.addVector(cx, cy, x1, y1);

    if (similarity > similarityThreshold) {

        // Display the similarity
        double radius = log(100*similarity) / 10.0;
        if (radius > 0) {
            prevWindow.plotCircle(cx, cy, radius);
            saveWindow.plotCircle(cx, cy, radius);
        }
    }

    return homeAngle;
}

double DBVectorField::homeToSnapshot(int sx, int sy) {
    ostringstream title;
    title << runName << "_" << sx << "_" << sy;

    // Create the SimWindow.
    SimWindow prevWindow(title.str(), -1, -1, width, height, false);
    SimWindow saveWindow(title.str(), -1, -1, width, height, true);
    prevWindow.addPoint(sx, sy, 0, 2.5);
    saveWindow.addPoint(sx, sy, 0, 5);

    double AAE = 0;

    // Process snapshot
    Img *SS = ssDB->getImg(sx, sy);
    homingAlg->snapshot(SS);

    // Build a list of current position pairs.
    vector<Position> currentPositions;
    for (int cy=0; cy < height; cy++)
        for (int cx=0; cx < width; cx++)
            if (cx != sx || cy != sy)
                currentPositions.push_back(Position(cx, cy));
    Position::setSnapshot(sx, sy);

    // Sort the current positions based on their distance to (sx, sy).
    if (sortClosest)
        sort(currentPositions.begin(), currentPositions.end(),
             Position::closerToSnapshot);

    int n = (int) currentPositions.size();
    int imgWidth = cvDB->getImageWidth();
    for (int i=0; i<n; i++) {
        int cx = currentPositions[i].x;
        int cy = currentPositions[i].y;

        // Mark the current point in the preview window only.
        prevWindow.addPoint(cx, cy, 1, 2.5);
                
        // Make a randomly rotated copy of the CV image.
        Img *rotatedCV = NULL;
        float rot = rand() / (double) RAND_MAX;
        if (!randomRotation)
            rot = 0;
        float rotAngle = TWO_PI * rot;
        int rotShift = (int)(imgWidth * rot);//imgWidth*(1 - rot) - 1;
        ImgOps::rotate(cvDB->getImg(cx, cy), rotShift, rotatedCV);

        double homeAngle = home(prevWindow, saveWindow, SS, rotatedCV,
                                cx, cy, rotAngle);
        delete rotatedCV;

//log the ideal rotation - RH
/*string outFileName = "./testing/log.txt";
ofstream outFile(outFileName.c_str(), ios_base::app);
outFile << "ideal: " << (imgWidth - rotShift)/2 << endl;
outFile << endl;
outFile.close();*/

        double idealAngle = atan2(sy - cy, sx - cx);
        AAE += Angles::getAngularDifference(idealAngle, homeAngle);
        std::cout << "error: " << Angles::getAngularDifference(idealAngle,
                                                           homeAngle) << endl;
        
    }
    AAE /= n;
    std::cout << title.str() << " AAE: " << AAE << endl;
    return AAE;
}

