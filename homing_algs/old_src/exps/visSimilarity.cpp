#include "ImgSequence.h"
#include "SimWindow.h"
#include "SiftExtractor.h"
#include "Angles.h"
#include <sstream>
#include <iomanip>
#include <fstream>

double pointSize, radiusConstant, plusCrossSize;

void visualize(ImgSequence &sSeq, ImgSequence &cSeq, 
               double **percents, double **scales,
               double *percentEntropies, double *scaleEntropies );
void visualize(SimWindow &window, ImgSequence &sSeq, ImgSequence &cSeq,
               double **measures, int c, double xOffset, double yOffset);

int main() {

    /*
    string sDirName("engrLobbyLoopfourth");
    string cDirName("engrLobbyLoop");
    pointSize = 0.1;
    radiusConstant = 0.45;
    plusCrossSize = 2.5;
    */

    /*
    string sDirName("inco4fourth");
    string cDirName("inco4");
    pointSize = 0.1;
    radiusConstant = 0.45;
    plusCrossSize = 2.5;
    */

    /*
    string sDirName("inco4eighth");
    string cDirName("inco4");
    pointSize = 0.2;
    radiusConstant = 0.6;
    plusCrossSize = 3.0;
    */

    /*
    string sDirName("engrLobbyLoopeighth");
    string cDirName("engrLobbyLoop");
    pointSize = 0.2;
    radiusConstant = 0.6;
    plusCrossSize = 3.0;
    */

    string sDirName("inco4even");
    string cDirName("inco4odd");
    pointSize = 0.4;
    radiusConstant = 0.6;
    plusCrossSize = 7.0;

    /*
    string sDirName("engrLobbyLoopEven");
    string cDirName("engrLobbyLoopOdd");
    pointSize = 0.2;
    radiusConstant = 0.6;
    plusCrossSize = 3.0;
    */

    ostringstream sDir, cDir;
    sDir << "/data/seq/" << sDirName;
    cDir << "/data/seq/" << cDirName;
    ImgSequence sSeq(sDir.str()), cSeq(cDir.str());
    int nS = sSeq.getLength(), nC = cSeq.getLength();

    double **percents = new double*[nS-1];
    double **scales = new double*[nS-1];
    for (int s=0; s<nS-1; s++) {
        percents[s] = new double[nC];
        scales[s] = new double[nC];
    }

    double percentEntropies[nC];
    double scaleEntropies[nC];

    // Load the various files where our results are stored.
    ostringstream percentsFilename, scalesFilename, percentEntropiesFilename,
                  scaleEntropiesFilename;
    percentsFilename << sDirName << "/percents.txt";
    scalesFilename << sDirName << "/scales.txt";
    percentEntropiesFilename << sDirName << "/percentEntropies.txt";
    scaleEntropiesFilename << sDirName << "/scaleEntropies.txt";
    ifstream percentsFile(percentsFilename.str().c_str());
    ifstream scalesFile(scalesFilename.str().c_str());
    ifstream percentEntropiesFile(percentEntropiesFilename.str().c_str());
    ifstream scaleEntropiesFile(scaleEntropiesFilename.str().c_str());

    for (int s=0; s<nS-1; s++) {
        for (int c=0; c<nC; c++) {
            percentsFile >> percents[s][c];
            scalesFile >> scales[s][c];
        }
    }

    for (int c=0; c<nC; c++) {
        percentEntropiesFile >> percentEntropies[c];
        scaleEntropiesFile >> scaleEntropies[c];
    }

    visualize(sSeq, cSeq, percents, scales, percentEntropies, scaleEntropies);

    return 0;
}

void visualize(ImgSequence &sSeq, ImgSequence &cSeq, 
               double **percents, double **scales,
               double *percentEntropies, double *scaleEntropies ) {

    int nS = sSeq.getLength(), nC = cSeq.getLength();
    
    // Create a window for display
    double minX = cSeq.getMinX();
    double minY = cSeq.getMinY();
    double maxX = cSeq.getMaxX();
    double maxY = cSeq.getMaxY();

    double width = 1.1 * (maxX - minX);
    double heightPerSection = 1.05 * (maxY - minY) + 1.0;
    double height = heightPerSection*nC;
    double margin = 0.05 * width;

    SimWindow window("out", minX - margin, minY - margin, minX + 2*width + margin, minY + height + margin, true);

    for (int c=0; c<nC; c++) {
        visualize(window, sSeq, cSeq, percents, c, 0, c*heightPerSection);
        visualize(window, sSeq, cSeq, scales, c, width, c*heightPerSection);
    }

    // Draw the entropy to the right of the final snapshot position.
//    Pose sPose = sSeq.getTruePose(nS-1);
//    double xText = sSeq.getMaxX() + 5 * radiusFactor + xOffset;
//    double yText = 0.5*(sSeq.getMinY() + sSeq.getMaxY()) + yOffset;
//    ostringstream oss;
//    oss << entropy;
//    window.plotText(xText, yText, oss.str().c_str());
}

void visualize(SimWindow &window, ImgSequence &sSeq, ImgSequence &cSeq,
               double **measures, int c, double xOffset, double yOffset) {

    int nS = sSeq.getLength(), nC = cSeq.getLength();
    
    // Fill in marks at all current positions
    for (int ci=0; ci<nC; ci++) {
        Pose cPose = cSeq.getTruePose(ci);
        window.addPoint(cPose.x + xOffset, cPose.y + yOffset, 24, pointSize);
    }

    // Determine the s-position of maximum belief.
    double maxValue = 0;
    int maxS = 0;
    for (int s=0; s<nS-1; s++) {
        if (measures[s][c] > maxValue) {
            maxValue = measures[s][c];
            maxS = s;
        }
    }

    // Draw circles sized according to the probability of the given distribution
    Pose firstPose = cSeq.getTruePose(0);
    Pose secondPose = cSeq.getTruePose(1);
    double dx = firstPose.x - secondPose.x;
    double dy = firstPose.y - secondPose.y;
    double radiusFactor = radiusConstant*sqrt(dx*dx + dy*dy);
    for (int s=0; s<nS-1; s++) {
        Pose currentPose = sSeq.getTruePose(s);
        Pose nextPose = sSeq.getTruePose(s+1);

        double radius = radiusFactor * measures[s][c];

        window.plotEllipseFromTo(currentPose.x + xOffset, currentPose.y + yOffset, nextPose.x + xOffset, nextPose.y + yOffset, radius);

        // Draw a plus at the position of maximum belief.
        if (s == maxS) {
            double x = 0.5*(currentPose.x + nextPose.x) + xOffset;
            double y = 0.5*(currentPose.y + nextPose.y) + yOffset;
            window.addPoint(x, y, 2, plusCrossSize);
        }
    }

    // Draw an X at the current position.
    Pose currentPose = cSeq.getTruePose(c);
    window.addPoint(currentPose.x + xOffset, currentPose.y + yOffset, 5,
                    plusCrossSize);
}
