#include "ImgSequence.h"
#include "SimWindow.h"
#include "SiftExtractor.h"
#include "Angles.h"
#include <sstream>
#include <iomanip>
#include <fstream>

int main() {

    // Experiment
    string sDirName("inco4even");
    string cDirName("inco4odd");

    // Experiment
//    string sDirName("inco4fourth");
//    string cDirName("inco4");

//    string sDirName("inco4eighth");
//    string cDirName("inco4");

//    string sDirName("engrLobbyLoopEven");
//    string cDirName("engrLobbyLoopOdd");

//    string sDirName("engrLobbyLoopfourth");
//    string cDirName("engrLobbyLoop");

//    string sDirName("engrLobbyLoopeighth");
//    string cDirName("engrLobbyLoop");

    // Create directory for output.
    ostringstream command, resultsFilename;
    command << "rm -fr " << sDirName << "; mkdir " << sDirName;
    system(command.str().c_str());
    resultsFilename << sDirName << "/results.txt";
    ofstream resultsFile(resultsFilename.str().c_str());
    resultsFile << "sDir: " << sDirName << endl;
    resultsFile << "cDir: " << cDirName << endl;

    ostringstream sDir, cDir;
    sDir << "/data/seq/" << sDirName;
    cDir << "/data/seq/" << cDirName;
    ImgSequence sSeq(sDir.str()), cSeq(cDir.str());
    int nS = sSeq.getLength(), nC = cSeq.getLength();

    vector<Keypoint*> sKeysArray[nS], cKeysArray[nC];

    // Create extractor.
    assert(sSeq.getImageWidth() == cSeq.getImageWidth());
    SiftExtractor extractor(sSeq.getImageWidth(), sSeq.getImageHeight());

    // We expect the keypoints to be pre-computed.  Load them up for both
    // sequences.
    for (int i=0; i<nS; i++) {
        ostringstream oss;
        oss << sDir.str() << "/" << setfill('0') << setw(3) << i << ".keys";
        SiftExtractor::loadKeys(oss.str(), sKeysArray[i]);
    }
    for (int i=0; i<nC; i++) {
        ostringstream oss;
        oss << cDir.str() << "/" << setfill('0') << setw(3) << i << ".keys";
        SiftExtractor::loadKeys(oss.str(), cKeysArray[i]);
    }

    // Now compute all matches between each image in cSeq and each image in
    // sSeq.
    vector<Match> allMatches[nS][nC];
    for (int c = 0; c < nC; c++) {
        for (int s = 0; s < nS; s++) {
            extractor.match(sKeysArray[s], cKeysArray[c], allMatches[s][c]);
        }
    }

    double **percents = new double*[nS-1];
    double **scales = new double*[nS-1];
    for (int s=0; s<nS-1; s++) {
        percents[s] = new double[nC];
        scales[s] = new double[nC];
    }

    double percentEntropies[nC];
    double scaleEntropies[nC];

    double avgPercentEntropy = 0, avgScaleEntropy = 0,
           avgPercentDistance = 0, avgScaleDistance = 0;

    for (int c = 0; c < nC; c++) {

        resultsFile << "c: " << c << endl;

        // Go through all adjacent pairs of snapshots and compute the two
        // measures for each pair (s, s+1).
        double maxPercent = 0, maxScale = 0;
        int maxPercentS = 0, maxScaleS = 0;
        for (int s = 0; s < nS-1; s++) {
            percents[s][c] = extractor.percentMatched(sKeysArray[s],
                                                      sKeysArray[s+1],
                                                      allMatches[s][c],
                                                      allMatches[s+1][c]);
            if (percents[s][c] > maxPercent) {
                maxPercent = percents[s][c];
                maxPercentS = s;
            }

            scales[s][c] = extractor.scaleDiff(sKeysArray[s], sKeysArray[s+1],
                                               cKeysArray[c],
                                               allMatches[s][c],
                                               allMatches[s+1][c]);
            if (scales[s][c] > maxScale) {
                maxScale = scales[s][c];
                maxScaleS = s;
            }
        }

        // Compute and print the entropy of the measures.
        percentEntropies[c] = 0;
        scaleEntropies[c] = 0;
        double log2 = log(2.0);
        for (int s = 0; s < nS-1; s++) {
            if (percents[s][c] > 1e-10)
                percentEntropies[c] -= percents[s][c] *log(percents[s][c])/log2;
            if (scales[s][c] > 1e-10)
                scaleEntropies[c] -= scales[s][c] * log(scales[s][c])/log2;
        }
        resultsFile << "\tentropy: %: " << percentEntropies[c] << ", n: " 
             << scaleEntropies[c] << endl;
        avgPercentEntropy += percentEntropies[c];
        avgScaleEntropy += scaleEntropies[c];

        // Determine the distance between position at which the maximum was
        // found and the ideal position.
        Pose cPose = cSeq.getTruePose(c);
        double percentX = 0.5*(sSeq.getTruePose(maxPercentS).x 
                           + sSeq.getTruePose(maxPercentS+1).x);
        double percentY = 0.5*(sSeq.getTruePose(maxPercentS).y 
                           + sSeq.getTruePose(maxPercentS+1).y);
        double scaleX = 0.5*(sSeq.getTruePose(maxScaleS).x 
                           + sSeq.getTruePose(maxScaleS+1).x);
        double scaleY = 0.5*(sSeq.getTruePose(maxScaleS).y 
                           + sSeq.getTruePose(maxScaleS+1).y);
        double dx = percentX - cPose.x;
        double dy = percentY - cPose.y;
        double percentDistance = sqrt(dx*dx + dy*dy);
        dx = scaleX - cPose.x;
        dy = scaleY - cPose.y;
        double scaleDistance = sqrt(dx*dx + dy*dy);
        avgPercentDistance += percentDistance;
        avgScaleDistance += scaleDistance;
        resultsFile << "\tdistance: %:" << percentDistance;
        resultsFile << ", n: " << scaleDistance << endl;

        // Print the measures.
        for (int s = 0; s < nS-1; s++) {
            percents[s][c] /= maxPercent;
            scales[s][c] /= maxScale;
            resultsFile << "\ts: " << s << ", %: " << percents[s][c]
                 << "\tn: " << scales[s][c] << endl;
        }

    }

    avgPercentEntropy /= nC;
    avgScaleEntropy /= nC;
    avgPercentDistance /= nC;
    avgScaleDistance /= nC;
    resultsFile << "average entropy: %: " << avgPercentEntropy << ", n: "
         << avgScaleEntropy << endl;
    resultsFile << "average distance: %: " << avgPercentDistance << ", n: "
         << avgScaleDistance << endl;

    // Create a bunch of files to store the results computed thus far.
    ostringstream percentsFilename, scalesFilename, percentEntropiesFilename,
                  scaleEntropiesFilename;
    percentsFilename << sDirName << "/percents.txt";
    scalesFilename << sDirName << "/scales.txt";
    percentEntropiesFilename << sDirName << "/percentEntropies.txt";
    scaleEntropiesFilename << sDirName << "/scaleEntropies.txt";
    ofstream percentsFile(percentsFilename.str().c_str());
    ofstream scalesFile(scalesFilename.str().c_str());
    ofstream percentEntropiesFile(percentEntropiesFilename.str().c_str());
    ofstream scaleEntropiesFile(scaleEntropiesFilename.str().c_str());

    for (int s=0; s<nS-1; s++) {
        for (int c=0; c<nC; c++) {
            percentsFile << percents[s][c] << " ";
            scalesFile << scales[s][c] << " ";
        }
        percentsFile << endl;
        scalesFile << endl;
    }

    for (int c=0; c<nC; c++) {
        percentEntropiesFile << percentEntropies[c] << " ";
        scaleEntropiesFile << scaleEntropies[c] << " ";
    }
    percentEntropiesFile << endl;
    scaleEntropiesFile << endl;

    return 0;
}
