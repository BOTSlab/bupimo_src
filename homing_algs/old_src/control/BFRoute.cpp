#include "BFRoute.h"
#include <fstream>
#include <sstream>
#include <cassert>
#include <iomanip>

BFRoute::BFRoute(string inSSeqDir)
    : sSeqDir(inSSeqDir),
      sSeq(sSeqDir),
      nS(sSeq.getLength())
{
    // Determine the directory of the parent sequence.
    ostringstream parentSeqDirFilename;
    parentSeqDirFilename << sSeqDir << "/parentSeqDir.txt";
    ifstream parentSeqDirFile(parentSeqDirFilename.str().c_str());
    parentSeqDirFile >> parentSeqDir;

    // Determine the length of the p sequence.
    ostringstream pLengthFilename;
    pLengthFilename << sSeqDir << "/pLength.txt";
    ifstream pLengthFile(pLengthFilename.str().c_str());
    pLengthFile >> nP;

    // Read the g2p file into the g2p array.
    ostringstream g2pFilename;
    g2pFilename << sSeqDir << "/g2p.txt";
    ifstream g2pFile(g2pFilename.str().c_str());
    g2p = new int[nS];
    for (int s=0; s<nS; s++) {
        g2pFile >> g2p[s];
    }

    // Read the goal file into the goal array.
    ostringstream goalFilename;
    goalFilename << sSeqDir << "/goal.txt";
    ifstream goalFile(goalFilename.str().c_str());
    goal = new int[nP];
    for (int p=0; p<nP; p++) {
        goalFile >> goal[p];
    }

    // Now read match vectors for all p-nodes which were *not* selected
    // as g-nodes (snapshots).  We can access these nodes by iterating
    // through all of the skipped values in g2p.
    //
    // We will create two arrays of vectors and initialize both with NULLs
    // since we should have match vectors only for non-snapshot nodes.
    Mprev = new int*[nP];
    Mnext = new int*[nP];
    for (int j=0; j<nP; j++) {
        if (goal[j] == 1) {
            Mprev[j] = NULL;
            Mnext[j] = NULL;
        } else {
            cout << "\tloading vectors for " << j << endl;
            loadMatchVector(string("prev"), j, Mprev[j]);
            loadMatchVector(string("next"), j, Mnext[j]);
        }
    }

}

void BFRoute::loadMatchVector(string suffix, int j, int *&M) {
    ostringstream MFilename;
    MFilename << sSeqDir << "/M" << suffix << setfill('0') << setw(3)
                  << j << ".txt";
    ifstream MFile(MFilename.str().c_str());

    // Get the length of the file in number of lines.
    int nFeatures = 0;
    char buffer[80];
    while (true) {
        MFile.getline(buffer, 80);
        if (MFile.eof()) break;
        nFeatures++;
    }

    // Now allocate the required-size array.
    M = new int[nFeatures];
    
    // Close the files and open them again so that we can start again at
    // the beginning (BAD).
    MFile.close();
    MFile.open(MFilename.str().c_str());

    // Read each line into the array
    for (int k=0; k<nFeatures; k++)
        MFile >> M[k];
}

BFRoute::~BFRoute() {
    delete [] g2p;

    for (int j=0; j<nP; j++) {
        if (Mprev[j] != NULL) delete [] Mprev[j];
        if (Mnext[j] != NULL) delete [] Mnext[j];
    }
}
