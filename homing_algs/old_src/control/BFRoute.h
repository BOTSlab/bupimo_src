/**
 * BFRoute stands for "Bayes Filter Route".  This structure represents the
 * information stored when learning a feature-based route for incremental
 * homing.
 *
 * @Andrew Vardy
 */

#ifndef BFROUTE_H
#define BFROUTE_H

#include "ImgSequence.h"

class BFRoute {
public:
    BFRoute(string inSSeqDir);

    virtual ~BFRoute();

private:
    void loadMatchVector(string suffix, int j, int *&M);

public:
    
    string sSeqDir;
    ImgSequence sSeq;
    int nS, nP;

    string parentSeqDir;

    int *g2p;
    int *goal;

    // The match tables are stored as 2-D arrays, indexed first by p, then
    // by the index of the feature from the corresponding snapshot.
    int **Mprev;
    int **Mnext;
};

#endif
