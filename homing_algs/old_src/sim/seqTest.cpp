#include "SeqWindow.h"

int main( int argc, char ** argv ) {

    ImgSequence seq("/data/seq/inco4eighth");
    SeqWindow seqWindow("inco4", seq, false);

    for (int i=0; i<seq.getLength(); i++) {
        seqWindow.addPoint(i, 2);
    }
} 
