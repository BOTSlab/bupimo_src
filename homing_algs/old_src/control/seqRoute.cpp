#include "SeqAdaptor.h"
#include "RouteManager.h"

int main( int argc, char ** argv ) {
    ImgSequence seq("/data/seq/original4");
    SeqAdaptor adaptor(seq);
    RouteManager manager(adaptor);

//    for (int p=0; p<seq.getLength(); p++) {
for (int p=0; p<5; p++) {
        cout << "seqRoute: idealP: " << p << endl;
        bool arrived = false;
        bool routeComplete = false;
        double homeAngle = manager.getHomeAngle(1, arrived, routeComplete);
    }

    return 0;
}
