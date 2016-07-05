#include "ImgSequence.h"

int main( int argc, char *argv[] ) {
    ImgSequence seq("/data/seq/engrLobbyLoop");

    Pose lastPose = seq.getTruePose(0);
    double distance = 0;
    for (int i=1; i<seq.getLength(); i++) {
        Pose pose = seq.getTruePose(i);
        double dx = pose.x - lastPose.x;
        double dy = pose.y - lastPose.y;
        distance += sqrt(dx*dx + dy*dy);

        lastPose = pose;
    }

    cout << "Total distance travelled: " << distance << endl;

    return 0;
}
