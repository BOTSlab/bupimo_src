/*
 * Computes and displays home vectors for one or more snapshot positions paired
 * in turn with all other positions from an image database.
 *
 * usage:
 *      dbVectorField
 *
 * \author Andrew Vardy
 */

#include "DBVectorField.h"
#include "SingleConfig.h"
//#include <glog/logging.h>

int main( int argc, char ** argv ) {

//    google::InitGoogleLogging(argv[0]);

    SingleConfig *config = SingleConfig::getInstance();
    int sx = config->getInt("homeVectors.sx");
    int sy = config->getInt("homeVectors.sy");

    DBVectorField field;
    int width = field.getWidth();
    int height = field.getHeight();

    // Snapshot position.  If 'allSnapshots' is true then we use all snapshot
    // positions.  If not and 'selectedSnapshots' is true we use the snapshots
    // given in arrays 'sxArray' and 'syArray'.  If not, we use the single
    // snapshot at (sx, sy).
    bool allSnapshots = false;
    bool selectedSnapshots = false;
    int nSS = 4;
    int sxArray[4] = {3, 9, 0, 6};
    int syArray[4] = {4, 4, 12, 12};

    if (allSnapshots) {
        double overallAAE = 0;
        for (int sy=0; sy < height; sy++)
            for (int sx=0; sx < width; sx++)
                overallAAE += field.homeToSnapshot(sx, sy);
        overallAAE /= (width * height);
        std::cout << "Overall AAE: " << overallAAE << endl;

    } else if (selectedSnapshots) {
        double overallAAE = 0;
        for (int i=0; i<nSS; i++)
            overallAAE += field.homeToSnapshot(sxArray[i], syArray[i]);
        overallAAE /= nSS;
        std::cout << "Overall AAE: " << overallAAE << endl;

    } else {
        std::cout << "AAE: " << field.homeToSnapshot(sx, sy) << endl;
    }
}
