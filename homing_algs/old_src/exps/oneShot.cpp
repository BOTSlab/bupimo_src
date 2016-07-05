/**
 * Processes a pairs of images specified on the command line and compute a home
 * vector.
 *
 * \author Andrew Vardy
 */

#include "TotalHomingAlg.h"

#include <sstream>
#include <iomanip>
#include <fstream>

int main(int argc, char *argv[]) {

    if (argc != 3) {
        cout << "usage:\n\toneShot IMAGE_S IMAGE_C" << endl;
        return 0;
    }

    Img S(argv[1]);
    Img C(argv[2]);
    int width = S.getWidth();
    int height = S.getHeight();

    // Instantiate homing algorithm.
    TotalHomingAlg *homingAlg =
            TotalHomingAlg::createFromConfig(width, height);

    // Pass snapshot image.
    homingAlg->snapshot(&S);

    // Compute home angle.
    double homeAngle, similarity;
    homingAlg->getHomeAngle(&C, homeAngle, similarity);

    cout << "\thomeAngle: " << homeAngle << endl;
    cout << "\tsimilarity: " << similarity << endl;

    return 0;
}
