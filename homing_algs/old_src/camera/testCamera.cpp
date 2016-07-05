#include "OCVCamera.h"
#include <sstream>
#include <cstdlib>
using namespace std;

int main(int argc, char **argv) {
    Camera *camera = new OCVCamera;

    for (int i=0; i<1; i++) {
        Img *img = NULL;
        camera->getImg(img);

        // A random number of multiplication operations
        /*
        int r = rand() % 1000000;
        for (int j=0; j<r; j++)
            34.2 * 26.7;
        cout << "r: " << r << endl;
        */

        ostringstream oss;
        oss << i << ".jpg";
        img->save(oss.str());
        delete img;
    }

    delete camera;
    return 0;
}
