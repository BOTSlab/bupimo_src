#include "Imgdb.h"
#include "ImgOps.h"
#include "ImgWindow.h"
#include <cv.h>

int main( int argc, char *argv[] ) {
    Imgdb imgdb("/data/db/preproc/original", 10, 17);
    Img* S = imgdb.getImg(5, 4);
    Img* C = imgdb.getImg(5, 6);
    int w = S->getWidth();
    int h = S->getHeight();

    ImgWindow window("opticFlow");
    window.addImg("S", *S);
    window.addImg("C", *C);

    Img *U = NULL;
    Img *V = NULL;
    ImgOps::blockMatch(*S, *C, U, V);

    U->saveGrd("u.grd");
    V->saveGrd("v.grd");

    window.addImg("U", *U);
    window.addImg("V", *V);

    // Draw motion vectors
    window.addImg("Vectors", *C);
    int step = 5;
    int outW = U->getWidth();
    int outH = U->getHeight();
    for (int i=0; i<outW; i+= step) {
        for (int j=0; j<outH; j+= step) {
            int u = U->get(i, j);
            int v = V->get(i, j);
            if (u != 0 || v != 0)
                window.drawArrow(i, j, i + (int)u, j + (int)v, 1, 1, 1);
        }
    }
    
    // Draw direction of motion encoded as grayscale
    Img Dirs(outW, outH);
    for (int i=0; i<outW; i++) {
        for (int j=0; j<outH; j++) {
            double u = U->get(i, j);
            double v = V->get(i, j);
            if (u != 0 || v != 0) {
                double theta = atan2(v, u);
                Dirs.set(i, j, theta + M_PI);
            }
        }
    }
    window.addImg("Dirs", Dirs);

    window.interact();
    return 0;
}
