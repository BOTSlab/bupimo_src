#include "ImgWindow.h"
#include <cmath>

void extractGVD(Img &distanceMap, Img &gvd) {
    int w = distanceMap.getWidth();
    int h = distanceMap.getHeight();

    for (int i=1; i<w-1; i++)
        for (int j=1; j<h-1; j++) {
            float val = distanceMap.get(i, j);
            int numbHigher = 0;
            for (int di=-1; di<=1; di++)
                for (int dj=-1; dj<=1; dj++)
                    if (distanceMap.get(i + di, j + dj) >= val)
                        numbHigher++;

            if (numbHigher <= 4)
                gvd.set(i, j, 1);            
        }
}

int main( int argc, char *argv[] ) {

    // Load and display input distance map image.
    Img distanceMap(string("hassoun.bmp"));

    int w = distanceMap.getWidth();
    int h = distanceMap.getHeight();
    Img gvd(w, h);
    gvd.setAll(0);

    extractGVD(distanceMap, gvd);

    // Now load the original map (which the distance map was created from) and
    // overlay it on the gvd.
    Img originalMap(string("map.bmp"));

    for (int i=0; i<w; i++)
        for (int j=0; j<h; j++)
            if (originalMap.get(i, j) == 1)
                gvd.set(i, j, 1);
    
    gvd.save("gvd.bmp");

// Display distance and gvd images.
ImgWindow outputWindow("Output");
outputWindow.clear();
outputWindow.addImg("Distance Image", distanceMap);
outputWindow.addImg("GVD", gvd);
outputWindow.refresh();
ImgWindow::waitAll();

    return 0;
}
