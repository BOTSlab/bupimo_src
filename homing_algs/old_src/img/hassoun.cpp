#include "ImgWindow.h"
#include <cmath>

// Apply Hassoun & Sanghvi's Distance Transform

void hassoun(Img &obstacle) {
    // Display distance map image.
    ImgWindow outputWindow("Output");

    int w = obstacle.getWidth();
    int h = obstacle.getHeight();

    // Initialize current distance map image.
    Img *current = new Img(w, h);

    // Obstacles should be given a value of 0.  All other pixels should be
    // given an arbitrarily large value.
    int vmax = w;
    current->setAll(0); // Just sets boundary to 0.
    for (int i=0; i<w; i++)
        for (int j=0; j<h; j++)
            if (obstacle.get(i, j) == 1)
                current->set(i, j, 0);
            else
                current->set(i, j, vmax);

    // Initialize an image to store the next distance map.
    Img *next = new Img(*current);

    // Somewhat arbitrary starting value.
    int n = 200;
    for (int i=0; i<n; i++) {
        for (int i=1; i<w-1; i++)
            for (int j=1; j<h-1; j++) {
                float val = vmax;
                for (int di=-1; di<=1; di++)
                    for (int dj=-1; dj<=1; dj++) {
                        float neighbourVal = current->get(i + di, j + dj);
                        float distance = sqrt(di*di + dj*dj);
                        float total = neighbourVal + distance;
                        if (total < val)
                            val = total;
                    }
                next->set(i, j, val);
            }

        // Swap the current and next image pointers.
        Img *tmp = next;
        next = current;
        current = tmp;
/*
ImgWindow::pauseWindow();
outputWindow.clear();
outputWindow.addImg("Distance Image", *current);
outputWindow.refresh();
*/
    }

    current->save("hassoun.bmp");
            
outputWindow.clear();
outputWindow.addImg("Distance Image", *current);
outputWindow.refresh();
ImgWindow::waitAll();

    delete current;
    delete next;
}

int main( int argc, char *argv[] ) {

    // Load and display input image.
    Img input(argv[1]);
    ImgWindow inputWindow(argv[1]);
    inputWindow.setImg(input);

    // Create obstacle image
    Img obstacle(input);

    hassoun(obstacle);

    return 0;
}
