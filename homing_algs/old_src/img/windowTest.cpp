#include "ImgWindow.h"

int main( int argc, char *argv[] ) {

    Img img(string("/home/av/work/pics/www/Old/bunsen.png"));
    ImgWindow window("windowTest");

    window.addImg("Bunsen Honeydew", img);
    window.refresh();

    ImgWindow::pauseWindow();

    Img img2(img);

    window.addImg("Evil Dr. Honeydew", img2);

    // Left eye
    window.drawCircle(130, 70, 10, 1, 0, 0);
    window.drawCircle(130, 70, 5, 0, 1, 0);

    // Right eye
    window.drawCircle(175, 70, 10, 1, 0, 0);
    window.drawCircle(175, 70, 5, 0, 1, 0);

    window.drawText(175, 40, "Ha ha!", 0, 0, 0);

    // Draw arrows of increasing length.
    int cx = 155;
    int cy = 124;
    for (double theta = M_PI; theta > 0; theta -= 0.2)
        window.drawArrow(cx, cy, theta, theta*40,
                         1.0 - theta/M_PI, 1.0 - theta/M_PI, theta/M_PI);

    window.interact();
    return 0;
}
