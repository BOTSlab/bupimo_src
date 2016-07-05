#include "SimWindow.h"
#include "ImgWindow.h"

int main( int argc, char ** argv ) {
    SimWindow window("TEST", 0, 0, 10, 10, false);

    window.plotCircle(5, 5, 5);

    for ( int i=0; i<30; i++ )
        window.addPoint(1+8.0*(i/29.0), 9, i, 2.5);
    for ( int i=30; i<60; i++ )
        window.addPoint(1+8.0*((i-30)/29.0), 8, i, 0.05);
    for ( int i=60; i<90; i++ )
        window.addPoint(1+8.0*((i-60)/29.0), 7, i, 0.025);

    /*
    double angle = 0, mag=1.0, x=0, y=0, lastX=0, lastY=0;
    for ( int i=0; i<30; i++ ) {
        x = 5 + mag*cos(angle);
        y = 5 + mag*sin(angle);

        window.plotCircle(x, y, (30-i)/100.0);
        window.plotText(x, y, "Hey!");

        angle += 0.2;
        mag += 0.1;
        usleep(10000);
        lastX = x;
        lastY = y;
    }
    */
    window.plotEllipseFromTo(5, 1, 4, 9, 1);
    window.plotEllipseFromTo(5, 1, 4, 9, 0.5);
    window.plotEllipseFromTo(5, 1, 4, 9, 0.1);

//    ImgWindow::pauseWindow();
}
