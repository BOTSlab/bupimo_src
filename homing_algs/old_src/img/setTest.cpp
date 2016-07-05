#include "ImgWindow.h"
#include "ImgSet.h"
#include <ncurses.h>
using namespace std;

int main( int argc, char *argv[] ) {

    ImgWindow window("setTest");
    ImgSet set(string("/data/db/gallery2_occ"), string("360_50"));
//    ImgSet set(string("/data/db/gallery2/point0"), string("720_100"));

    // ncurses initialization
    initscr();
//    clear();
    noecho();
    cbreak();
    timeout(0);
//    WINDOW *cWindow = newwin(24, 80, 0, 0);
    keypad(stdscr, TRUE);

    // Enter into a loop where we can move around the image set.
    double x=0, y=0, theta=0, delta=1, deltaTheta=0.1, lastX, lastY, lastTheta;
    bool done = false;
    Img *img = NULL;
    while (!done) {
        lastX = x;
        lastY = y;
        lastTheta = theta;
        int ch = wgetch(stdscr);
        if (ch == KEY_LEFT)
            theta += deltaTheta;
        else if (ch == KEY_RIGHT)
            theta -= deltaTheta;
        else if (ch == KEY_DOWN) {
            x += delta*cos(theta);
            y += delta*sin(theta);
        } else if (ch == KEY_UP) {
            x -= delta*cos(theta);
            y -= delta*sin(theta);
        }
        else if (ch == 'q')
            done = true;
        bool gotImg = set.getImg(x, y, theta, img, 0.999);
        if (!gotImg) {
            // There was no image sufficiently close to (x, y).  It is off-map.
            x = lastX;
            y = lastY;
            theta = lastTheta;
        } else
            window.setImg(*img);

        wclear(stdscr);
        wprintw(stdscr, "x, y: %f, %f\n", x, y);
        //wprintw(stdscr, 
        //        "Use the arrow keys to move the camera.  Hit 'q' to quit..\n");
        wrefresh(stdscr);
    }

    endwin();
    exit(0);
    return 0;
}
