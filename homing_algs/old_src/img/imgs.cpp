/**
 * Display all images passed to the command line.  The first argument should be
 * the desired scale of the images.
 *
 * usage:
 *
 *      imgs 4 one.pgm two.png
 *
 * @author Andrew Vardy
 */

#include "ImgWindow.h"
#include <sstream>
using namespace std;

int main( int argc, char *argv[] ) {

    if (argc < 3) {
        cout << "usage:\n\timgs SCALE FILE1 FILE2 ..." << endl;
        return -1;
    }

    int scale;
    istringstream iss(argv[1]);
    iss >> scale;

    ImgWindow window("imgs", scale);
    for (int i=2; i<argc; i++) {
        string str(argv[i]);
        cout << "imgs: " << str << endl;
        Img img(str);
        //window.addImg(str, img);
        window.addImg(img);
    }
    window.refresh();

    ImgWindow::waitAll();
    return 0;
}
