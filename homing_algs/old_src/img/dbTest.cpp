#include "ImgWindow.h"
#include "Imgdb.h"

int main( int argc, char *argv[] ) {
    Imgdb db("/home/av/work/data/db/bubblescope_first/unfolded", 3, 3);
    ImgWindow window("dbTest");

    for ( int j=0; j<db.getHeight(); j++ )
        for ( int i=0; i<db.getWidth(); i++ ) {
            Img *img = db.getImg(i, j);
            window.setImg(*img);
        }

    Img::printNumberOfImages();

    ImgWindow::waitAll();
    return 0;
}
