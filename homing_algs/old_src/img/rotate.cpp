#include "ImgOps.h"
//#include "Imgdb.h"
//#include "ImgWindow.h"

int main( int argc, char *argv[] ) {
    Img img(argv[1]);

    Img* rotated = NULL;
    int width = img.getWidth();
    ImgOps::rotate(&img, width/2, rotated);
    rotated->save(argv[1]);

    //ImgWindow::waitAll();
    return 0;
}
