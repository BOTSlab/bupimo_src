#include "ImgOps.h"
#include "Imgdb.h"
#include "ImgWindow.h"

int main( int argc, char *argv[] ) {
    Imgdb imgdb("/data/db/preproc/original", 10, 17);
    Img* img = imgdb.getImg(5, 5);
    ImgWindow window("opsTest");
    window.addImg("original", *img);
    window.refresh();

    Img* smooth = NULL;
    Img* subt = NULL;
    Img* abs = NULL;

    //smooth = ImgOps::smooth(img, smooth, 11);
    ImgOps::smooth(img, 11, smooth);
    window.addImg("smooth", *smooth);

    //subt = ImgOps::sub(img, smooth, subt);
    ImgOps::sub(img, smooth, subt);
    window.addImg("subt", *subt);

    //abs = ImgOps::fabs(subt, abs);
    ImgOps::fabs(subt, abs);
    window.addImg("abs", *abs);

    Img::printNumberOfImages();

    window.interact();
    //ImgWindow::waitAll();

    delete smooth;
    delete subt;
    delete abs;
    return 0;
}
