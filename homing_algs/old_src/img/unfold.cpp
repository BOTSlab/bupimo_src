#include "Unfolder.h"
#include "ImgWindow.h"
#include "ImgOps.h"

int main( int argc, char *argv[] ) {

    if (argc != 3)
        cout << "usage:\n\tunfold INPUT_FILE OUTPUT_FILE\n";

    Img input(argv[1]);

    Unfolder unfolder;
    Img *output = NULL;
    Img *debug = unfolder.unfold(input, output, true);

//    // Now rotate the image by 90 degrees.
//    int width = output->getWidth();
//    Img* rotated = ImgOps::rotate(output, width/4, NULL);

    // Write output image to file.
    output->save(string(argv[2]));

    // Write debug image to file.
    debug->save("debug.jpg");
 
    // Display the debug image.
//    ImgWindow window("Unfolder debug image");
//    window.setImg(*debug);
//    window.refresh();

    ImgWindow::waitAll();
    return 0;
}
