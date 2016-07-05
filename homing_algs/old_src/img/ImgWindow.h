/*
 * Displays a single image in a window.  Utilizing the CImg library for the
 * actual display.  (Problems encountered in using the OpenCV highgui library
 * for this purpose.)
 *
 * Requires: CImg to be installed.
 *
 * Andrew Vardy
 */
#ifndef IMGWINDOW_H
#define IMGWINDOW_H

typedef float imageType;

#include "Img.h"
#include "CImg.h"
//#include "SiftExtractor.h"
#include <string>
#include <vector>
using namespace cimg_library;
using namespace std;

class ImgWindow {
public:
    ImgWindow( string inName, int inScale=1 );
    ~ImgWindow();
    void close();
    void clear();
    void setImg( Img &input );
    void addImg( Img &input );
    void addImg( string imageName, Img &input );

    /// Drawing methods.
    void drawCircle(int cx, int cy, int radius,
                    imageType red, imageType green, imageType blue);
    void drawText(int cx, int cy, const char *const text,
                  imageType red, imageType green, imageType blue);
    void drawLine(int x0, int y0, int x1, int y1, 
                   imageType red, imageType green, imageType blue);
    void drawArrow(int x0, int y0, int x1, int y1, 
                   imageType red, imageType green, imageType blue);
    void drawArrow(int x0, int y0, double angle, double magnitude,
                   imageType red, imageType green, imageType blue);
    void drawVerticalBar(int x, int radius,
                         imageType red, imageType green, imageType blue);

/*
    void drawKeypoints(vector<Keypoint*> &keys,
                       imageType red, imageType green, imageType blue,
                       int verticalShift=0);

    void addImagesWithMatches(Img &A, Img &B,
                              vector<Keypoint*> &aKeys,
                              vector<Keypoint*> &bKeys, vector<Match> &matches,
                              string title);
*/

    void refresh();

    /*
     * This is a blocking call that will allow the user to interact with
     * the ImgWindow, which will subsequently respond to certain inputs.  The
     * call to interact will not return until the user closes the window.
     */
    void interact();

    static void pauseWindow();

    // Wait for all displays to be closed.
    static void waitAll();
private:
    CImgDisplay display;
    string name;
    CImgList<imageType> list;
    int scale;

    int firstDisplayIndex, inputCounter;

    // Whether the window should be resized on the next call to refresh.
    bool doResize;

    // A static list of window displays.  To be used in 'wait' to determine if
    // all displays have been closed.
    static vector< CImgDisplay * > displays;
};

#endif
