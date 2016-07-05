#include "ImgWindow.h"
#include <algorithm>

vector< CImgDisplay * > ImgWindow::displays;

ImgWindow::ImgWindow( string inName, int inScale ) :
    display(),
    name(inName),
    list(),
    scale(inScale),
    firstDisplayIndex(0),
    inputCounter(0),
    doResize(true)
{
    displays.push_back(&display);
}

ImgWindow::~ImgWindow() {
    close();
}

//
// May be called twice.  Once if called through 'interact'.  The second time
// if the ImgWindow is destroyed.
//
void ImgWindow::close() {
    clear();
    if (!display.is_closed())
        display.close();

    // Remove this window's display from our global list of displays
    vector< CImgDisplay *>::iterator it;
    it = find(displays.begin(), displays.end(), &display);
    if (it != displays.end())
        displays.erase(it);
}

void ImgWindow::clear() {
    list.clear();
    doResize = true;
}

void ImgWindow::setImg( Img &input ) {
    list.clear();
    addImg("", input);
    refresh();
}

void ImgWindow::addImg( Img &inputImage ) {
    addImg("", inputImage);
}

void ImgWindow::addImg( string imageName, Img &inputImage ) {
    // Normalize a copy of the input image.  CImgDisplay will normalize all
    // images in the list to the same scale prior to display.  If we normalize
    // now then at least the full contrast of all images will be visible.
    Img input(inputImage);
    input.normalize();

    int w = input.getWidth();
    int h = input.getHeight();
    int n = w*h;
    int twoN = 2*n;
    imageType *inputData = new imageType[3*n];
    for ( int j=0; j<h; j++ ) {
        for ( int i=0; i<w; i++ ) {
            int x = j*w + i;
            imageType value = input.get(i, j);
            inputData[x] = value;         // Red channel
            inputData[n + x] = value;   // Green channel
            inputData[twoN + x] = value; // Blue channel
        }
    }

    CImg<imageType> image(inputData, w, h, 1, 3); // Colour image.
    imageType bgColorArray[3] = {0, 0, 0};
    imageType fgColorArray[3] = {1, 1, 1};
    image.draw_text(2, 5, imageName.c_str(), fgColorArray, bgColorArray);
    list.push_back(image);

    doResize = true;
    delete [] inputData;
}

void ImgWindow::drawCircle(int cx, int cy, int radius,
                           imageType red, imageType green, imageType blue) {
    imageType colorArray[3] = {red, green, blue};
    list.back().draw_circle(cx, cy, radius, colorArray);
}

void ImgWindow::drawText(int cx, int cy, const char *const text,
                         imageType red, imageType green, imageType blue) {
    imageType fgArray[3] = {red, green, blue};
    list.back().draw_text(cx, cy, text, fgArray);
}

void ImgWindow::drawLine(int x0, int y0, int x1, int y1, 
                         imageType red, imageType green, imageType blue) {
    imageType fgArray[3] = {red, green, blue};
    list.back().draw_line(x0, y0, x1, y1, fgArray);
}

void ImgWindow::drawArrow(int x0, int y0, int x1, int y1, 
                         imageType red, imageType green, imageType blue) {
    imageType fgArray[3] = {red, green, blue};
    list.back().draw_arrow(x0, y0, x1, y1, fgArray);
}

void ImgWindow::drawArrow(int x0, int y0, double angle, double magnitude,
                          imageType red, imageType green, imageType blue) {
    imageType fgArray[3] = {red, green, blue};
    int x1 = x0 + (int)(magnitude * cos(angle));
    int y1 = y0 + (int)(magnitude * sin(angle));
    drawArrow(x0, y0, x1, y1, red, green, blue);
}

void ImgWindow::drawVerticalBar(int x, int radius, imageType red,
                                imageType green, imageType blue) {
    imageType fgArray[3] = {red, green, blue};
    int height = list.back().height();
    for (int i=-radius; i<=radius; i++)
        list.back().draw_line(x, 0, x, height-1, fgArray);
}

/*
void ImgWindow::drawKeypoints(vector<Keypoint*> &keys,
                              imageType red, imageType green, imageType blue,
                              int verticalShift) {

    int radius = 1;
    for (int i=0; i<keys.size(); i++) {
        int cx = (int) keys[i]->x;
        int cy = (int) keys[i]->y + verticalShift;
        drawCircle(cx, cy, radius, red, green, blue);
    }
}

void ImgWindow::addImagesWithMatches(Img &A, Img &B,
                                     vector<Keypoint*> &aKeys,
                                     vector<Keypoint*> &bKeys,
                                     vector<Match> &matches,
                                     string title) {
    int width = A.getWidth();
    int height = A.getHeight();
    assert(B.getWidth() == width && B.getHeight() == height);

    const int GAP = 5;

    // Stack the two images so that we can draw the match vectors.
    Img Big(width, 2*height + GAP);
    Big.setAll(1);
    for (int i=0; i<width; i++) {
        for (int j=0; j<height; j++) {
            Big.set(i, j, A.get(i, j));
            Big.set(i, height+GAP+j, B.get(i, j));
        }
    }

    addImg(title, Big);

    // Draw the matches
    int n = matches.size();
    for (int m=0; m<n; m++) {
        Keypoint *aKey = aKeys[matches[m].a];
        Keypoint *bKey = bKeys[matches[m].b];
        int x0 = aKey->x;
        int y0 = aKey->y;
        int x1 = bKey->x;
        int y1 = bKey->y + height + GAP;

        if (bKey->sigma < aKey->sigma)
            // Feature has contracted, draw in green.
            drawLine(x0, y0, x1, y1, 0, 1, 0);
        else if (bKey->sigma > aKey->sigma)
            // Feature has expanded, draw in red.
            drawLine(x0, y0, x1, y1, 1, 0, 0);
        else
            // Feature has the same scale, draw in white.
            drawLine(x0, y0, x1, y1, 1, 1, 1);
    }
}
*/

void ImgWindow::refresh() {
    // The width and height of the overall image to be displayed.
    int w = 0;
    int h = 0;

    CImgList<imageType> displayList;
    for (int i=firstDisplayIndex; i < list.size(); i++)
    {
        CImg<imageType> displayImg = list[i];

        displayImg.resize(displayImg.width() * scale,
                          displayImg.height() * scale);

        if (h + displayImg.height() > CImgDisplay::screen_height() - 50)
            break;

        displayList.push_back(displayImg);

        w = max(w, displayImg.width());
        h += displayImg.height();
    }

    if (doResize) {
        display.resize(w, h, true);
        doResize = false;
    }

    display.set_title(name.c_str());
    display.display(displayList, 'y');
    display.show();
}

void ImgWindow::interact() {

    refresh();

    while (true) {
        display.wait();

        if (display.is_closed()) {
            close();
            return;
        }

        if (inputCounter > 0) {
            inputCounter--;
            continue;
        }

        bool changeMade = false;
        if (display.is_keyARROWUP()) {
            if (firstDisplayIndex > 0) {
                firstDisplayIndex--;
                changeMade = true;
            }
        } else if (display.is_keyARROWDOWN()) {
            if (firstDisplayIndex < list.size() - 1) {
                firstDisplayIndex++;
                changeMade = true;
            }
        } else if (display.is_keyARROWRIGHT()) {
            if (scale < 3) {
                scale++;
                changeMade = true;
            }
        } else if (display.is_keyARROWLEFT()) {
            if (scale > 1) {
                scale--;
                changeMade = true;
            }
        }

        if (changeMade) {
            doResize = true;
            inputCounter = 1;
            refresh();
        }
    }
}

void ImgWindow::pauseWindow() {
    ImgWindow window("Close to continue");    
    Img blank(300, 40);
    blank.setAll(0);
    window.setImg(blank);
    window.interact();
}

void ImgWindow::waitAll() {
    // First, are there any displays to wait for?
    if (displays.size() == 0)
        return;

    cout << "Waiting for all image windows to be closed... " << endl;
    bool allDisplaysClosed = false;
    while (!allDisplaysClosed) {
        CImgDisplay::wait_all();
        allDisplaysClosed = true;
        for (int i=0; i<displays.size(); i++) {
            if (!displays[i]->is_closed())
                allDisplaysClosed = false;
        }
    }
    cout << "Done." << endl;
}
