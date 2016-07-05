#include "ImgWindow.h"
#include "ImgOps.h"
#include "SiftExtractor.h"

void drawKeys(vector<Keypoint*> &keys, Img &src, ImgWindow &window) {
    // Create a copy of the input image and draw dots at the locations of
    // the keypoints.
    Img* drawImg = new Img(src);
    int width = src.getWidth();
    int height = src.getHeight();
    int radius = 1;
    for (int i=0; i<keys.size(); i++) {
        int cx = (int) keys[i]->x;
        int cy = (int) keys[i]->y;
        for (int x=cx-radius; x<=cx+radius; x++)
            for (int y=cy-radius; y<=cy+radius; y++)
                if (x >= 0 && x < width && y >= 0 && y < height)
                    drawImg->set(x, y, 1);
    }
    window.addImg("", *drawImg);

    delete drawImg;
}

void drawMatches(vector<Keypoint*> &keysA, vector<Keypoint*> &keysB,
                 vector<Match> &matches,
                 Img &imgA, Img &imgB, ImgWindow &window) {

    // First build a big image equivalent to imgA stacked on top of imgB.
    int width = imgA.getWidth();
    int height = imgA.getHeight();
    Img bigImg(width, height*2);
    for (int y=0; y<height; y++)
        for (int x=0; x<width; x++)
            bigImg.set(x, y, imgA.get(x, y));
    for (int y=height; y<2*height; y++)
        for (int x=0; x<width; x++)
            bigImg.set(x, y, imgB.get(x, y-height));

    for (int m=0; m<matches.size(); m++) {
        int a = matches[m].a;
        int b = matches[m].b;

        Keypoint *keyA = keysA[ a ];
        Keypoint *keyB = keysB[ b ];

        int x1 = (int) keyA->x;
        int y1 = (int) keyA->y;
        int x2 = (int) keyB->x;
        int y2 = (int) keyB->y + height;

        ImgOps::drawLine(&bigImg, x1, y1, x2, y2);
    }

    window.addImg("", bigImg);
    window.refresh();
}


int main( int argc, char *argv[] ) {
    Img *imgA = new Img(string("/home/av/work/data/db/bubblescope_first/unfolded/0_0.pgm"));
    //Img *imgB = new Img(string("/home/av/work/data/db/bubblescope_first/unfolded/0_1.pgm"));
    Img *rawB = new Img(string("/home/av/work/data/db/bubblescope_first/unfolded/0_1.pgm"));
    Img *imgB = NULL;
    ImgOps::rotate(rawB, -150, imgB);
    
//    Img imgA(string("/data/foxtrapScreenshots/right1_despeckled.bmp"));
//    Img imgB(string("/data/foxtrapScreenshots/right2_despeckled.bmp"));

    ImgWindow window("Input image");
    window.addImg("Image A", *imgA);
    window.addImg("Image B", *imgB);
    window.refresh();

    // Extract keypoints from both images
    int width = imgA->getWidth();
    int height = imgA->getHeight();
    SiftExtractor extractor(width, height);
    vector<Keypoint*> keysA, keysB;
    extractor.extract(*imgA, keysA);
    extractor.extract(*imgB, keysB);
    cout << "keypoints from image A: " << keysA.size() << endl;
    cout << "keypoints from image B: " << keysB.size() << endl;

    // Draw keypoints
    drawKeys(keysA, *imgA, window);
    drawKeys(keysB, *imgB, window);
    window.refresh();
    cout << "Drawing done." << endl;

    /*
    // Save the keypoints and then re-load them to test saving and loading.
    SiftExtractor::saveKeys(keysA, "keysA.keys");
    SiftExtractor::saveKeys(keysB, "keysB.keys");
    cout << "Saving done." << endl;
    SiftExtractor::clearKeys(keysA);
    SiftExtractor::clearKeys(keysB);
    SiftExtractor::loadKeys("keysA.keys", keysA);
    SiftExtractor::loadKeys("keysB.keys", keysB);
    cout << "Loading done." << endl;
    cout << "keypoints from image A: " << keysA.size() << endl;
    cout << "keypoints from image B: " << keysB.size() << endl;

    drawKeys(keysA, *imgA, window);
    drawKeys(keysB, *imgB, window);
    window.refresh();
    cout << "Drawing done." << endl;
    */

    // Compute matches
    vector<Match> matches;
    extractor.match(keysA, keysB, matches);
    cout << "matches: " << matches.size() << endl;

    drawMatches(keysA, keysB, matches, *imgA, *imgB, window);

    extractor.deviationFilter(keysA, keysB, matches, 1.0);
    cout << "filtered matches: " << matches.size() << endl;

    drawMatches(keysA, keysB, matches, *imgA, *imgB, window);
    window.refresh();

    window.interact();

    // Free keypoints
    SiftExtractor::clearKeys(keysA);
    SiftExtractor::clearKeys(keysB);

    delete imgA;
    delete rawB;
    delete imgB;
    return 0;
}
