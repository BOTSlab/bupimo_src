#include "ImgWindow.h"
#include "ImgOps.h"
#include "SiftExtractor.h"

void drawKeys(string name, vector<Keypoint*> &keys, Img &src,
              ImgWindow &window) {
    // Create a copy of the input image and draw dots at the locations of
    // the keypoints.
    Img* drawImg = new Img(src);
    window.addImg(name, *drawImg);

    window.drawKeypoints(keys, 0, 0, 1);

    delete drawImg;
}

int main( int argc, char *argv[] ) {

    if (argc < 3 || ((argc+1) % 2) != 0 )  // Should be an odd number >= 3
    {
        cout << "usage:\n\tdrawKeys IMAGE_1 IMAGE_2 KEYS_1 KEYS_2 ...\n";
        cout << "argc: " << argc << endl;
        return 0;
    }

    ImgWindow window("");
    int n = (argc - 1) / 2;
    for (int i=0; i<n; i++) {

        string imgFilename(argv[i+1]);
        Img img(imgFilename);

        string keyFilename(argv[i+1+n]);
        vector<Keypoint*> keys;
        SiftExtractor::loadKeys(keyFilename, keys);

        drawKeys(keyFilename, keys, img, window);

        SiftExtractor::clearKeys(keys);
    }

    window.interact();
    return 0;
}
