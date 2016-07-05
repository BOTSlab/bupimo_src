#include "ImgWindow.h"
#include "ImgOps.h"
#include "ImgSequence.h"
#include "SiftExtractor.h"

void drawKeys(string name, vector<Keypoint*> &keys, Img &src,
              ImgWindow &window) {
    // Create a copy of the input image and draw dots at the locations of
    // the keypoints.
    Img* drawImg = new Img(src);
    drawImg->normalize();

    window.addImg(name, *drawImg);

    int radius = 2;
    for (int i=0; i<keys.size(); i++) {
        int cx = (int) keys[i]->x;
        int cy = (int) keys[i]->y;
        window.drawCircle(cx, cy, radius+1, 0, 0, 0);
        window.drawCircle(cx, cy, radius, 1, 1, 1);
    }

    delete drawImg;
}

void drawMatches(vector<Keypoint*> &keysA, vector<Keypoint*> &keysB, vector<Match> &matches, Img &imgA, Img &imgB, ImgWindow &window) {

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
    ImgSequence seq(string("/data/seq/bruneau"));

    Img *S = seq.getImg(0);
    Img Mask(string("/data/seq/bruneau/mask.png"));

    ImgWindow window("bruneau", 1);

    // Extract keypoints from S
    int width = S->getWidth();
    int height = S->getHeight();
    SiftExtractor extractor(width, height);
    vector<Keypoint*> rawKeysS;
    extractor.extract(*S, rawKeysS);

    // Keep only keypoints that are in the white area of the mask image.
    vector<Keypoint*> keysS;
    for (int k=0; k<rawKeysS.size(); k++) {
        Keypoint *key = rawKeysS[k];
        if (Mask.get((int)key->x, (int)key->y) > 0)
            keysS.push_back(key);
    }


    cout << "keypoints from S: " << keysS.size() << endl;

    // Draw keypoints
    drawKeys("", keysS, *S, window);
    cout << "Drawing done." << endl;

    double sigmas[keysS.size()][seq.getLength()];
    for (int s=0; s<keysS.size(); s++) {
        sigmas[s][0] = keysS[s]->sigma;
        for (int c=1; c<seq.getLength(); c++)
            sigmas[s][c] = 0;
    }

    for (int c = 0; c<seq.getLength(); c++) {
//for (int c = 0; c<seq.getLength(); c+=3) {
//cout << "c: " << c << endl;
        Img *C = seq.getImg(c);
        vector<Keypoint*> keysC;
        extractor.extract(*C, keysC);

        vector<Match> matches;
        extractor.match(keysS, keysC, matches);

        // For each matched key, determine the match from S and fill in an
        // entry in the array 'sigma'
        vector<Keypoint*> matchedKeys;
        for (int m=0; m<matches.size(); m++) {
            Keypoint *key = keysC[matches[m].b];
            if (Mask.get((int)key->x, (int)key->y) > 0) {
                matchedKeys.push_back(key);
                sigmas[matches[m].a][c] = key->sigma;
            }
        }

/*
        for (int k=0; k<matchedKeys.size(); k++) {
            cout << "\tx: " << matchedKeys[k]->x << endl;
            cout << "\ty: " << matchedKeys[k]->y << endl;
        }
cout << "\tnumber of matched keys: " << matchedKeys.size() << endl;
*/

        drawKeys("", matchedKeys, *C, window);
    
    }

    // Print out the sigma values
    for (int s=0; s<keysS.size(); s++) {
        for (int c=0; c<seq.getLength(); c++) {
            cout << sigmas[s][c] << "\t";
        }
        cout << endl;
    }

    window.interact();

    ImgWindow::waitAll();
    return 0;
}
