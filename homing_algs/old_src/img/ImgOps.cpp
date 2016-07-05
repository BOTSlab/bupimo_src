#include "ImgOps.h"
#include <cmath>
#include <cstdlib>

void ImgOps::rotate(Img* A, int shift, Img*& B) {
    int w = A->getWidth();
    int h = A->getHeight();
    setAndCheck(A, B);

    for ( int x=0; x<w; x++ )
        for ( int y=0; y<h; y++ )
            B->set(x, y, A->getToroidal(x-shift, y));
/*
    // Copy the keypoints from A into B and then rotate them.
    B->deepCopyKeypoints(A->keypoints);
    int nKeys = (int) B->keypoints.size();
    for (int i=0; i<nKeys; i++)
        B->keypoints[i]->x = Img::wrap(B->keypoints[i]->x + shift, w);
*/
}

void ImgOps::diffHorz(Img* A, Img*& B) {
    setAndCheck(A, B);
    cvSobel( A->image, B->image, 1, 0, 1);
}

void ImgOps::diffVert(Img* A, Img*& B) {
    setAndCheck(A, B);
    cvSobel( A->image, B->image, 0, 1, 1);
}

void ImgOps::smooth(Img* A, int radius, Img*& B) {
    setAndCheck(A, B);
    cvSmooth( A->image, B->image, CV_GAUSSIAN, 2*radius + 1 );
}

void ImgOps::add(Img* A, float scalar, Img*& B) {
    setAndCheck(A, B);
    int w = A->getWidth();
    int h = A->getHeight();
    for ( int y=0; y<h; y++ )
        for ( int x=0; x<w; x++ )
            B->set(x, y, A->get(x,y) + scalar);
}

void ImgOps::mult(Img* A, float scalar, Img*& B) {
    setAndCheck(A, B);
    int w = A->getWidth();
    int h = A->getHeight();
    for ( int y=0; y<h; y++ )
        for ( int x=0; x<w; x++ )
            B->set(x, y, A->get(x,y) * scalar);
}

void ImgOps::add(Img* A, Img* B, Img*& C) {
    comb(1, A, 1, B, true, C);
}
void ImgOps::sub(Img* A, Img* B, Img*& C) {
    comb(1, A, -1, B, true, C);
}
void ImgOps::mult(Img* A, Img* B, Img*& C) {
    comb(1, A, 1, B, false, C);
}

void ImgOps::div(Img* A, Img* B, Img*& C) {
// Precondition: A and B (and C if its non-NULL) have the same dimensions.
    int w = A->getWidth();
    int h = A->getHeight();
    if ( w != B->getWidth() || h != B->getHeight() || 
         (C != NULL && (w != C->getWidth() || h != C->getHeight()) ) ) 
    {
        cout << "ImgOps: Images mismatched!" << endl;
        exit(-1);
    }
    if ( C == NULL )
        C = new Img(w, h);

    for ( int x=0; x<w; x++ )
        for ( int y=0; y<h; y++) {
            float a = A->get(x, y);
            float b = B->get(x, y);
            if ( std::fabs(b) > 0.0001 )
                C->set(x, y, a/b);
        }
}

void ImgOps::comb( float ca, Img* A, float cb, Img* B, bool plus, Img*& C) {
// Precondition: A and B (and C if its non-NULL) have the same dimensions.
    int w = A->getWidth();
    int h = A->getHeight();
    if ( w != B->getWidth() || h != B->getHeight() || 
         (C != NULL && (w != C->getWidth() || h != C->getHeight()) ) ) 
    {
        cout << "ImgOps: Images mismatched!" << endl;
        exit(-1);
    }
    if ( C == NULL )
        C = new Img(w, h);

    for ( int x=0; x<w; x++ )
        for ( int y=0; y<h; y++) {
            float a = A->get(x, y);
            float b = B->get(x, y);
            if ( plus )
                C->set(x, y, ca*a + cb*b);
            else
                C->set(x, y, ca*a * cb*b);
        }
}

void ImgOps::logicAnd(Img* A, Img* B, Img*& C) {
    logicComb(A, B, true, C);
}

void ImgOps::logicOr(Img* A, Img* B, Img*& C) {
    logicComb(A, B, false, C);
}

void ImgOps::logicComb(Img* A, Img* B, bool andOp, Img*& C) {
// Precondition: A and B (and C if its non-NULL) have the same dimensions.
    int w = A->getWidth();
    int h = A->getHeight();
    if ( w != B->getWidth() || h != B->getHeight() || 
         (C != NULL && (w != C->getWidth() || h != C->getHeight()) ) ) 
    {
        cout << "ImgOps: Images mismatched!" << endl;
        exit(-1);
    }
    if ( C == NULL )
        C = new Img(w, h);

    for ( int x=0; x<w; x++ )
        for ( int y=0; y<h; y++) {
            float a = A->get(x, y);
            float b = B->get(x, y);
            if ( andOp )
                C->set(x, y, (int)a && (int)b);
            else
                C->set(x, y, (int)a || (int)b);
        }
}


float ImgOps::ssd(Img* A, Img*& B) {
// Precondition: A and B have the same dimensions.
    int w = A->getWidth();
    int h = A->getHeight();
    if ( w != B->getWidth() || h != B->getHeight()) {
        cout << "ImgOps: Images mismatched!" << endl;
        exit(-1);
    }

    float ssd = 0;
    for ( int x=0; x<w; x++ )
        for ( int y=0; y<h; y++) {
            float diff = A->get(x, y) - B->get(x, y);
            ssd += diff*diff;
        }

    return ssd;
}

void ImgOps::sqdDiff(Img* A, Img* B, Img*& C) {
// Precondition: A and B (and C if its non-NULL) have the same dimensions.
    int w = A->getWidth();
    int h = A->getHeight();
    if ( w != B->getWidth() || h != B->getHeight() || 
         (C != NULL && (w != C->getWidth() || h != C->getHeight()) ) ) 
    {
        cout << "ImgOps: Images mismatched!" << endl;
        exit(-1);
    }
    if ( C == NULL )
        C = new Img(w, h);

    for ( int x=0; x<w; x++ )
        for ( int y=0; y<h; y++) {
            float diff = A->get(x, y) - B->get(x, y);
            C->set(x, y, diff*diff);
        }
}


void ImgOps::eqdWarp(Img* In, float alpha, float k, bool taylor) {
    int w = In->getWidth();
    int h = In->getHeight();
    Img* Out = new Img(w, h);
    Out->setAll(0);

    // Compute image gradient (stored as two images)
    Img* GradX = NULL;
    Img* GradY = NULL;
    if (taylor) {
        ImgOps::diffHorz(In, GradX);
        ImgOps::diffVert(In, GradY);
    }

    float delta = 2*M_PI / w;
    float gamma0 = delta * (h-1)/2;
    for ( int i=0; i<w; i++ )
        for ( int j=0; j<h; j++) {
            float beta = 2*M_PI - delta*i;
            float gamma = gamma0 - delta*j;

            float vx = (float) 
                       ((std::cos(alpha)*std::sin(beta) -
                        std::sin(alpha)*std::cos(beta))/std::cos(gamma));
            float vy = (float)
                       ((std::cos(alpha)*std::cos(beta) + 
                        std::sin(alpha)*std::sin(beta))*std::sin(gamma));

            if ( taylor) {
                // Use the Taylor series expansion
                float in = In->get(i, j);
                float gx = GradX->get(i, j);
                float gy = GradY->get(i, j);
                Out->set(i, j, in - k*(gx*vx + gy*vy) );
            } else {
                int x = (int) (i - k*vx + 0.5);
                int y = (int) (j - k*vy + 0.5);
                if ( y > 0 && y < h ) 
                    Out->set(i, j, In->getToroidal(x, y));
            }
        }
}

void ImgOps::cos(Img* A, Img*& B) {
    setAndCheck(A, B);
    int w = A->getWidth();
    int h = A->getHeight();
    for ( int y=0; y<h; y++ )
        for ( int x=0; x<w; x++ )
            B->set(x, y, std::cos(A->get(x, y) ));
}

void ImgOps::sin(Img* A, Img*& B) {
    setAndCheck(A, B);
    int w = A->getWidth();
    int h = A->getHeight();
    for ( int y=0; y<h; y++ )
        for ( int x=0; x<w; x++ )
            B->set(x, y, std::sin(A->get(x, y) ));
}

void ImgOps::tan(Img* A, Img*& B) {
    setAndCheck(A, B);
    int w = A->getWidth();
    int h = A->getHeight();
    for ( int y=0; y<h; y++ )
        for ( int x=0; x<w; x++ )
            B->set(x, y, std::tan(A->get(x, y) ));
}

void ImgOps::atan2(Img* A, Img* B, Img *&C) {
// Precondition: A and B (and C if its non-NULL) have the same dimensions.
    int w = A->getWidth();
    int h = A->getHeight();
    if ( w != B->getWidth() || h != B->getHeight() || 
         (C != NULL && (w != C->getWidth() || h != C->getHeight()) ) ) 
    {
        cout << "ImgOps: Images mismatched!" << endl;
        exit(-1);
    }
    if ( C == NULL )
        C = new Img(w, h);

    for ( int x=0; x<w; x++ )
        for ( int y=0; y<h; y++) {
            float a = A->get(x, y);
            float b = B->get(x, y);
            C->set(x, y, std::atan2(a,b));
        }
}

void ImgOps::fabs(Img* A, Img*& B) {
    setAndCheck(A, B);
    int w = A->getWidth();
    int h = A->getHeight();
    for ( int y=0; y<h; y++ )
        for ( int x=0; x<w; x++ )
            B->set(x, y, std::fabs(A->get(x, y)));
}

void ImgOps::threshold(Img* In, float threshold, Img *&Out) {
    setAndCheck(In, Out);
    int w = In->getWidth();
    int h = In->getHeight();

    for ( int y=0; y<h; y++ )
        for ( int x=0; x<w; x++) {
            if (In->get(x, y) <= threshold )
                Out->set(x, y, 0);
            else
                Out->set(x, y, 1);
        }
}

void ImgOps::negate(Img* In, Img *&Out) {
    setAndCheck(In, Out);
    int w = In->getWidth();
    int h = In->getHeight();

    float max = In->getMax();
    for ( int y=0; y<h; y++ )
        for ( int x=0; x<w; x++ )
            Out->set(x, y, max - In->get(x,y));
}

void ImgOps::histThreshold(Img* In, int bins, float fraction, Img *&Out) {
// Use a histogram to determine the threshold that separates the lower
// 'fraction' proportion of pixels from the upper '1 - fraction' proportion.
// Then use this threshold to segment the image.
    setAndCheck(In, Out);
    int w = In->getWidth();
    int h = In->getHeight();

    // Create and initialize image histogram
    float* hist = new float[bins];
    for ( int i=0; i<bins; i++ )
        hist[i] = 0;

    // Fill histogram
    float max = In->getMax();
    float min = In->getMin();
cout << "max: " << max << endl;
cout << "min: " << min << endl;
    float range = max - min;
    for ( int y=0; y<h; y++ )
        for ( int x=0; x<w; x++) {
            float value = In->get(x, y);
            int bin = (int) round(((value - min)/range) * (bins - 1));
            hist[bin]++;
        }
cout << "HIST:" << endl;
for ( int i=0; i<bins; i++ )
cout << hist[i] << endl;

    // Choose the bin such that at least 'fraction' percent of image pixels are
    // in that bin or a lower bin. 
    float sum = 0;
    float n = w*h;
    int thresholdBin = -1;
    while ( sum / n < fraction) {
        thresholdBin++;
        sum += hist[thresholdBin];
    }
    float threshold;
    if ( thresholdBin == -1 )
        threshold = 0;
    else
        threshold = ((thresholdBin + 1) / (float) bins) * range + min;
    cout << "threshold bin: " << thresholdBin << endl;
    cout << "threshold: " << threshold << endl;

    // Finally, threshold the image
    for ( int y=0; y<h; y++ )
        for ( int x=0; x<w; x++) {
            if (In->get(x, y) < threshold )
                Out->set(x, y, 0);
            else
                Out->set(x, y, 1);
        }

    delete []hist;
}

class Pixel {
public:
    Pixel( int inX=0, int inY=0, float inValue=0) {
        x = inX;
        y = inY;
        value = inValue;
    };
    int x;
    int y;
    float value;
};

int comparePixels(const void * a, const void * b) {
  if ( (*(Pixel*)a).value > (*(Pixel*)b).value ) return 1;
  if ( (*(Pixel*)a).value ==(*(Pixel*)b).value ) return 0;
  if ( (*(Pixel*)a).value < (*(Pixel*)b).value ) return -1;
}

void ImgOps::adaptiveThreshold(Img* In, float percentile, Img *&Out) {
// Threshold the input image according to an adaptive threshold.  This threshold
// is determined by sorting all positive pixel values and determining the value
// which segments the sorted array (of positive pixel values) into two
// sub-arrays of size (percentile)*n and (1-percentile)*n, where n = width *
// height.  Effectively, pixels belonging to the first sub-array will be
// thresholded to 0, while the rest will be set to 1.
    setAndCheck(In, Out);
    int w = In->getWidth();
    int h = In->getHeight();
    int n = w * h;

    // Create and fill array for sorting
    Pixel* array = new Pixel[n];
    int p = 0;
    for ( int y=0; y<h; y++ )
        for ( int x=0; x<w; x++) {
            float value = In->get(x, y);
            if ( value > 0) {
                array[p].x = x;
                array[p].y = y;
                array[p].value = value;
                p++;
            }
        }

cout << "Percentage of positives: " << p/(float)n << endl;

    // Sort
    qsort(array, p, sizeof(Pixel), comparePixels);

int highs = 0;
    Out->setAll(0);
    int begin = (int)(p*percentile + 0.5);
    int end = p;
    for ( int i=begin; i<end; i++) {
        int x = array[i].x;
        int y = array[i].y;
        Out->set(x, y, 1);
highs++;
    }
cout << "percent positives set high: " << highs / (float)p << endl;
cout << "percent total set high: " << highs / (float)n << endl;
    
    delete []array;
}

void ImgOps::drawVectorImage(Img* U, Img* V, int imageScale, int spacing,
                              float vectorScale, bool unit, Img *&Out) {
    int w = U->getWidth();
    int h = U->getHeight();
    int ow = imageScale * w;
    int oh = imageScale * h;
    if ( Out == NULL )
        Out = new Img(ow, oh);
    Out->setAll(0);
    
    for ( int y=0; y<h; y+=spacing )
        for ( int x=0; x<w; x+=spacing) {

            float u = U->get(x, y);
            float v = V->get(x, y);
            if ( unit) {
                // Make vector unit length
                float length = sqrt(u*u + v*v);
                if ( length != 0) {
                    u /= length;
                    v /= length;
                }
            }
            
            u *= imageScale * vectorScale;
            v *= imageScale * vectorScale;

            // Draw the vector into the output image
            int ox = imageScale * x;
            int oy = imageScale * y;
            CvPoint endPoint = cvPoint( (int)round(ox+u), (int)round(oy+v) );
            // Draw a rectangle as the base of the vector
            cvRectangle(Out->image, cvPoint(ox-1, oy-1), cvPoint(ox+1, oy+1),
                        CV_RGB(0.4, 0.4, 0.4), -1, CV_AA);
            cvLine(Out->image, cvPoint(ox, oy), endPoint, CV_RGB(1, 1, 1), 1, 
                   CV_AA);
        }
}

void ImgOps::drawLine(Img *In, int x1, int y1, int x2, int y2) {
    assert(In != NULL);
    cvLine(In->image, cvPoint(x1, y1), cvPoint(x2, y2), CV_RGB(1, 1, 1), 1, 
           CV_AA);
}

void ImgOps::preOpticFlow(Img &A, Img &B, IplImage *&byteA, IplImage *&byteB) {
    CvSize size = cvSize(A.getWidth(), A.getHeight());

    // We need 8-bit versions of A and B.
    byteA = cvCreateImage(size, IPL_DEPTH_8U, 1);
    byteB = cvCreateImage(size, IPL_DEPTH_8U, 1);
    cvConvertScale(A.image, byteA, 255);
    cvConvertScale(B.image, byteB, 255);
}

void ImgOps::postOpticFlow(IplImage *byteA, IplImage *byteB) {
    // Deallocate 8-bit images.
    cvReleaseImage(&byteA);
    cvReleaseImage(&byteB);
}

/*
void ImgOps::lucasKanade(Img &A, Img &B, Img *&U, Img *&V) {
    IplImage *byteA, *byteB;
    preOpticFlow(A, B, byteA, byteB);
    
    int w = byteA->width;
    int h = byteB->height;
    if (U == NULL) {
        U = new Img(w, h);
        V = new Img(w, h);
    }

    cvCalcOpticalFlowLK(byteA, byteB,
                        cvSize(5, 5),
                        U->image, V->image);

    postOpticFlow(byteA, byteB);
}

void ImgOps::hornSchunck(Img &A, Img &B, Img *&U, Img *&V) {
    IplImage *byteA, *byteB;
    preOpticFlow(A, B, byteA, byteB);
    
    int w = byteA->width;
    int h = byteB->height;
    if (U == NULL) {
        U = new Img(w, h);
        V = new Img(w, h);
    }

    cvCalcOpticalFlowHS(byteA, byteB,
                    0, // use previous velocity field
                    U->image, V->image,
                    0.5, // lambda --- smoothness weight
                    cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 100, 0.1));

    postOpticFlow(byteA, byteB);
}

void ImgOps::blockMatch(Img &A, Img &B, Img *&U, Img *&V) {
    IplImage *byteA, *byteB;
    preOpticFlow(A, B, byteA, byteB);

    int w = byteA->width;
    int h = byteB->height;

    CvSize blockSize = cvSize(11, 11);
    CvSize shiftSize = cvSize(1, 1);
    CvSize rangeSize = cvSize(2, 2); //30
    int outW = (w - blockSize.width) / shiftSize.width;
    int outH = (h - blockSize.height) / shiftSize.height;

    if (U == NULL) {
        U = new Img(outW, outH);
        V = new Img(outW, outH);
    }

    cvCalcOpticalFlowBM(byteA, byteB,
                        blockSize, shiftSize, rangeSize,
                        0, // use previous motion field.
                        U->image, V->image);

    postOpticFlow(byteA, byteB);
}
*/

void ImgOps::setAndCheck(Img* In, Img*& Out) {
    int width = In->getWidth();
    int height = In->getHeight();
    if ( Out == NULL )
        Out = new Img(width, height);
    else {
        if ( Out->getWidth() != width || Out->getHeight() != height) {
            cout << "ImgOps: Image size mismatch!" << endl;
            exit(-1);
        }
        Out->setAll(0);
    }
}
