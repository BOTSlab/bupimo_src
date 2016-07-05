#include "OCVCamera.h"

OCVCamera::OCVCamera() {
    capture = cvCaptureFromCAM( CV_CAP_ANY );
    if (!capture) {
        cerr << "OCVCamera: Cannot capture camera!" << endl;
        assert(false);
    }
}

OCVCamera::~OCVCamera() {
    cvReleaseCapture(&capture);
    cvReleaseImage(&greyIpl);
    cout << "OCVCamera: destructed." << endl;
}

void OCVCamera::getImg(Img *&img) {
    IplImage* colorIpl = cvQueryFrame(capture);
    if(!colorIpl) {
        cerr << "OCVCamera: Cannot capture image!" << endl;
        img = NULL;
        return;
    }

    if (greyIpl == NULL)
        greyIpl = cvCreateImage(cvSize(colorIpl->width, colorIpl->height),
                                colorIpl->depth, 1);
    else
        assert(greyIpl->width == colorIpl->width &&
               greyIpl->height == colorIpl->height &&
               greyIpl->depth == colorIpl->depth);

    cvCvtColor(colorIpl, greyIpl, CV_RGB2GRAY);

    if (img == NULL)
        img = new Img(greyIpl);
    else
        img->copyFrom(greyIpl);
}
