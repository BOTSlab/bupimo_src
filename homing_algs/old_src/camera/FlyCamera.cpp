#include "FlyCamera.h"

using namespace FlyCapture2;

FlyCamera::FlyCamera() /*: logFile("FlyCamera.log") */ {
    init();
}

FlyCamera::~FlyCamera() {
    // Stop capturing images
    Error error;
    error = cam.StopCapture();
    if (error != PGRERROR_OK) {
        printError( error );
        return;
    }      

    // Disconnect the camera
    error = cam.Disconnect();
    if (error != PGRERROR_OK) {
        printError( error );
    }
}

void FlyCamera::printError(Error error) {
    //error.PrintErrorTrace();
    cout << error.GetDescription() << endl;
}

void FlyCamera::getImg( Img *&img ) {
    Error error;

    Image rawImage;    
    // Retrieve an image
    error = cam.RetrieveBuffer( &rawImage );
    if (error != PGRERROR_OK) {
        printError( error );
        return;
    }

    // Create a greyscale image
    Image greyImage;

    // Convert the raw image
    error = rawImage.Convert( PIXEL_FORMAT_MONO8, &greyImage );
    if (error != PGRERROR_OK) {
        printError( error );
        return;
    }

    // Convert to Img.
    int width = greyImage.GetCols();
    int height = greyImage.GetRows();
   if (img == NULL)
        img = new Img(width, height);
    else
        assert(img->getWidth() == width && img->getHeight() == height);
    for (int i=0; i<width; i++)
        for (int j=0; j<height; j++)
            img->set(i, j, *greyImage(j,i) / 255.0f);
}

bool FlyCamera::init() {
    Error error;

    // Connect to the camera.
    error = cam.Connect();   // Connects to first camera detected
    if (error != PGRERROR_OK) {
        printError( error );
        return false;
    }

    // Get the camera information
    CameraInfo camInfo;
    error = cam.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK) {
        printError( error );
        return false;
    }

    cout << "FlyCamera: Camera model detected: "
            << camInfo.modelName << endl;

    // Start capturing images
    error = cam.StartCapture();
    if (error != PGRERROR_OK) {
        printError( error );
        return false;
    }

    // The first 2-3 images captured seem to have artefacts.  Therefore, we'll
    // capture 5 in a row as throwaways...
    for (int i=0; i<5; i++) {
        Img *img = NULL;
        getImg(img);
        delete img;
    }

    return true;
}
