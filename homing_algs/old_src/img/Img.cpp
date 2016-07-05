#include <cassert>
#include <sstream>
#include <vector>
#include "Img.h"
//#include "SiftExtractor.h"
#include "Files.h"
//#include <glog/logging.h>

int Img::numberOfImages = 0;
int Img::nextID = 0;

Img::Img( int inWidth, int inHeight ) {
    image = cvCreateImage(cvSize(inWidth, inHeight), IPL_DEPTH_32F, 1);
    postConstruct();
}

Img::Img( int inWidth, int inHeight, float initial ) {
    image = cvCreateImage(cvSize(inWidth, inHeight), IPL_DEPTH_32F, 1);
    postConstruct();
    setAll(initial);
}

Img::Img( const Img &other ) {
    image = cvCloneImage(other.image);
    postConstruct();
    // We must make a separate copy of any keypoints (otherwise they could
    // get deleted twice when the destructor is called).
//    deepCopyKeypoints(other.keypoints);
}

Img::Img( string inFilename ) {
    init(inFilename);
}

void Img::postConstruct() {
    width = image->width;
    height = image->height;
    step = image->widthStep/sizeof(float);
    data = (float *)image->imageData;
    numberOfImages++;
    id = nextID++;
    filename = "";
}

/*
void Img::deepCopyKeypoints(const vector<Keypoint*> &inKeypoints) {
    int nKeys = (int) inKeypoints.size();
    for (int i=0; i<nKeys; i++) {
        Keypoint *copy = new Keypoint(*inKeypoints[i]);
        keypoints.push_back(copy);
    }
}
*/

Img::Img( char* inFilename ) {
    init(string(inFilename));
}

void Img::init( string inFilename ) {
    if (inFilename.substr(inFilename.length()-3) == "grd") {
        initGrd(inFilename);
        return;
    }

    filename = inFilename;
    IplImage* byteImage = cvLoadImage(filename.c_str(), 0);
    init(byteImage);
    cvReleaseImage(&byteImage);

    // If a corresponding .keys file exists at the same location, load the
    // keypoints.
//    string keysFile = inFilename.substr(0, inFilename.length()-3);
//    keysFile.append("keys");
//    if (Files::fileExists(keysFile)) {    
//        LOG(INFO) << "Loading: " << keysFile << endl;
//        SiftExtractor::loadKeys(keysFile, keypoints);
//    }
}

void Img::init( IplImage* byteImage ) {
    assert(byteImage->nChannels == 1);
    assert(byteImage->depth == IPL_DEPTH_8U);

    width = byteImage->width;
    height = byteImage->height;
    int byteStep = byteImage->widthStep/sizeof(uchar);
    uchar* byteData = (uchar *)byteImage->imageData;

    // Create a floating point image and copy the loaded image into it
    image = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
    data = (float *)image->imageData;
    step = image->widthStep/sizeof(float);
    for ( int y=0; y<height; y++ )
        for ( int x=0; x<width; x++ ) {
            uchar val = byteData[y*byteStep+x];
            data[y*step+x] = val / 255.0f;
        }

    numberOfImages++;
    id = nextID++;
}

void Img::initGrd( string inFilename ) {
    ifstream in(inFilename.c_str());

    // First put all of the file's lines into a vector.  Number of lines is
    // the image's height.
    vector<string> lines;
    string line;
    while( getline(in, line) )
        lines.push_back(line);
    height = lines.size();

    // Now find out the width of the image by reading one line.
    stringstream firstLineStream(lines[0]);
    string word;
    width = 0;
    while ( firstLineStream >> word ) // Behaviour is to break at whitespace
        width++;
    std::cout << "width: " << width << endl;
    std::cout << "height: " << height << endl;

    // Now create the IPL image
    image = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
    step = image->widthStep/sizeof(float);
    data = (float *)image->imageData;
    
    // Fill 'er up
    for ( int y=0; y<lines.size(); y++ ) {
        stringstream lineStream(lines[y]);
        int x = 0;
        while ( lineStream >> word ) {
            stringstream wordStream(word);
            float fValue;
            wordStream >> fValue;
            set(x++, y, fValue);
        }
    }

    numberOfImages++;
    id = nextID++;
    filename = inFilename;
}

Img& Img::operator=( const Img &other ) {
    destroy();
    image = cvCloneImage(other.image);
    postConstruct();
    // We must make a separate copy of any keypoints (otherwise they could
    // get deleted twice when the destructor is called).
//    deepCopyKeypoints(other.keypoints);
}

Img::Img( IplImage *image ) {
    init(image);
}

void Img::copyFrom( IplImage* byteImage ) {
    assert(byteImage->nChannels == 1);
    assert(byteImage->depth == IPL_DEPTH_8U);
    assert(width == byteImage->width);
    assert(height == byteImage->height);

    int byteStep = byteImage->widthStep/sizeof(uchar);
    uchar* byteData = (uchar *)byteImage->imageData;

    // Create a floating point image and copy the loaded image into it
    image = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
    data = (float *)image->imageData;
    step = image->widthStep/sizeof(float);
    for ( int y=0; y<height; y++ )
        for ( int x=0; x<width; x++ ) {
            uchar val = byteData[y*byteStep+x];
            data[y*step+x] = val / 255.0f;
        }
}

void Img::normalize() {
    normalize(1);
}

void Img::normalize255() {
    normalize(255);
}

void Img::normalize( float highest ) {
// Postcondition: All image values are in the range [0, 1]
    // First find minimum and maximum values in image
    float max = -FLT_MAX;
    float min = FLT_MAX;
    float val=0;
    for ( int y=0; y<height; y++ )
        for ( int x=0; x<width; x++ ) {
            val = data[y*step+x];
            if ( val > max )
                max = val;
            if ( val < min )
                min = val;
        }

    if (max == min) {
        // All pixels have the same value.  If that value is 0 or 1 we will
        // do nothing.  For any other value we will set all image values to 1.
        if (max != 0 && max != 1)
            setAll(1);
    } else {
        // Now transform each value from the range [min, max] to [0, 1]
        for ( int y=0; y<height; y++ )
            for ( int x=0; x<width; x++ ) {
                val = data[y*step+x];
                data[y*step+x] = highest*(val - min) / (max - min);
            }
    }
}

Img::~Img() {
    destroy();
}

void Img::destroy() {
    cvReleaseImage(&image);
    numberOfImages--;

//    SiftExtractor::clearKeys(keypoints);
}

int Img::getWidth() {
    return width;
}

int Img::getHeight() {
    return height;
}

float Img::get( int x, int y ) {
    assert( x >= 0 && x < width && y >= 0 && y < height );

    return data[y*step+x];
}

float Img::getToroidal( int x, int y ) {
    int ix = wrap(x, width);
    int iy = wrap(y, height);
    return data[iy*step+ix];
}

float Img::getMax() {
    float max = -FLT_MAX;
    float val=0;
    for ( int y=0; y<height; y++ )
        for ( int x=0; x<width; x++ ) {
            val = data[y*step+x];
            if ( val > max )
                max = val;
        }
    return max;
}

float Img::getMin() {
    float min = FLT_MAX;
    float val=0;
    for ( int y=0; y<height; y++ )
        for ( int x=0; x<width; x++ ) {
            val = data[y*step+x];
            if ( val < min )
                min = val;
        }
    return min;
}

void Img::getMaxPos( int &maxX, int &maxY ) {
    float max = -FLT_MAX;
    float val=0;
    for ( int y=0; y<height; y++ )
        for ( int x=0; x<width; x++ ) {
            val = data[y*step+x];
            if ( val > max ) {
                max = val;
                maxX = x;
                maxY = y;
            }
        }
}

void Img::getMinPos( int &minX, int &minY ) {
    float min = FLT_MAX;
    float val=0;
    for ( int y=0; y<height; y++ )
        for ( int x=0; x<width; x++ ) {
            val = data[y*step+x];
            if ( val < min ) {
                min = val;
                minX = x;
                minY = y;
            }
        }
}

float Img::getSum() {
    float sum = 0;
    for ( int y=0; y<height; y++ )
        for ( int x=0; x<width; x++ )
            sum += data[y*step+x];
    return sum;
}

int Img::getID() {
    return id;
}

void Img::printInfo() {
    cout << "width: " << width << endl;
    cout << "height: " << height << endl;
    cout << "min: " << getMin() << endl;
    cout << "max: " << getMax() << endl;
    cout << "sum: " << getSum() << endl;
}

void Img::save( string filename ) {
    Img copy(*this);
    copy.normalize();
    
    // Create an 8-bit image.
    IplImage* converted = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

    // Convert from the 'copy' image to the 8-bit image, then save.
    cvConvertScale(copy.image, converted, 255);
    cvSaveImage(filename.c_str(), converted);

    cvReleaseImage(&converted);
}

void Img::saveGrd( string filename ) {
    ofstream out(filename.c_str());

    // Write image information.
//    out << "width: " << width << endl;
//    out << "height: " << height << endl;
    for ( int y=0; y<height; y++ ) {
        for ( int x=0; x<width; x++ ) {
            out << get(x,y);
            if ( x != width-1)
                out << " ";
        }
        if ( y != height-1)
            out << endl;
    }
}

/*
bool Img::hasKeypoints() {
    return keypoints.size() > 0;
}

vector<Keypoint*>& Img::getKeypoints() {
    return keypoints;
}

void Img::setKeypoints(vector<Keypoint*> &inKeypoints) {
    assert(keypoints.size() == 0);

    int nKeys = (int) inKeypoints.size();
    for (int i=0; i<nKeys; i++)
        keypoints.push_back(inKeypoints[i]);
}
*/

void Img::set( int x, int y, float value ) {
    data[y*step+x] = value;
}

void Img::setAll( float f ) {
    for ( int y=0; y<height; y++ )
        for ( int x=0; x<width; x++ )
            data[y*step+x] = f;
}

int Img::wrap( int i, int m ) {
    if ( i >= 0 )
        return i % m;
    int v = (i % m) + m;
    if ( v == m )
        v = 0;
    return v;
}

void Img::printNumberOfImages() {
    cout << "numberOfImages: " << numberOfImages << endl;
}
