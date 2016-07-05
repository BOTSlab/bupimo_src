/*
 * Image class for floating-point data in the range [0, 1].  Currently
 * implemented as an OpenCV IPL image.
 *
 * Requires: OpenCV to be installed.
 *
 * \author Andrew Vardy
 */

#ifndef IMG_H
#define IMG_H

//#include "Keypoint.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cv.h>
#include <highgui.h>
#include <vector>
using namespace std;

class Img {
public:
    /**
     * Construt an inWidth x inHeight image.
     */
    Img( int inWidth, int inHeight );

    /**
     * Construct a inWidth x inHeight image filled with value initial.
     */
    Img( int inWidth, int inHeight, float initial );

    /**
     * Copy constructor.
     */
    Img( const Img &other );

    /**
     * Load the image from the given file.  This file should be one of the
     * formats that OpenCV understands (e.g. .png, .jpg, .bmp) or it should be
     * a .grd file (my old grayscale image format).
     *
     * If a .keys file exists in the same directory as the image file, the
     * keypoints from that file will be loaded.  These keys can later be
     * accessed by 'getKeypoints'
     *
     */
    Img( string filename );
    Img( char* filename );

    /**
     * Copies the contents of this image from the given IplImage.
     */
    Img( IplImage *image );

    /**
     * Destructor.  Will destroy the image as well as any keypoints currently
     * loaded.
     */
    ~Img();

    /**
     * Assignment operator.
     */
    Img& operator=( const Img &other );

    void copyFrom( IplImage* byteImage );

    /// Accessors...

    /**
     * Return the width of the image.
     */
    int getWidth();

    /**
     * Return the height of the image.
     */
    int getHeight();

    /**
     * Get the value of pixel (x, y).
     */
    float get( int x, int y );

    /**
     * Get the value of pixel (x, y) with horizontal wrap-around for x.
     */
    float getToroidal( int x, int y );

    /**
     * Get the maximum value in the image.
     */
    float getMax();

    /**
     * Get the minimum value in the image.
     */
    float getMin();

    /**
     * Set x and y to the position of the maximum value.
     */
    void getMaxPos( int &x, int &y );

    /**
     * Set x and y to the position of the minimum value.
     */
    void getMinPos( int &x, int &y );

    /**
     * Get the sum of all pixels.
     */
    float getSum();

    /**
     * Get the unique ID number for this image.
     */
    int getID();

    /**
     * Print some image statistics to standard out.
     */
    void printInfo();

    /**
     * Save the image to the given file.
     */
    void save( string inFilename );

    /**
     * Save the image to the given file using my old grayscale image format.
     */
    void saveGrd( string inFilename );

    /**
     * Get the name of the file that this image was loaded from.
     */
    void getFilename();

    /**
     * Return true if a vector of keypoints has been associated with this Img,
     * either by being loaded at construction, or through a call to
     * setKeypoints.
     */
//    bool hasKeypoints();

    /**
     * Return a reference to the vector of keypoints previously associated with
     * this image.
     *
     * \pre The conditions for hasKeypoints returning true must be satisfied.
     */
//    vector<Keypoint*>& getKeypoints();

    /// Modifiers...

    /**
     * Set pixel (x, y) to the given value.
     */
    void set( int x, int y, float value );

    /**
     * Set all pixels to the given value.
     */
    void setAll( float f );

    /**
     * Normalize the image so that all values are in the range [0, 1].
     */
    void normalize();

    /**
     * Normalize the image so that all values are in the range [0, 255].  This
     * may be useful for exporting to other image formats for display or storage
     */
    void normalize255();

    /**
     * Copy the given keypoint pointers.  Note that the Img will destroy the
     * corresponding keypoints when it is destroyed, therefore the code that
     * calls this function should not try to destroy them.
     */
//    void setKeypoints(vector<Keypoint*>& inKeypoints);

    friend class ImgWindow;
    friend class ImgOps;

    static int wrap( int i, int m );

    static void printNumberOfImages();

    // The following two methods break the encapsulation of image
    // data by this class.  However, they can be very useful.
    IplImage *getIplImage() { return image; }

    float *getData() { return data; }

private:
    // Private helper methods...
    void init( string inFilename );
    void init( IplImage* byteImage );
    void initGrd( string inFilename );

    // Called by the first three constructors to initialize certain data
    // members from the IPL image.
    void postConstruct();
//    void deepCopyKeypoints(const vector<Keypoint*> &inKeypoints);
    void destroy();
    void normalize( float highest );

    // Member data...
public:
    IplImage *image;
private:

    int width, height, step;
    float *data;
    string filename;

    // The ID number of this image.  This should be unique for all images used
    // during a single session.
    int id;

//    vector<Keypoint*> keypoints;

    static int numberOfImages;
    static int nextID;
};

#endif
