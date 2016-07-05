#include "SIFTHoming.h"
#include <iostream>
#include <fstream>
using namespace std;

// NOTES: - IMAGES MUST BE SAME DIMENSIONS IN ORDER FOR THIS TO WORK
//        - IF NO SIFT MATCHES ARE FOUND, RESULTING PIXEL WILL BE 0
//        - IF BLANK CONSTRUCTOR IS USED, setSS and setCV MUST BE CALLED BEFORE HOMING

int main(int argc, char **argv) {

    SIFTHoming *SIFT = NULL;
    if (argc == 1) {
        // Construct a HiSS object with (CV, SS), in this case image filenames
        // We could also use a blank constructor, then use setSS and setCV in
        // that order
        cout << "No arguments.  Choosing default files:" << endl;
        cout << "cv: 7_7.pgm, ss: 7_10.pgm" << endl;
        SIFT = new SIFTHoming("/data/db/preproc/original/4_8.pgm", 
                              "/data/db/preproc/original/8_8.pgm");
	
        // The following test should yield 1/4 width as the image direction
        //SIFTHoming * SIFT = new SIFTHoming("7_7.pgm", "7_10.pgm");
	
        // The following test should yield 3/4 width as the image direction
        //SIFTHoming * SIFT = new SIFTHoming("7_10.pgm", "7_7.pgm");
	
        // The following test should yield 2/4 width as the image direction
        //SIFTHoming * SIFT = new SIFTHoming("6_7.pgm", "3_7.pgm");
	
        // The following test should yield 0/4 or 4/4 width as the image
        // direction
        //SIFTHoming * SIFT = new SIFTHoming("3_7.pgm", "6_7.pgm");
    } else if (argc == 3) {
        cout << "cv: " << argv[1] << ", ss: " << argv[2] << endl;
        SIFT = new SIFTHoming(argv[1], argv[2]);
    } else {
        cout << "Bad arguments!" << endl;
        exit(-1);
    }
	
	// Once both the SS and CV are set, we can get the 'homing pixel'
	// Convert this to whatever angle you like based on your scene geometry
	// This pixel is relative to the CURRENT VIEW image:
	// 0 = left side of image, WIDTH = right side of image
	int pixel;
    double matchFraction;
	SIFT->getHomePixel(pixel, matchFraction);
	
	printf("\nHoming Pixel: %d\n\n", pixel);

    // Output the result to a file.
    ofstream out("hiss_output.txt");
    out << pixel << endl;
	
	// Here we could use the setCV method to set a new CV image
	// Then call the getHomingAngle() method again
	// Doing this does not recompute SS keypoints, saving time
	
	return 0;
}


