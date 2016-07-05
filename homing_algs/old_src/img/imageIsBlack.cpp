#include "Img.h"
#include <iostream>
using namespace std;

int main( int argc, char *argv[] ) {

    if (argc != 2) {
        cout << "\tusage: imageIsBlack IMAGE_FILE" << endl << endl;
        cout << "Prints a 1 if image is entirely black, 0 otherwise." << endl;
        return 0;
    }

    Img img(argv[1]);
	if (img.getSum() < 1)
        cout << "1" << endl;
    else
        cout << "0" << endl;

    return 0;
}
