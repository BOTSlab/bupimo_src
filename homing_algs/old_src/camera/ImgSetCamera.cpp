#include "ImgSetCamera.h"
#include "Angles.h"
#include "SingleConfig.h"
#include <sys/file.h>
#include <sstream>
#include <fstream>
#include <cassert>

ImgSetCamera::ImgSetCamera() {

    SingleConfig* config = SingleConfig::getInstance();
    imgSet = new ImgSet(config->getString("ImgSet.dirName"),
                        config->getString("ImgSet.prefix"));
}

ImgSetCamera::~ImgSetCamera() {
    delete imgSet;
}

bool ImgSetCamera::init() {
    return true;
}

void ImgSetCamera::getImg( Img *&img ) {
    // Get lock file used by the hacked MobileSim to indicate the pose of the
    // camera in the simulated world.  A locking mechanism is used to avoid
    // data hazards with MobileSim, which is running as a completely
    // independent process.
    int fd = open("/tmp/truePosition.lock",O_RDONLY,S_IRWXU);
    if (fd == -1) {
        cerr << "Camera: Unable to open lock file!" << endl;
        exit(-1);
    }
    int success = flock(fd, LOCK_EX);
    if (success != 0) {
        cerr << "Camera: Unable to lock file!" << endl;
        exit(-1);
    }

    // Read the true (x, y, theta) position from /tmp/truePosition.txt which
    // is created by MobileSim.
    ifstream in("/tmp/truePosition.txt");
    double x=-999, y=-999;
    double th=-999;
    in >> x;
    in >> y;
    in >> th;
    if ( x == -999 && y == -999 && th == -999 ) {
        cerr << "ImgSetCamera: Read failed!!" << endl;
        exit(-1);
    }
    x /= 1000.0;
    y /= 1000.0;
    th *= PI_OVER_180;
    
    //cout << "ImgSetCamera: read (" << x << ", " << y << ", " << th << ")" 
    //     << endl;

    // Release lock
    close(fd);

    bool gotImg = imgSet->getImg(x, y, th, img);
    assert(gotImg);

    /** PREVIOUS METHOD DID THE RENDERING DIRECTLY HERE.
     *
    // Render the image using POV-Ray
    ostringstream cmd;
    cmd << "rendom -w /data/worlds/gallery2/gallery2.pov "
        << "-i /data/worlds/gallery2/gallery2.ini "
        << "-x " << x << " -y " << y << " -t " << th
        << " > rendom.out 2>&1";
    system(cmd.str().c_str());

    // Load and return the image just rendered.
    return cvLoadImage("rendom.png", 0);
    */
}
