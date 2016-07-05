#include "DriverAdaptor.h"
#include <cassert>
#include <cstdlib>
#include <glog/logging.h>

DriverAdaptor::DriverAdaptor(Position2dProxy *inPosition,
                             CursesListener *inListener, int *inKey,    
                             Camera *inCamera, bool inSimulator)
    : position(inPosition), listener(inListener),
      key(inKey), camera(inCamera), simulator(inSimulator),
      rawImg(NULL)
{
}

DriverAdaptor::~DriverAdaptor() {
    delete rawImg;
}

double DriverAdaptor::getX() {
    return position->GetXPos();
}

double DriverAdaptor::getY() {
    return position->GetYPos();
}

double DriverAdaptor::getTheta() {
    return position->GetYaw();
}

void DriverAdaptor::setSpeed(double v, double omega) {
    position->SetSpeed(v, omega);
}

void DriverAdaptor::goTo(double x, double y, double theta) {
    player_pose2d_t desired;
    desired.px = x;
    desired.py = y;
    desired.pa = theta;
    position->GoTo(desired);
}

bool DriverAdaptor::isStalled() {
    return position->GetStall();
}

void DriverAdaptor::stop() {
    // Set the desired position to the current position.  Using setSpeed(0, 0)
    // does not work as expected with a position-mode driver like snd.
    player_pose2d_t desired;
    desired.px = position->GetXPos();
    desired.py = position->GetYPos();
    desired.pa = position->GetYaw();
    position->GoTo(desired);
}

void DriverAdaptor::printHeader(string msg) {
    listener->printHeader(msg);
}

void DriverAdaptor::print(string msg) {
    listener->print(msg);
}

int DriverAdaptor::getKey() {
    return *key;
}

void DriverAdaptor::getImg(Img *&img) {
    assert(camera != NULL);

    if (simulator) {
        // We assume that the camera used when in simulator mode has
        // pre-unfolded images, so they can be returned directly.
        camera->getImg(img);
    } else {
        camera->getImg(rawImg);

        // Unfold the image to go from the hyperbolic 'donut' to a rectangular
        // panoramic image.
        unfolder.unfold(*rawImg, img);
    }

    img->getKeypoints().clear();
}

void DriverAdaptor::captureFromObservationCamera(string filename) {
    ostringstream command;
    if (simulator)
        command << "import -window \"Stage: ./gallery.world\" " 
                << filename;
    else 
        command << "ssh 192.168.0.103 \'~/bin/capture " << filename
                << "\' &> /dev/null";
    system(command.str().c_str());
    
    LOG(INFO) << command.str();
}
