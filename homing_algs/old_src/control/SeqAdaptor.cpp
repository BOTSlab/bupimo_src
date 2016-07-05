#include "SeqAdaptor.h"
#include <cassert>

SeqAdaptor::SeqAdaptor(ImgSequence &inSequence)
    : DriverAdaptor(NULL, NULL, NULL, NULL, false), // Base class implementation
      sequence(inSequence), index(-1)                // not used at all (BAD:
{}                                                  // would be better to make
                                                    // it pure abstract)
SeqAdaptor::~SeqAdaptor() {
}

double SeqAdaptor::getX() {
    return sequence.getOdoPose(index).x;
}

double SeqAdaptor::getY() {
    return sequence.getOdoPose(index).y;
}

double SeqAdaptor::getTheta() {
    return sequence.getOdoPose(index).theta;
}

void SeqAdaptor::setSpeed(double v, double omega) {
}

void SeqAdaptor::goTo(double x, double y, double theta) {
}

void SeqAdaptor::printHeader(string msg) {
    print(msg);
}

void SeqAdaptor::print(string msg) {
    cout << msg << endl;
}

int SeqAdaptor::getKey() {
}

void SeqAdaptor::getImg(Img *&img) {
    index++;
    img = sequence.getImg(index);
}
