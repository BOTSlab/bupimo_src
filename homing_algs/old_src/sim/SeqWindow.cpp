#include "SeqWindow.h"
#include <cassert>

SeqWindow::SeqWindow( string title, ImgSequence &inSequence, bool saveToFile)
   : imgSequence(inSequence)
{
    double minX = imgSequence.getMinX();
    double minY = imgSequence.getMinY();
    double maxX = imgSequence.getMaxX();
    double maxY = imgSequence.getMaxY();

    double width = maxX - minX;
    double height = maxY - minY;
    double maxDim = max(width, height);
    visibleVectorLength = 0.05 * max(width, height);

    double margin = maxDim * 0.15;
    window = new SimWindow(title, minX-margin, minY-margin,
                           maxX+margin, maxY+margin, saveToFile);
}

SeqWindow::~SeqWindow() {
    delete window;
}

void SeqWindow::addVector(double x, double y, double magnitude, double angle,
                          double color) {
    magnitude *= visibleVectorLength;
    double x1 = x + magnitude * cos(angle);
    double y1 = y + magnitude * sin(angle);
    window->addVector(x, y, x1, y1, color);
}

void SeqWindow::addPoint(double x, double y, int code) {
    window->addPoint(x, y, code);
}

void SeqWindow::addCircle(double x, double y, double radius) {
    window->plotCircle(x, y, radius);
}

void SeqWindow::addVector(int index, double magnitude, double angle,
                          double color) {
    Pose currentPose = imgSequence.getTruePose(index);
    magnitude *= visibleVectorLength;
    double x1 = currentPose.x + magnitude * cos(angle);
    double y1 = currentPose.y + magnitude * sin(angle);
    window->addVector(currentPose.x, currentPose.y, x1, y1, color);
}

void SeqWindow::addPoint(int index, int code) {
    Pose currentPose = imgSequence.getTruePose(index);
    window->addPoint(currentPose.x, currentPose.y, code);
}

void SeqWindow::addCircle(int index, double radius) {
    Pose currentPose = imgSequence.getTruePose(index);
    window->plotCircle(currentPose.x, currentPose.y, radius);
}

void SeqWindow::clear() {
    window->clear();
}
