/**
 * An abstract Camera class.
 *
 * Andrew Vardy
 */
#ifndef CAMERA_H
#define CAMERA_H

#include "Img.h"
#include <iostream>
#include <string>
using namespace std;

class Camera {
public:
    virtual ~Camera() = 0;
    virtual void getImg(Img *&img) = 0;
};

#endif
