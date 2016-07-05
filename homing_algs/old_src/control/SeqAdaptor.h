/**
 * A faked variant of DriverAdaptor that gets images from an ImgSequence.
 *
 * @Andrew Vardy
 */

#ifndef SEQADAPTOR_H
#define SEQADAPTOR_H

#include "DriverAdaptor.h"
#include "ImgSequence.h"

class SeqAdaptor : public DriverAdaptor {
public:
    SeqAdaptor(ImgSequence &sequence);

    virtual ~SeqAdaptor();

    double getX();
    double getY();
    double getTheta();

    /**
     * Does nothing.
     */
    void setSpeed(double v, double omega);

    /**
     * Does nothing.
     */
    void goTo(double x, double y, double theta);

    /**
     * Just prints a message.
     */
    void printHeader(string msg);

    /**
     * Print a message to the console.
     */
    void print(string msg);

    /**
     * Does nothing.
     */
    int getKey();

    /**
     * Set img to the next image from the sequence, then advance index.
     */
    void getImg(Img *&img);

private:
    ImgSequence sequence;
    int index;
};

#endif
