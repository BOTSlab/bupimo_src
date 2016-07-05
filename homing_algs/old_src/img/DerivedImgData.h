/**
 * Represents features and related information associated with an Img.
 *
 * \author Andrew Vardy
 */

#ifndef DERIVEDIMGDATA_H
#define DERIVEDIMGDATA_H

//#include "Keypoint.h"
#include <vector>
using namespace std;

class DerivedImgData {
public:
    DerivedImgData();
    DerivedImgData(const DerivedImgData &other);
    DerivedImgData& operator=(const DerivedImgData &other);
    ~DerivedImgData();
//    bool hasKeypoints();
//    vector<Keypoint*>& getKeypoints();
private:
    void destroy();
    void deepCopy(const DerivedImgData &other);
    
//    vector<Keypoint*> keypoints;
};

#endif
