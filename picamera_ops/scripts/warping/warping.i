%module warping
%{
#include "warping.h"
%}

%include "std_vector.i"

namespace std {
%template(Array) vector < float>;
}   

void init(int n);

void set_snapshot(int i, std::vector < float > oneDimage);

void compute_home_vector(int i, std::vector < float > oneDimage);

float get_home_x();

float get_home_y();
