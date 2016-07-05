%module test
%{
#include "test.h"
%}

%include "std_vector.i"

namespace std {
%template(Line)  vector < int >;
    %template(Array) vector < vector < int> >;
}   

void print_array(std::vector< std::vector < int > > myarray);
