#include "test.h"

void print_array(std::vector< std::vector < int > > myarray)
{
    for (int i=0; i<2; i++)
        for (int j=0; j<2; j++)
            printf("[%d][%d] = [%d]\n", i, j, myarray[i][j]);
}

