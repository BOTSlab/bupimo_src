/*
 * Some common statistical calculations.
 *
 * Andrew Vardy
 */
#ifndef STATS_H
#define STATS_H

#include <vector>
#include <algorithm>
#include <cassert>
using namespace std;

class Stats {
public:
    // Should work for ints, floats, and doubles.
    template <class T>
    static double mean( vector<T>& vec ) {
        int n = vec.size();
        assert(n != 0);
        double sum = 0;
        for (int i=0; i<n; i++)
            sum += vec[i];
        return sum / n;
    }

    template <class T>
    static double median( vector<T>& inVec ) {
        vector<T> vec = inVec; // Operate on a copy of the original vector.

        int n = vec.size();
        assert(n != 0);
        sort(vec.begin(), vec.end());
        if (n % 2 == 0)
            // n is even.  Average middle two elements.
            return (vec[n/2 - 1] + vec[n/2])/ 2.0;
        else
            return vec[n/2];
    }
};

#endif
