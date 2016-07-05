/*
 * Some common computations on arrays.
 *
 * Andrew Vardy
 */

#ifndef ARRAYS_H
#define ARRAYS_H

#include <vector>
using namespace std;

class Arrays {
public:
    /**
     * Binary search.  Returns the position of the sought key in the input
     * array (-1 if the key does not exist).  Code taken from page 62 of: 
     *
     *  A. Drozdek, "Data Structures and Algorithims in C++", 3rd Edition.
     *
     * \param arr input array
     * \param n size of input array
     * \param key the key we are searching for
     * \returns the position of the sought key, or -1 if it does not exist
     * \pre The input array must be sorted in non-decreasing order.
     */
    template<class T>
    static int binarySearch(const T arr[], int n, const T& key) {
        int lo = 0, mid, hi = n-1;
        while (lo <= hi) {
            mid = (lo + hi)/2;
            if (key < arr[mid])
                hi = mid - 1;
            else if (arr[mid] < key)
                lo = mid + 1;
            else return mid;  // success: return the index of
        }                     //   the cell occupied by key;
        return -1;            // failure: key not in the array
    }

    /**
     * Determines the longest strictly increasing subsequence of the given
     * array.  The implementation comes from here:
     *
     *  http://www.algorithmist.com/index.php/Longest_Increasing_Subsequence.cpp
     *
     * \param a input vector
     * \param b will contain the indices of the input vector which form the
     *          longest increasing subsequence (output parameter)
     */
    static void longestIncSub(vector<int> &a, vector<int> &b) {
        vector<int> p(a.size());
        int u, v;
     
        if (a.empty()) return;
     
        b.push_back(0);
     
        for (size_t i = 1; i < a.size(); i++) {
            if (a[b.back()] < a[i]) {
                p[i] = b.back();
                b.push_back(i);
                continue;
            }
     
            for (u = 0, v = b.size()-1; u < v;) {
                int c = (u + v) / 2;
                if (a[b[c]] < a[i]) u=c+1; else v=c;
            }
     
            if (a[i] < a[b[u]]) {
                if (u > 0) p[i] = b[u-1];
                b[u] = i;
            }	
        }
     
        for (u = b.size(), v = b.back(); u--; v = p[v]) b[u] = v;
    }

    /**
     * Determines the longest increasing subsequence of the given array.  The
     * notation and implementation are taken from page 3 of:  
     *
     *   [1] M.H. Albert et al, "On the longest increasing subsequence of a
     *       circular list", Inf. Process. Lett., 101:2, 55-59, 2007.
     *
     * \param x input array
     * \param n size of input array
     * \param out output array (must be size n but only the first r values will
     *          be valid).
     * \param r length of the longest increasing subsequence.
     */
/*
    static void longestIncSub(int *x, int n, int *out, int r) {

        // t stores the positions of the least possible ending terms.  That is
        // x[t[k-1]] "is the value of the least possible ending term in an
        // increasing subsequence of length k in the prefix of the sequence
        // that has been scanned to this point" [1].
        int *t = new int[n];

        r = 0;
        t[0] = 0;

        for (int i=0; i<n; i++) {
            int y = x[i];

            ///
        }

        delete t;
    }
*/

};

#endif
