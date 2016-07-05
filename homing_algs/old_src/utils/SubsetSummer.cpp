#include "SubsetSummer.h"
#include <cassert>
#include <climits>
#include <cstdlib>

SubsetSummer::SubsetSummer(int *inInput, int inN) {

    // Make a copy of the input array (useful for calls to other methods).
    n = inN;
    input = new int[n];
    for (int i=0; i<n; i++)
        input[i] = inInput[i];

    // Determine the sum of the negative values (N) and positive values (P).
    N = 0, P = 0;
    for (int i=0; i<n; i++) {
        if (input[i] < 0)
            N += input[i];
        else
            P += input[i];
    }

    // If the input consists of all negatives then we set P to the largest
    // value.  Similarly for the case of all positives.  Note that in either of
    // these cases a sum of zero is not possible.  However, we still wish to
    // find a subset with some non-zero sum.
    int largest = INT_MIN, smallest = INT_MAX;
    for (int i=0; i<n; i++) {
        if (input[i] > largest)
            largest = input[i];
        if (input[i] < smallest)
            smallest = input[i];
    }
    if (largest < 0)
        P = largest;
    if (smallest > 0)
        N = smallest;

    //cout << "N: " << N << endl;
    //cout << "P: " << P << endl;

    // Create the DP table Q.
    width = P - N + 1;
    Q = new bool*[n];
    for (int i=0; i<n; i++)
        Q[i] = new bool[width];
    
    // Initialize the first row of Q.
    //cout << "Q[0]:" << endl;
    for (int c=0; c<width; c++) {
        int s = N + c;
        if (input[0] == s)
            Q[0][c] = true;
        else
            Q[0][c] = false;

        //cout << Q[0][c];
    }
    //cout << endl << endl;

    //cout << "Q:" << endl;
    for (int r=1; r<n; r++) {
        for (int c=0; c<width; c++) {
            int s = N + c;
            if (Q[r-1][c])
                Q[r][c] = true;
            else if (input[r] == s)
                Q[r][c] = true;
            else if (s - input[r] < N || s - input[r] > P)
                Q[r][c] = false;
            else {
                assert(s - input[r] - N >= 0 && s - input[r] - N < width);
                Q[r][c] = Q[r-1][s - input[r] - N];
            }

            //cout << Q[r][c];
        }
        //cout << endl;
    }

    bigSolTableBuilt = false;
}

SubsetSummer::~SubsetSummer() {
    delete [] input;

    // Deallocate Q.
    for (int r=0; r<n; r++)
        delete [] Q[r];
    delete [] Q;

    if (bigSolTableBuilt) {
        // Deallocate BigSolTable.
        for (int r=0; r<n; r++)
            for (int c=0; c<width; c++)
                if (BigSolTable[r][c] != NULL)
                    delete [] BigSolTable[r][c];
        for (int r=0; r<n; r++)
            delete [] BigSolTable[r];
        delete [] BigSolTable;
    }
}

bool SubsetSummer::isSumPossible(int sum) {
    if (sum < N || sum > P)
        return false;

    return Q[n-1][-N+sum];
}

void SubsetSummer::getBestSolution(bool *solution) {
    // Set the solution array to all false.
    for (int i=0; i<n; i++)
        solution[i] = false;

    // If an ideal solution does not exist.  We set solution to the subset that
    // yields the sum closest to 0.  The purpose of the loop below is to set
    // delta to the value of this non-ideal subset.
    int delta = 0;
    if (!isSumPossible(0)) {
        delta = 1;
        bool approxSolutionFound = false;
        int maxDelta = max(-N, P);
        for (; delta <= maxDelta; delta++) {
            if (-N-delta > 0 && Q[n-1][-N-delta]) {
                delta = -delta;
                break;
            } if (-N+delta < width && Q[n-1][-N+delta]) {
                break;
            }
        }
    }
    //cout << "delta (subset sum): " << delta << endl;

    // Fill in the solution array by following one trail of additions
    // to the subset backwards from Q[n-1][-N+delta].  Note that other subsets
    // may be possible.  In 'buildBiggestSolutionTable' we build the biggest of
    // all subsets.
    int r = n-1;
    int c = -N+delta;
    int partialSum = 0;
    for (; r > 0; r--) {
        if ((Q[r-1][c] == 0)) {
            // The subset without input[r] did not sum to 0. input[r] is needed.
            solution[r] = true;

            // On the next loop we will look at the value of Q corresponding to
            // the sum of the subset without input[r], obtained by subtracting
            // input[r].
            c -= input[r];

            partialSum += input[r];
            if (partialSum == delta)
                break;
        }
    }
    // Special check required for input[0].
    if (partialSum != delta)
        solution[0] = true;
}

bool SubsetSummer::getBiggestSolutionForSum(int sum, bool *solution) {
    assert(bigSolTableBuilt);

    // Set the solution array to all zeros.
    for (int i=0; i<n; i++)
        solution[i] = false;

    if (!Q[n-1][-N+sum])
        return false;

    // Copy the solution from the table.
    bool *storedSolution = BigSolTable[n-1][-N+sum];
    for (int i=0; i<n; i++)
        solution[i] = storedSolution[i];

    return true;
} 

void SubsetSummer::buildBiggestSolutionsTable() {
    itBigSolTableBuilder();
    bigSolTableBuilt = true;
}

/*
void SubsetSummer::recSetBuilder(int row, int col, VEC_OF_SETS &result) {
//cout << "row: "<<row<<"\tcol: "<<col<<endl;
    if (row < 0)
        // Base case.
        return;

    if (!Q[row][col])
        // Other base case.  If we can't form this sum, then there is naturally
        // no viable subset for it.
        return;

    // Add to result the sets from the same column of the previous row.
    recSetBuilder(row-1, col, result);
    
    // Add to result augmented sets from the column of the previous row shifted
    // by the input corresponding to the current row.
    VEC_OF_SETS S;
    recSetBuilder(row-1, col-input[row], S);
    VEC_OF_SETS::iterator it = S.begin();
    for (; it != S.end(); it++) {
        SET augmentedSet;
        augmentedSet.insert(it->begin(), it->end());
        augmentedSet.insert(row);
        result.push_back(augmentedSet);
    }

    // Add to result the current input if it corresponds to the current column.
    if (input[row] == N+col) {
        SET justInputR;
        justInputR.insert(row);
        result.push_back(justInputR);
    }
}
*/

void SubsetSummer::itBigSolTableBuilder() {

    // Build the solution table, filled with NULL values.
    BigSolTable = new bool**[n];
    for (int r=0; r<n; r++) {
        BigSolTable[r] = new bool*[width];
        for (int c=0; c<width; c++)
            BigSolTable[r][c] = NULL;
    }

    // Initialize the first row of the table.
    for (int c=0; c<width; c++) {
//cout << "c: " << c << endl;
        int s = N + c;
        if (input[0] == s) {
            bool *set = new bool[n];
            for (int i=1; i<n; i++)
                set[i] = false;
            set[0] = true;
            BigSolTable[0][c] = set;
        } 
    }

    for (int r=1; r<n; r++) {
//cout << "r: " << r << endl;
        for (int c=0; c<width; c++) {
//cout << "c: " << c << endl;
            int s = N + c;

            // The solution table entry for (r, c) is the largest of the
            // following sets:
            //  - BigSolTable[r-1][shifted] U {r}: where shifted = c - input[r]
            //    and shifted >= 0 && shifted < width
            //  - BigSolTable[r-1][c]
            //  - {r}: if input[r] == s
            //
            // Call these three sets A, B, and C.  If either of A or B has
            // non-zero size, they will be the largest.  Therefore we first
            // calculate the sizes of A and B.  If either is non-empty then the
            // larger one will be used for the current entry of the table.  If
            // both are empty and C is not, C will be used.  Otherwise the
            // current entry will remain empty. 
            
            int ASize = 0;
            int shifted = c - input[r];
            if (shifted >=0 && shifted < width && Q[r-1][shifted]) {
                for (int i=0; i<n; i++)
                    ASize += BigSolTable[r-1][shifted][i];
                ASize++;
            }

            int BSize = 0;
            if (Q[r-1][c])
                for (int i=0; i<n; i++)
                    BSize += BigSolTable[r-1][c][i];

            bool chooseA = false, chooseB = false, chooseC = false;

/*
            if (ASize > 0 && ASize == BSize) {
                // Sets A and B are of equal size.  Choose the one whose
                // absolute sum is smaller.  This favours sets that are more
                // closely clustered around 0.
                bool *setA = new bool[n];
                bool *setB = new bool[n];
                for (int i=0; i<n; i++)
                    setA[i] = BigSolTable[r-1][shifted][i];
                setA[r] = true;
                for (int i=0; i<n; i++)
                    setB[i] = BigSolTable[r-1][c][i];

                int sumA = 0, sumB = 0;
                for (int i=0; i<n; i++) {
                    if (setA[i])
                        sumA += abs(input[i]);
                    if (setB[i])
                        sumB += abs(input[i]);
                }

                if (sumA < sumB)
                    chooseA = true;
                else
                    chooseB = true;

                // BAD: Inefficient!
                delete setA;
                delete setB;

            } else 
*/

            if (ASize > 0 && ASize >= BSize)
                chooseA = true;
            else if (BSize > 0 && BSize >= ASize)
                chooseB = true;
            else if (input[r] == s)
                chooseC = true;

            if (chooseA) {
                bool *set = new bool[n];
                for (int i=0; i<n; i++)
                    set[i] = BigSolTable[r-1][shifted][i];
                set[r] = true;
                BigSolTable[r][c] = set;
            } else if (chooseB) {
                bool *set = new bool[n];
                for (int i=0; i<n; i++)
                    set[i] = BigSolTable[r-1][c][i];
                BigSolTable[r][c] = set;
            } else if (chooseC) {
                bool *set = new bool[n];
                for (int i=0; i<n; i++)
                    set[i] = false;
                set[r] = true;
                BigSolTable[r][c] = set;
            }

        }
    }

/*
cout << endl << "BigSolTable sizes" << endl;
for (int r=0; r<n; r++) {
    cout << "r: " << r << endl;
    for (int c=0; c<width; c++) {
        int size = 0;
        if (BigSolTable[r][c] != NULL)
            for (int i=0; i<n; i++)
                size += BigSolTable[r][c][i];
        cout << size << " ";
    }
    cout << endl;
}
*/
    
}
