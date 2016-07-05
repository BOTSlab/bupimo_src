/**
 * Class to solve the sub-set sum problem.
 *
 * Uses the dynamic programming solution described here:
 *       http://en.wikipedia.org/wiki/Subset_sum_problem
 * for building the Q-table of possible sums that can be formulated given the
 * input array (passed to the constructor).
 *
 * \author Andrew Vardy
 */

#ifndef SUBSETSUMMER_H
#define SUBSETSUMMER_H

#include <iostream>
using namespace std;

class SubsetSummer {
public:
    /**
     * Constructor.  The parameters define a particular instance of the
     * subset-sum problem.  Internally, the Q table is built.
     *
     * \param inInput An array of ints.
     * \param inN The size of input.
     */
    SubsetSummer(int *inInput, int inN);

    /**
     * Destructor.
     */
    ~SubsetSummer();

    /**
     * Determine if there exists a subset of the entries of the input array
     * whose sum is the given value.
     *
     * \returns Whether or not an optimal (i.e. sum of 0) solution has been
     * found.
     */
    bool isSumPossible(int sum);

    /**
     * If an optimal (i.e. sum of 0) solution exists indicate the corresponding
     * members of the subset by the true values in 'solution'.  If not, set the
     * best sub-optimal solution in 'solution'.  The best sub-optimal solution
     * yields the lowest sum (sub-optimal because 0 cannot be reached).
     *
     * NOTE: The size of the solution (i.e. number of members of the subset) is
     * arbitrary.  To determine the solution with the largest size (i.e.
     * largest subset of the input array), call 'buildBiggestSolutionsTable'
     * then 'getBiggestSolutionForSum'.
     *
     * \param solution An array of n values indicating whether the
     * corresponding entry in the input array forms part of the solution.
     */
    void getBestSolution(bool *solution);
    
    /**
     * Build a table containing all possible subsets of the input array for all 
     * entries of the Q table.
     */
    void buildBiggestSolutionsTable();

    /**
     * If a subset of the input array with the given sum exists, return true
     * and set the indices of this subset in 'solution'.
     *
     * \param solution An array of n values indicating whether the
     * corresponding entry in the input array forms part of the solution.
     *
     * \pre buildBiggestSolutionsTable has been called.
     */
    bool getBiggestSolutionForSum(int sum, bool *solution);

private:
    /**
     * Recursively computes entry (row, col) of BigSolTable.
     */
    //void recSetBuilder(int row, int col, VEC_OF_SETS &result);

    /**
     * Iteratively computes all entries of BigSolTable.
     */
    void itBigSolTableBuilder();

    // A copy of the input array (size n).
    int *input;
    int n;

    // N is the sum of the negative values in 'input'.  P is the sum of the
    // positive values.  width = P - N + 1;
    int N, P, width;

    // The Q table indicates the sums that can be formed.  Each row indicates an
    // entry of the input array (row 0 represents input[0]).  Each column
    // represents a possible sum, starting with N (column 0 represents N,
    // column width-1 represents P).  Q[row][col] indicates whether the
    // corresponding sum (i.e. value corresponding to col) can be formed from
    // input[0...row].  Built by constructor.
    bool **Q;

    // A table of solutions, formed with the same structure as the Q table
    // except that each entry is an array of bools, indicating which entries of
    // the input array form the solution.  Built by 'buildBiggestSolutionsTable'
    bool ***BigSolTable;

    // Whether or not BigSolTable has been built.
    bool bigSolTableBuilt;
};

#endif
