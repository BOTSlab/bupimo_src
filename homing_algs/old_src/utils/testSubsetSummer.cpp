#include "SubsetSummer.h"
#include "rng.h"
#include <cassert>

void testGetBestSolution(int *input, int n, bool idealFound, bool *idealSolution) {
    cout << endl << "--testGetBestSolution-------------------------------------"
         << endl;
    cout << "\tinput: ";
    for (int i=0; i<n; i++)
        cout << input[i] << " ";
    cout << endl;

    SubsetSummer summer(input, n);
    bool found = summer.isSumPossible(0);
    bool *solution = new bool[n];
    summer.getBestSolution(solution);

    bool testPassed = found == idealFound;
    for (int i=0; i<n; i++)
        if (solution[i] != idealSolution[i])
            testPassed = false;
    if (testPassed) {
        cout << "Test passed!" << endl;
    } else {
        cout << "Test failed!" << endl;
        cout << "\tfound: " << found << endl;
    }

    cout << "\tsolution: ";
    for (int i=0; i<n; i++)
        cout << solution[i] << " ";
    cout << endl;
    delete solution;
}

void testIsSumPossible(int *input, int n, int sum, bool idealFound) {
    cout << endl << "--testIsSumPossible---------------------------------------"
         << endl;
    cout << "\tinput: ";
    for (int i=0; i<n; i++)
        cout << input[i] << " ";
    cout << endl;

    SubsetSummer summer(input, n);
    bool found = summer.isSumPossible(sum);
    bool success = (found == idealFound);

    if (success) {
        cout << "Test passed!" << endl;
    } else {
        cout << "Test failed!" << endl;
    }
}

int main() {
    {
        int n = 5;
        int input[] = {-7, -3, -2, 5, 8};
        bool idealSolution[] = {0, 1, 1, 1, 0};
        testGetBestSolution(input, n, true, idealSolution);
    }

    {
        int n = 3;
        int input[] = {10, -2, 2};
        bool idealSolution[] = {0, 1, 1};
        testGetBestSolution(input, n, true, idealSolution);
    }

    {
        int n = 3;
        int input[] = {-2, 2, 3};
        bool idealSolution[] = {1, 1, 0};
        testGetBestSolution(input, n, true, idealSolution);
    }

    {
        int n = 7;
        int input[] = {2, -5, 8, 4, 10, -14, 3};
        bool idealSolution[] = {1, 0, 1, 1, 0, 1, 0};
        testGetBestSolution(input, n, true, idealSolution);
    }

    {
        int n = 7;
        int input[] = {2, -5, 8, 4, 10, -16, 3};
        bool idealSolution[] = {1, 0, 0, 1, 1, 1, 0};
        testGetBestSolution(input, n, true, idealSolution);
    }


    // Ideal solution not possible...

    {
        int n = 3;
        int input[] = {-2, 1, 3};
        bool idealSolution[] = {1, 1, 0};
        testGetBestSolution(input, n, false, idealSolution);
    }

    {
        int n = 3;
        int input[] = {3, -2, 1};
        bool idealSolution[] = {0, 1, 1};
        testGetBestSolution(input, n, false, idealSolution);
    }

    {
        int n = 7;
        int input[] = {2, -5, 8, 4, 10, -30, 4};
        bool idealSolution[] = {0, 1, 0, 1, 0, 0, 0};
        testGetBestSolution(input, n, false, idealSolution);
    }

    {
        int n = 4;
        int input[] = {-4, -3, -1, 1};
        bool idealSolution[] = {0, 0, 1, 1};
        testGetBestSolution(input, n, true, idealSolution);
    }

    // Test isSumPossible

    {
        int n = 5;
        int input[] = {-2, -1, 0, 1, 2};
        testIsSumPossible(input, n, -3, true);
        testIsSumPossible(input, n, -2, true);
        testIsSumPossible(input, n, -1, true);
        testIsSumPossible(input, n, 0, true);
        testIsSumPossible(input, n, 1, true);
        testIsSumPossible(input, n, 2, true);
        testIsSumPossible(input, n, 3, true);
    }

    {
        int n = 3;
        int input[] = {-5, -4, -3 };
        testIsSumPossible(input, n, -5, true);
        testIsSumPossible(input, n, -4, true);
        testIsSumPossible(input, n, -3, true);
        testIsSumPossible(input, n, -2, false);
        testIsSumPossible(input, n, -1, false);
        testIsSumPossible(input, n, 0, false);
        testIsSumPossible(input, n, 1, false);
        testIsSumPossible(input, n, 2, false);
    }

    {
        int n = 3;
        int input[] = {5, 4, 3 };
        testIsSumPossible(input, n, 5, true);
        testIsSumPossible(input, n, 4, true);
        testIsSumPossible(input, n, 3, true);
        testIsSumPossible(input, n, 2, false);
        testIsSumPossible(input, n, 1, false);
        testIsSumPossible(input, n, 0, false);
        testIsSumPossible(input, n, -1, false);
        testIsSumPossible(input, n, -2, false);
    }

    // Test getBiggestSolutionForSum

    {
        cout << endl << 
             "--test getBiggestSolutionForSum--------------------------------"
             << endl;
        int n = 9;
        int input[] = {2, 20, -1, -10, 3, -2, -20, 1, 10};
        cout << "input: {2, 20, -1, -10, 3, -2, -20, 1, 10}" << endl;
        SubsetSummer summer(input, n);
        summer.buildBiggestSolutionsTable();
        bool *solution = new bool[n];
        for (int sum=-3; sum<=5; sum++) {
            cout << "\tsum: " << sum << endl;
            bool found = summer.getBiggestSolutionForSum(sum, solution);
            cout << "\tfound: " << found << endl;
            int verifySum = 0;
            for (int i=0; i<n; i++)
                if (solution[i] == 1)
                    verifySum += input[i];
            if (found)
                assert(verifySum == sum);
            cout << "\tsolution: ";
            for (int i=0; i<n; i++)
                if (solution[i] == 1)
                    cout << input[i] << " ";
            cout << endl << endl;
        }
    }

/*
    {
        // Test on random arrays of increasing size.
        RNG rng;
        for (int n = 5; n<100; n++) {
            cout << endl << 
                 "--test on random array-------------------------------------"
                 << endl;

            cout << "n: " << n << endl;
            int *input = new int[n];
            for (int i=0; i<n; i++)
                input[i] = (int) rng.uniform(-10, 10);
    
            SubsetSummer summer(input, n);
            summer.buildBiggestSolutionsTable();
            bool *solution = new bool[n];
            for (int sum=-5; sum<=5; sum++) {
                cout << "\tsum: " << sum << endl;
                bool found = summer.getBiggestSolutionForSum(sum, solution);
                cout << "\tfound: " << found << endl;
                int verifySum = 0;
                for (int i=0; i<n; i++)
                    if (solution[i] == 1)
                        verifySum += input[i];
                if (found)
                    assert(verifySum == sum);
            }

            delete [] input;
            delete [] solution;
        }
    }
*/


    return 0;
}
