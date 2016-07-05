#include "Arrays.h"
#include <iostream>
using namespace std;

void test(int *a, int n) {
    vector<int> input(a, a+n);
    vector<int> output;
    Arrays::longestIncSub(input, output);

    cout << "input: ";
    for (int i = 0; i < input.size(); i++)
        cout << input[i] << " ";
    cout << endl << "output: ";
    for (int i = 0; i < output.size(); i++)
        cout << input[output[i]] << " ";
    cout << endl;
}

int main() {
    int a[] = {0, 10, 3, 4, 5, 8, 6, 7, 1};
    test(a, 9);

    int b[] = {10, 0, 10, 3, 4, 5, 8, 6, 7, 1};
    test(b, 10);

    int c[] = {10, 0, 10, 3, 4, 5, 8, 6, 7, 1, 10};
    test(c, 11);
}
