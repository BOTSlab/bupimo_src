/*
 * Provides non-blocking methods 'checkForKey' and 'checkForInteger' to check
 * if the user has provided the requested input.  Utilizes the ncurses library
 * for this purpose.
 *
 * Andrew Vardy
 */

#ifndef CURSESLISTENER_H
#define CURSESLISTENER_H

#include <iostream>
#include <sstream>
#include <ncurses.h>
using namespace std;

#define KEY_ESC 27 // Not defined in "ncurses.h" for some reason.

class CursesListener {
public:
    /**
     * Constructor.  Initializes ncurses.
     */
    CursesListener();

    /**
     * Destructor.  Ends ncurses input mode.
     */
    ~CursesListener();

    /**
     * Check to see if the user has pressed the desired key.  If so, return
     * true, otherwise return false.
     */ 
    int checkKey();

    /**
     * Check to see if the user has entered an integer.  The result will remain
     * false until the user presses the enter key to indicate that data entry
     * is complete.  Only when the user has finally pressed enter will the
     * return value of this function be true.  At this point, 'result' will
     * be set to the entered integer.
     */
    bool checkForInteger(int &result);

    /**
     * Print a string in row 0.
     */
    void printHeader(string msg);

    /**
     * Print a string in row 1 or greater.  The row at which the string is
     * printed will increase (with wrap-around at 24).
     */
    void print(string msg);

private:
    // The string that characters read by 'checkForInteger' will be placed into.
    string intStr;

    string headerStr;

    int printRow;
};

#endif
