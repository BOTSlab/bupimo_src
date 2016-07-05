#include "CursesListener.h"

CursesListener::CursesListener() {
    // ncurses initialization
    initscr();
    noecho();
    cbreak();
    timeout(0);
    keypad(stdscr, true);

    printRow = 1;
    intStr = "";
}

CursesListener::~CursesListener() {
    endwin();
}

int CursesListener::checkKey() {
    // Now read the key pressed (if any) and check if its the one required.
    int key = getch();
    flushinp();
    return key;
}

bool CursesListener::checkForInteger(int &result) {
    // The return value of this function.
    bool retVal = false;

    // Now read the key pressed (if any).  If it is anything but a newline,
    // place it into intStr.
    int ch = getch();

    if (ch != -1 && ch != '\n')
        intStr += ch;
    else if (ch == '\n') {
        istringstream istr(intStr);
        istr >> result;
        retVal = true;
        intStr = "";
    }

    return retVal;
}

void CursesListener::printHeader(string msg) {
    clear();
    headerStr = msg;
    mvprintw(0, 0, headerStr.c_str());
}

void CursesListener::print(string msg) {
    mvprintw(printRow, 0, msg.c_str());
    printRow = (printRow + 1) % 24;
    if (printRow == 0) {
        printRow = 1;
        clear();
        mvprintw(0, 0, headerStr.c_str());
    }
}
