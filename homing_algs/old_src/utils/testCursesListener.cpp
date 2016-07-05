#include "CursesListener.h"
#include "unistd.h"
#include <iostream>
#include <sstream>
using namespace std;

int main(int argc, char *argv[]) {

    int result;
    {
        CursesListener listener;

        listener.printHeader("Test for certain key presses......\n");
        while (true) {
            int key = listener.checkKey();

            if (key == KEY_UP    /*keyboard*/ || key == 'b'       /*presenter*/)
                listener.print("up\n");
            if (key == KEY_RIGHT /*keyboard*/ || key == KEY_NPAGE /*presenter*/)
                listener.print("right\n");
            if (key == KEY_LEFT  /*keyboard*/ || key == KEY_PPAGE /*presenter*/)
                listener.print("left\n");
            if (key >= 48 && key < 58) {
                ostringstream oss;
                oss << (key - 48);
                listener.print(oss.str().c_str());
            }
            if (key == KEY_ESC   /*keyboard*/ || key == KEY_F(5)  /*presenter*/)
            {
                listener.print("quitting\n");
                break;
            }
            usleep(100e3); // 100 milliseconds
        }

        listener.printHeader("Enter an integer...");
        while (!listener.checkForInteger(result)) 
            ;
    }
    cout << "result: " << result << endl;
}
