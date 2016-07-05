#include "SingleConfig.h"
#include <iostream>

int main() {
    SingleConfig* config = SingleConfig::getInstance(true);

    string version = config->getString("version");
    string missingString = config->getString("missingString", "default");

    double x = config->getDouble("x");
    double missingDouble = config->getDouble("missingDouble", -1.0);

    int missingInt = config->getInt("missingInt", 4);
    
    cout << "version: " << version << endl;
    cout << "missingString: " << missingString << endl;
    cout << "x: " << x << endl;
    cout << "missingDouble: " << missingDouble << endl;
    cout << "missingInt: " << missingInt << endl;
    return 0;
}
