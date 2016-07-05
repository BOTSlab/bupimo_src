#include "SingleConfig.h"
#include <cstdlib>

// Initialize static instance pointer.
SingleConfig* SingleConfig::instance = NULL;

SingleConfig::SingleConfig(bool inVerbose) :
    verbose(inVerbose)
{
    if (verbose)
        cout << "SingleConfig: Loading config.cfg from current directory..."
             << endl;

    try {
        config.readFile("config.cfg");
        defaultsOnly = false;
    } catch (...) {
        cout << "SingleConfig: config.cfg unavailable.  All param's will "
             << "use default values." << endl;
        defaultsOnly = true;
    }
}

SingleConfig* SingleConfig::getInstance(bool verbose) {
    if (instance == NULL)
        instance = new SingleConfig(verbose);
    return instance;
}

void SingleConfig::checkDefaultsOnly(const char* path) {
    if (defaultsOnly) {
        cout << "SingleConfig: config.cfg must at least contain value for '" 
             << path << "'" << endl;
        exit(0);
    }
}

bool SingleConfig::getBool(const char* path) {
    checkDefaultsOnly(path);
    bool value;
    if (!config.lookupValue(path, value)) {
        cout << "SingleConfig: problem looking up " << path << endl;
        exit(0);
    }

    if (verbose)
        cout << "SingleConfig: read \'" << path << "\' value: " << value 
             << endl;
    return value;
}

int SingleConfig::getInt(const char* path) {
    checkDefaultsOnly(path);
    int value;
    if (!config.lookupValue(path, value)) {
        cout << "SingleConfig: problem looking up " << path << endl;
        exit(0);
    }
    if (verbose)
        cout << "SingleConfig: read \'" << path << "\' value: " << value 
             << endl;
    return value;
}

float SingleConfig::getFloat(const char* path) {
    checkDefaultsOnly(path);
    float value;
    if (!config.lookupValue(path, value)) {
        cout << "SingleConfig: problem looking up " << path << endl;
        exit(0);
    }
    if (verbose)
        cout << "SingleConfig: read \'" << path << "\' value: " << value 
             << endl;
    return value;
}

double SingleConfig::getDouble(const char* path) {
    checkDefaultsOnly(path);
    double value;
    if (!config.lookupValue(path, value)) {
        cout << "SingleConfig: problem looking up " << path << endl;
        exit(0);
    }
    if (verbose)
        cout << "SingleConfig: read \'" << path << "\' value: " << value 
             << endl;
    return value;
}

string SingleConfig::getString(const char* path) {
    checkDefaultsOnly(path);
    string value;
    if (!config.lookupValue(path, value)) {
        cout << "SingleConfig: problem looking up " << path << endl;
        exit(0);
    }
    if (verbose)
        cout << "SingleConfig: read \'" << path << "\' value: " << value 
             << endl;
    return value;
}

bool SingleConfig::getBool(const char* path, bool defaultValue) {
    bool value;
    if (!config.lookupValue(path, value))
        return defaultValue;
    if (verbose)
        cout << "SingleConfig: read \'" << path << "\' value: " << value 
             << endl;
    return value;
}

int SingleConfig::getInt(const char* path, int defaultValue) {
    int value;
    if (!config.lookupValue(path, value))
        return defaultValue;
    if (verbose)
        cout << "SingleConfig: read \'" << path << "\' value: " << value 
             << endl;
    return value;
}

float SingleConfig::getFloat(const char* path, float defaultValue) {
    float value;
    if (!config.lookupValue(path, value))
        return defaultValue;
    if (verbose)
        cout << "SingleConfig: read \'" << path << "\' value: " << value 
             << endl;
    return value;
}

double SingleConfig::getDouble(const char* path, double defaultValue) {
    double value;
    if (!config.lookupValue(path, value))
        return defaultValue;
    if (verbose)
        cout << "SingleConfig: read \'" << path << "\' value: " << value 
             << endl;
    return value;
}

string SingleConfig::getString(const char* path, string defaultValue) {
    string value;
    if (!config.lookupValue(path, value))
        return defaultValue;
    if (verbose)
        cout << "SingleConfig: read \'" << path << "\' value: " << value 
             << endl;
    return value;
}
