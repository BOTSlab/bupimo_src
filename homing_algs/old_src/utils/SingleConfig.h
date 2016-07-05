/**
 * A Singleton class wrapped around libconfig's Config class.  Useful for
 * instances when all configuration parameters for a body of code naturally
 * fits into one global config file.
 *
 * @author Andrew Vardy
 *
 * BUG?: In the Camera class if cameraType is not provided in config.cfg then
 * a segmentation fault is generated.
 */
#include <libconfig.h++>
#include <cassert>
#include <iostream>
using namespace std;
using namespace libconfig;

class SingleConfig {
private:
    /**
     * Private constructor prevents direct instantiation.
     */
    SingleConfig(bool inVerbose);

public:
    /**
     * Obtain a pointer to the current instance if it exists.  If it doesn't
     * exist, create it and return a pointer to it.
     */
    static SingleConfig* getInstance(bool verbose = false);

    bool getBool(const char* path);

    int getInt(const char* path);
    
    float getFloat(const char* path);
    
    double getDouble(const char* path);
    
    string getString(const char* path);

    bool getBool(const char* path, bool defaultValue);

    int getInt(const char* path, int defaultValue);
    
    float getFloat(const char* path, float defaultValue);
    
    double getDouble(const char* path, double defaultValue);
    
    string getString(const char* path, string defaultValue);
    
private:
    void checkDefaultsOnly(const char* path);

    // The instance.
    static SingleConfig *instance;

    bool verbose, defaultsOnly;

    // The config object associated with this instance.
    Config config;
};
