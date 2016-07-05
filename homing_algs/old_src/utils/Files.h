/*
 * Some common file-related methods.
 *
 * \author Andrew Vardy
 */
#ifndef FILES_H
#define FILES_H

#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

using namespace std;

class Files {
public:

    /**
     * Determines if the given file exists.
     *
     * Code taken from http://www.techbytes.ca/techbyte103.html
     */
    static bool fileExists(string strFilename) {
        struct stat stFileInfo;
        bool blnReturn;
        int intStat;

        // Attempt to get the file attributes
        intStat = stat(strFilename.c_str(),&stFileInfo);
        if(intStat == 0) {
            // We were able to get the file attributes
            // so the file obviously exists.
            blnReturn = true;
        } else {
            // We were not able to get the file attributes.
            // This may mean that we don't have permission to
            // access the folder which contains this file. If you
            // need to do that level of checking, lookup the
            // return values of stat which will give you
            // more details on why stat failed.
            blnReturn = false;
        }

        return (blnReturn);
    }
};

#endif
