#ifndef FileInfo_h
#define FileInfo_h

#include <string>
using namespace std;

class FileInfo {
public:
    // Show file info in OSD dialog (dispatches by extension)
    static void viewInfo(const string& path);
};

#endif
