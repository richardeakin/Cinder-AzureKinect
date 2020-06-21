// Adapted from k4aviewer

#pragma once

#include <array>
#include <vector>

#include "cinder/Filesystem.h"

namespace imx {

class FilePicker {
public:
    FilePicker();
    bool show();
    cinder::fs::path getPath() const    { return mSelectedPath; }

private:
    void changeWorkingDirectory( cinder::fs::path newDirectory );

    static constexpr size_t MaxPath = 4096;
    std::array<char, MaxPath> mCurrentDirectoryBuffer;
    cinder::fs::path mSelectedPath;

    std::vector<std::string> mCurrentDirectoryFiles;
    std::vector<std::string> mCurrentDirectorySubdirectories;

    bool mFilterExtensions = true;
};

} // namespace imx
