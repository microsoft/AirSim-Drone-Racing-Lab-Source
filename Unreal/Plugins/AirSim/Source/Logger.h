#pragma once

#include "AirBlueprintLib.h"
#include "HAL/PlatformFilemanager.h"
#include "Misc/FileHelper.h"

using namespace msr::airlib;

class FileLogger
{
public:
	void writeString(const std::string& str);
	void openFile(const std::string& filename);
	void closeFile();

	bool isLogFileOpen();

private:
	IFileHandle* log_file_handle_ = nullptr;
};