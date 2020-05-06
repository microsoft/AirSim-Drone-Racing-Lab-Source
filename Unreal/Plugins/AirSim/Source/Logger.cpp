#include "Logger.h"

void FileLogger::writeString(const std::string& str)
{
    try {
        if (log_file_handle_) {
            FString line_f = FString(str.c_str());
            log_file_handle_->Write((const uint8*)TCHAR_TO_ANSI(*line_f), line_f.Len());
        }
        else
            UAirBlueprintLib::LogMessageString("Attempt to write to log file when file was not opened", "", LogDebugLevel::Failure);
    }
    catch (std::exception& ex) {
        UAirBlueprintLib::LogMessageString(std::string("Write to log failed "), ex.what(), LogDebugLevel::Failure);
    }
}

void FileLogger::openFile(const std::string& filename)
{
    IPlatformFile& platform_file = FPlatformFileManager::Get().GetPlatformFile();
    log_file_handle_ = platform_file.OpenWrite(*FString(filename.c_str()));
}

void FileLogger::closeFile()
{
    if (log_file_handle_ != nullptr)
        delete log_file_handle_;

    log_file_handle_ = nullptr;
}

bool FileLogger::isLogFileOpen()
{
    return (log_file_handle_ != nullptr);
}