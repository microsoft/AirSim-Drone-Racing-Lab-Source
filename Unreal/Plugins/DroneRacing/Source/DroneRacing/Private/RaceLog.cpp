#include "RaceLog.h"
#include <string>
#include <functional>
#include "Runtime/Core/Public/Misc/Paths.h"
#include "Runtime/Core/Public/HAL/PlatformFilemanager.h"
#ifdef __linux__ 
	#include <experimental/filesystem>
#else
	#include <filesystem>
#endif

std::string RaceLog::LOG_TEXT;

std::ofstream *RaceLog::LOGGER = new std::ofstream();

void RaceLog::open(const std::string& file_name)
{
	FString dir = FPaths::Combine(FPaths::ProjectLogDir(), LOG_FOLDER_NAME);

	IPlatformFile& platformFile = FPlatformFileManager::Get().GetPlatformFile();
	if (!platformFile.DirectoryExists(*dir))
		platformFile.CreateDirectory(*dir);

	FString full_path = FPaths::Combine(dir, FString(file_name.c_str()));
	LOGGER->open(TCHAR_TO_UTF8(*full_path));
	LOG_TEXT = "";
}

void RaceLog::close()
{
	auto hash = hashStr(LOG_TEXT + "hash_salt55555");

	*LOGGER << "signature: " << hash;
	LOGGER->flush();
	LOG_TEXT = "";
	LOGGER->close();
}

void RaceLog::log(const std::string& log_text)
{
	*LOGGER << log_text;
	LOG_TEXT += log_text;
	LOGGER->flush();
}
RaceLog::~RaceLog()
{
	LOGGER->close();
}