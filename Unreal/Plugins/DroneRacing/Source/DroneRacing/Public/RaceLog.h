#pragma once
#include <fstream>
class RaceLog
{
public:
	//RaceLog();
	~RaceLog();

	void open(const std::string& file_name);
	void log(const std::string& log_text);
	void close();

private:
	static std::ofstream* LOGGER;
	static std::string LOG_TEXT;
	static std::string LOG_TEXT2;
	const FString LOG_FOLDER_NAME = FString("RaceLogs");

	inline unsigned long long hashStr(const std::string& s)
	{
		unsigned long long result = 0xcbf29ce484222325;

		for (char c : s) {
			result ^= c;
			result *= 1099511628211; //prime
		}

		return result;
	}
};