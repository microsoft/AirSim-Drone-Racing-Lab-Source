#ifndef sdf_utils_TIMING_HPP
#define sdf_utils_TIMING_HPP

/*
 *  timing.hpp includes utilities for recording of execution time
 *
 *  Stopwatch wraps chrono::steady_clock. A global stopwatch is provided.
 * 
 *  Profiler is a singleton which allows storage of many timers. 
 *  It is not indended that the Profiler class be used directly. 
 *  Instead use the provided macros.
 *  Profiling can be enabled/disabled using #define ENABLE_PROFILING
 * 
 *  example:
 * 
 *  #define ENABLE_PROFILING
 *  #include timing.hpp
 *  PROFILE_REINITIALIZE(10,100);
 *  ...
 *  PROFILE_START("foo do stuff");
 *  foo_do_stuff();
 *  PROFILE_RECORD("foo do stuff");
 *  ...
 *  PROFILE_PRINT_SUMMARY_FOR_SINGLE("foo do stuff");
 * 
 *
 */

#include <chrono>
#include <string>
#include <map>
#include <vector>

/**
 *   .======================.
 *  ||   Profiling Macros   ||
 *   '======================'
 */


/* Clears all stored data and preallocates space for later recordings*/
#define PROFILE_REINITIALIZE(prealloc_num_names, prealloc_num_events)        \
    ::sdf_utils::Profiler::reset_and_preallocate(prealloc_num_names, prealloc_num_events);

/* Clears the data for a single stored name */
#define PROFILE_RESET(name) \
    ::sdf_utils::Profiler::reset(name)

/* Starts (or restarts) the specified stopwatch */
#define PROFILE_START(name) \
    ::sdf_utils::Profiler::startTimer(name)

/* Record the value of the specified timer. Does not stop or restart the timer */
#define PROFILE_RECORD(name) \
    ::sdf_utils::Profiler::record(name)

/* Record the time of timers and value. Does not stop or restart the timer */
#define PROFILE_RECORD_DOUBLE(name, value)            \
    ::sdf_utils::Profiler::recordDouble(name, value)

/* Print a summary of the data for name */
#define PROFILE_PRINT_SUMMARY_FOR_SINGLE(name) \
    ::sdf_utils::Profiler::printSingleSummary(name)

/* Print a condensed summary for each of name in names */
#define PROFILE_PRINT_SUMMARY_FOR_GROUP(names) \
    ::sdf_utils::Profiler::printGroupSummary(names)

#define PROFILE_PRINT_SUMMARY_FOR_ALL()                \
    ::sdf_utils::Profiler::printAllSummary()

#define PROFILE_WRITE_SUMMARY_FOR_GROUP(filename, names)                 \
    ::sdf_utils::Profiler::writeGroupSummary(filename, names)

#define PROFILE_WRITE_SUMMARY_FOR_ALL(filename)                \
    ::sdf_utils::Profiler::writeAllSummary(filename)

/* Dumps all data to file */
#define PROFILE_WRITE_ALL(filename)              \
    ::sdf_utils::Profiler::writeAll(filename)

/*  Dumps data to file for each name that has fewer than count_limit instances */
#define PROFILE_WRITE_ALL_FEWER_THAN(filename, name_count_limit)         \
    ::sdf_utils::Profiler::writeAll(filename, name_count_limit)
    

namespace sdf_utils
{
    enum StopwatchControl {RESET, READ};

    class Stopwatch
    {
    public:
        Stopwatch()
            : start_time_(std::chrono::steady_clock::now())
        {}

        double operator() (const StopwatchControl control = READ)
        {
            const auto end_time = std::chrono::steady_clock::now();
            if (control == RESET)
            {
                start_time_ = end_time;
            }

            return std::chrono::duration<double>(end_time - start_time_).count();
        }

    private:
        std::chrono::steady_clock::time_point start_time_;
    };

    double GlobalStopwatch(const StopwatchControl control = READ);


    /* 
     *  Profiler: 
     *  Note - You should probably use the macros instead of calling the profiler directly
     */

    class Profiler
    {
    public:
        struct TimedDouble{
            TimedDouble(double t, double v) : time(t), value(v) {};
            double time;
            double value;
        };

    public:
        static Profiler* getInstance();

        /*
         *  Clears existing data and preallocates memory for data storage
         */
        static void reset_and_preallocate(size_t num_names, size_t num_events);

        static void reset(std::string name);

        static void addData(std::string name, double datum);

        static void startTimer(std::string timer_name);

        static double record(std::string timer_name);

        static double recordDouble(std::string timer_name, double datum);

        static std::vector<double> getData(std::string name);

        static void printSingleSummary(std::string name);
        
        static void printGroupSummary(const std::vector<std::string> &names);

        static void printAllSummary();

        static void writeAllSummary(const std::string &filename);
        
        static void writeGroupSummary(const std::string &filename,
                                      const std::vector<std::string> &names);

        static void writeAll(const std::string &filename,
                             size_t limit_per_name = std::numeric_limits<size_t>::max());

    protected:
        bool isTimerStarted(std::string timer_name);
        static std::vector<std::string> getAllNames();

    protected:
        std::map<std::string, std::vector<TimedDouble> > timed_double_data;
        std::map<std::string, std::vector<double> > data;
        std::map<std::string, Stopwatch> timers;
        std::vector<std::vector<double>> prealloc_buffer;
        
    private:
        static Profiler* m_instance;
        
    };
}

#endif // sdf_utils_TIMING_HPP
