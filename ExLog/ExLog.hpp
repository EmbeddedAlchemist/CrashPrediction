#pragma once

#include <ostream>
#include <sstream>
#include <string>
#include <iosfwd>
#include <vector>
#include <tuple>
#include <cstdint>
#include <ctime>


namespace ExLog {


    enum class LogLevel :std::uint8_t {
        Critical = 0,
        Error = 1,
        Warning = 2,
        Info = 3,
        Debug = 4,
        Verbose = 5
    };

    struct LogConfig {
        LogLevel level;
        std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>color;
        std::vector<std::ostream *> targetStream;
        std::string levelExtraStyle;
        std::string messageExtraStyle;
        std::string levelPrefix;
    };

    extern const LogConfig CriticalConfig;
    extern const LogConfig ErrorConfig;
    extern const LogConfig WarningConfig;
    extern const LogConfig InfoConfig;
    extern const LogConfig DebugConfig;
    extern const LogConfig VerboseConfig;



    class Logger: public std::ostream {
    private:
        std::stringbuf buf;
        const LogConfig &config;
        static LogLevel logLevel;
        static int fileNameWidth;
        static int lineWidth;

    private:
        static std::string getStylePrefix(
            const std::tuple<std::uint8_t, std::uint8_t, std::uint8_t> &foreground,
            const std::string &style);
        static std::string getStyleSuffix();
        static std::string getLevelPerfix(LogLevel level);
        static std::string formatFileName(const std::string &filename, int width);
        static std::string formatLineNumber(int line, int width);
        static std::string formatTime(std::tm &time);
        static bool shouldLog(const LogConfig &config);
 
    public:
        Logger(const LogConfig &config, const std::string &filename, int line);
        ~Logger();
        static void setLogLevel(LogLevel level);
        static void setFileNameWidth(int width);
        static void setLineWidth(int width);
    };
}


#define LOG(CONFIG) (ExLog::Logger(CONFIG, __FILE__, __LINE__))
#define LOG_C LOG(ExLog::CriticalConfig)
#define LOG_E LOG(ExLog::ErrorConfig)
#define LOG_W LOG(ExLog::WarningConfig)
#define LOG_I LOG(ExLog::InfoConfig)
#define LOG_D LOG(ExLog::DebugConfig)
#define LOG_V LOG(ExLog::VerboseConfig)
