#define _CRT_SECURE_NO_WARNINGS
#include "ExLog.hpp"
#include <iostream>
#include <string>
#include <iomanip>

#include <thread>
#include <mutex>
#include <queue>


namespace ExLog {

    constexpr LogLevel getDefaultLogLevel() {
#if defined(_DEBUG) || defined(DEBUG)
        return LogLevel::Debug;
#else
        return LogLevel::Error;
#endif // DEBUG || _DEBUG
        return LogLevel::Debug;
    }

    LogLevel Logger::logLevel = getDefaultLogLevel();
    int Logger::fileNameWidth = 16;
    int Logger::lineWidth = 4;


    const LogConfig CriticalConfig {
        LogLevel::Critical,
        {  0xFC, 0x60, 0xA8 },
        { &std::cout, &std::cerr },
        ";5",
        ";5",
        "[C]"
    };

    const LogConfig ErrorConfig {
        LogLevel::Error,
        { 0xFF, 0x59, 0x66 },
        { &std::cout, &std::cerr },
        "",
        "",
        "[E]"
    };

    const LogConfig WarningConfig {
        LogLevel::Warning,
        { 0xF1, 0x9A, 0x31 },
        { &std::cout },
        "",
        "",
        "[W]"

    };

    const LogConfig InfoConfig {
        LogLevel::Info,
        { 0x5D, 0xD9, 0xC1 },
        { &std::cout },
        "",
        "",
        "[I]"
    };

    const LogConfig DebugConfig{
        LogLevel::Debug,
        { 0x9D, 0xAC, 0xFF },
        { &std::cout },
        "",
        "",
        "[D]"
    };

    const LogConfig VerboseConfig{
        LogLevel::Verbose,
        { 0x80, 0x80, 0x80 },
        { &std::cout },
        "",
        "",
        "[V]"
    };


    struct LogMessage {
        const std::vector<std::ostream *> &targetStream;
        const std::string message;

        inline LogMessage(const std::vector<std::ostream *> &target, const std::string &&message)
            :targetStream(target), message(message) {}
    };

    std::queue<LogMessage> logMessageQueue;
    std::mutex logMessageQueueMutex;
    std::condition_variable logMessageQueueCv;
    
    std::thread logThread([]() {
        std::cout << "ExLog READY!" << std::endl;
        while (true) {
            std::unique_lock<std::mutex> lock(logMessageQueueMutex);
            logMessageQueueCv.wait(lock, []() { return !logMessageQueue.empty(); });
            LogMessage message = logMessageQueue.front();
            logMessageQueue.pop();
            lock.unlock();
            for (auto &out : message.targetStream) {
                *out << message.message;
            }
        }
        });


    std::string Logger::getStylePrefix(const std::tuple<std::uint8_t, std::uint8_t, std::uint8_t> &foreground, const std::string &style)
    {
        return (std::stringstream()
            << "\033[38;2;"
            << (int)std::get<0>(foreground) << ';' << (int)std::get<1>(foreground) << ';' << (int)std::get<2>(foreground)
            << style
            << 'm'
            ).str();
    }

    std::string Logger::getStyleSuffix()
    {
        return std::string("\033[0m");
    }

    std::string Logger::getLevelPerfix(LogLevel level){
        switch (level) {
        case LogLevel::Critical: return "[C]";
        case LogLevel::Error: return "[E]";
        case LogLevel::Warning: return "[W]";
        case LogLevel::Info: return "[I]";
        case LogLevel::Debug:return "[D]";
        default: return "[?]";
        }
    }

    std::string Logger::formatFileName(const std::string &inp, int width)
    {
        std::string filename = inp;
        size_t pos;
        pos = filename.find_last_of('\\');
        if (pos != std::string::npos)
            filename = filename.substr(pos + 1);

        pos = filename.find_last_of('/');
        if (pos != std::string::npos)
            filename = filename.substr(pos + 1);

        pos = filename.find_last_of('.');
        if (pos != std::string::npos)
            filename = filename.substr(0, pos);
           
        if (filename.length() > width) {
            int headLength = (width - 2) / 2; 
            int tailLength = width - 2 - headLength; 
            std::string head = filename.substr(0, headLength);
            std::string tail = filename.substr(filename.length() - tailLength);
            filename = head + ".." + tail;
        }
        if (filename.length() < width) {
            filename = std::string(width - filename.length(), ' ') + filename;
        }
        return filename;
    }

    std::string Logger::formatLineNumber(int line, int width)
    {
        std::string res = std::to_string(line);
        res.resize(width, ' ');
        return ':' + res;
    }

    std::string Logger::formatTime(std::tm &time)
    {
        return (std::ostringstream()
            << std::setfill('0') << std::setw(2) << (time.tm_year % 100) << '/'
            << std::setfill('0') << std::setw(2) << time.tm_mon << '/'
            << std::setfill('0') << std::setw(2) << time.tm_mday << '-'
            << std::setfill('0') << std::setw(2) << time.tm_hour << ':'
            << std::setfill('0') << std::setw(2) << time.tm_min << ':'
            << std::setfill('0') << std::setw(2) << time.tm_sec
            ).str();
    }

    Logger::Logger(const LogConfig &config, const std::string& filename, int line)
    :config(config), std::ostream(shouldLog(config)?&buf:nullptr){
        if (!shouldLog(config)) return;
        std::time_t rawTime = std::time(nullptr);
        std::tm time = *std::localtime(&rawTime);
        *this
            << formatTime(time)
            << ' '
            << formatFileName(filename, fileNameWidth)
            << formatLineNumber(line, lineWidth)
            << ' '
            << getStylePrefix(config.color, ";1" + config.levelExtraStyle)
            << config.levelPrefix
            << getStyleSuffix()
            << ' '
            << getStylePrefix(config.color, "" + config.messageExtraStyle);
    }

    Logger::~Logger() {
        if (!shouldLog(config)) return;
        *this << getStyleSuffix() << "\r\n";
        {
            std::lock_guard<std::mutex> lock(logMessageQueueMutex);
            logMessageQueue.push(LogMessage(config.targetStream, std::move(buf.str())));
        }
        logMessageQueueCv.notify_one();
    }

    bool Logger::shouldLog(const LogConfig &config){
        return static_cast<std::uint8_t>(config.level) <= static_cast<std::uint8_t>(logLevel);
    }

    void Logger::setFileNameWidth(int width){
        fileNameWidth = width;

    }

    void Logger::setLineWidth(int width){
        lineWidth = width;
    }

    void Logger::setLogLevel(LogLevel level){
        logLevel = level;
    }

}