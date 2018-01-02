#ifndef LOGGER_H
#define LOGGER_H

/*
* Logger.h
* Control log s
* Author: Binyamin Appelbaum
* Date: 27.11.17
* 
*/

#include <string>
#include <map>

enum LogLevel {_NONE_ = -1, _DEBUG_, _NORMAL_, _ERROR_, _ALWAYS_};

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__) // print only file name, without path
#define LOG(level, msg) Logger::GetInstance().PrintToLog(level, __FILENAME__, __func__, __LINE__, msg)

class Logger {
private:
    Logger(LogLevel screenLogLevel = _NORMAL_, LogLevel fileLogLevel = _NORMAL_);
    Logger(const Logger&) = default;
    ~Logger() = default;

    LogLevel m_tmpLevel;
    /**
     * screen log level for printing
     */ 
    LogLevel m_screenLogLevel;
    /**
     * file log level for printing
     */ 
    LogLevel m_fileLogLevel;
    /**
     * log destination file name
     */ 
    std::string m_logFileName;
    /**
     * log directory
     */ 
    std::string m_logDirName;
    /**
     * Print message to log file
     * @param message - string of the desired message to print
     */ 
    void PrintToFile(LogLevel level, const std::string& message) const;
    /**
     * Print message to stdout (screen)
     * @param message - string of the desired message to print
     */ 
    void PrintToScreen(LogLevel level, const std::string& message) const;

    template <typename T>
    std::string MarkMessageWithColor(const T& message, const std::string& color) const;

    static const std::string DEF_LOG_DIR_NAME;

    static const std::map<LogLevel, std::string> m_logLevelToStr;


public:
    static Logger& GetInstance();
    /**
     * print message to log (file or screen)
     * @param level -log level of the message (debug/normal/etc)
     * @param sourceFile - the cpp file that the message came from
     * @param funcName - function name that the nessage came from
     * @param lineNumber - the line where the message came from
     * @param message - the message body to print,
     */ 
    void PrintToLog(LogLevel level, const std::string& sourceFile, const std::string& funcName, int lineNumber, const std::string& message) const;
};



#endif // LOGGER_H