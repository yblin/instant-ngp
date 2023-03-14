//
// Copyright 2016-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_BASE_LOG_H_
#define CODELIBRARY_BASE_LOG_H_

#ifdef USE_GLOG
#include <glog/logging.h>
#else

#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <ctime>
#include <memory>

#include "codelibrary/base/message.h"

namespace cl {

/**
 * Simple logging framework in a single file.
 *
 * Example Usage:
 *
 *    LOG_ON(INFO);
 *    LOG(INFO) << "This is test info log";
 */
class Logger {
    Logger() = default;

public:
    Logger(const Logger&) = delete;

    Logger& operator=(const Logger&) = delete;

    // The severity level for logger.
    enum Severity {
        LOG_NONE    = 0,
        LOG_FATAL   = 1,
        LOG_WRONG   = 2,
        LOG_WARNING = 3,
        LOG_INFO    = 4,
        LOG_DEBUG   = 5,
        LOG_VERBOSE = 6
    };

    /**
     * Store all log data.
     *
     * A record has the severity of logging, the position where the file and
     * line call the log.
     */
    struct Record {
        Record(const Severity& s, const char* f, int l)
            : severity(s), filename(f), line(l) {
            size_t t1 = filename.find_last_of('\\');
            size_t t2 = filename.find_last_of('/');
            if (t1 == std::string::npos) t1 = 0;
            if (t2 == std::string::npos) t2 = 0;
            if (t1 != 0 || t2 != 0)
                filename = filename.substr(std::max(t1, t2) + 1);

            time = std::chrono::system_clock::now();
        }

        Record(const Record&) = delete;

        Record& operator =(const Record&) = delete;

        // A semantic trick to enable message streaming.
        Record& operator +=(const Message& m) {
            message << m;

            return *this;
        }

        // Severity level for logging.
        const Severity severity;

        // Source code filename.
        std::string filename;

        // Source code line.
        const int line;

        // Message of record.
        Message message;

        // Record the time when LOG() took place.
        std::chrono::time_point<std::chrono::system_clock> time;
    };

    /**
     * Get the singleton Logger object.
     *
     * The first time this method is called, a Logger object is constructed and
     * returned.
     * Consecutive calls will return the same object.
     */
    static Logger* GetInstance() {
        static Logger instance;
        return &instance;
    }

    /**
     * A semantic trick to enable logging streaming.
     */
    void operator +=(const Record& record) {
        // Print severity.
        printf("%s", SeverityToString(record.severity));

        // Print date time.
        std::time_t tt = std::chrono::system_clock::to_time_t(record.time);
        std::tm* tm = localtime(&tt);
        printf("%02d%02d %02d:%02d:%02d", tm->tm_mon + 1, tm->tm_mday,
               tm->tm_hour, tm->tm_min, tm->tm_sec);

        auto since_epoch = record.time.time_since_epoch();
        std::chrono::seconds s =
                std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
        since_epoch -= s;

        // Print milliseconds.
        using Milliseconds = std::chrono::milliseconds;
        Milliseconds milliseconds =
                std::chrono::duration_cast<Milliseconds>(since_epoch);
        printf(".%03" PRId64 "", milliseconds.count());

        // Print message.
        if (!record.message.IsMultiLine()) {
            printf(" %s:%d] %s\n", record.filename.c_str(), record.line,
                                   record.message.ToString().c_str());
        } else {
            printf(" %s:%d]\n", record.filename.c_str(), record.line);
            printf("%s\n", record.message.ToString().c_str());
        }

        fflush(stdout);

        if (record.severity == LOG_FATAL) exit(1);
    }

    /**
     * Check if severity is valid.
     */
    bool CheckSeverity(const Severity& severity) const {
        return severity <= severity_level_;
    }

    void set_severity_level(const Severity& severity_level) {
        severity_level_ = severity_level;
    }

    const Severity& severity_level() const {
        return severity_level_;
    }

private:
    /**
     * Convert severity to string.
     */
    static const char* SeverityToString(Severity severity) {
        switch (severity) {
        case LOG_FATAL:
            return "F";
        case LOG_WRONG:
            return "E";
        case LOG_WARNING:
            return "W";
        case LOG_INFO:
            return "I";
        case LOG_DEBUG:
            return "D";
        case LOG_VERBOSE:
            return "V";
        default:
            return "N";
        }
    }

    /**
     * The logger severity upper limit.
     *
     * All log messages have its own severity and if it is higher than the limit
     * those messages are dropped.
     */
    Severity severity_level_ = LOG_FATAL;
};

} // namespace cl

// Enable the namespace for log severity.
#define LOG_SEVERITY(severity) cl::Logger::LOG_##severity

/**
 * Main logging macros.
 *
 * Example usage:
 *
 *   LOG(INFO) << "This is a info log";
 */
#ifdef LOG
#undef LOG
#endif
#define LOG(severity) \
    !(cl::Logger::GetInstance()->CheckSeverity(LOG_SEVERITY(severity))) ? \
        (void) 0 : (*cl::Logger::GetInstance()) += \
            cl::Logger::Record(LOG_SEVERITY(severity), __FILE__, __LINE__) += \
            cl::Message() << cl::Message()

#ifdef LOG_IF
#undef LOG_IF
#endif
#define LOG_IF(severity, expression) \
    !(expression) ? (void) 0 : LOG(severity)

/**
 * Turn on the log and set the severity level.
 *
 * Example usage:
 *
 *    LOG_ON(INFO);
 */
#define LOG_ON(severity) \
    cl::Logger::GetInstance()->set_severity_level(LOG_SEVERITY(severity))

/**
 * CHECK dies with a fatal error if condition is not true.
 */
#ifdef CHECK
#undef CHECK
#endif
#define CHECK(condition) \
    LOG_IF(FATAL, !(condition)) << "Check failed: " #condition ". "

#endif // USE_GLOG

#endif // CODELIBRARY_BASE_LOG_H_
