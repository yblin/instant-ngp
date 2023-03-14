//
// Copyright 2018-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_BASE_TIMER_H_
#define CODELIBRARY_BASE_TIMER_H_

#include <string>
#include <chrono>

#include "codelibrary/base/log.h"
#include "codelibrary/base/message.h"

namespace cl {

/**
 * A timer for calculating user's process time.
 */
class Timer {
public:
    Timer() {
        Reset();
    }

    /**
     * Reset the timer.
     */
    void Reset() {
        elapsed_ = std::chrono::duration<double, std::ratio<1, 1>>::zero();
        if (running_) {
            started_ = std::chrono::steady_clock::now();
        }
    }

    /**
     * Timer start.
     */
    void Start() {
        running_ = true;
        started_ = std::chrono::steady_clock::now();
    }

    /**
     * Timer stop.
     */
    void Stop() {
        if (running_) {
            running_ = false;
            elapsed_ += (std::chrono::steady_clock::now() - started_);
        }
    }

    /**
     * Return the elapsed time in seconds.
     */
    double elapsed_seconds() const {
        return elapsed_.count() * 1e-9;
    }

    /**
     * Return the human readable elapsed time.
     */
    std::string elapsed_time() const {
        return ReadableTime(elapsed_seconds());
    }

    /**
     * Return the average time: elapsed_time / n_times.
     */
    std::string average_time(int n_times) {
        return ReadableTime(elapsed_seconds() / n_times);
    }

    /**
     * Convert a time in seconds to human readable string.
     */
    static std::string ReadableTime(double time) {
        if (time < 1e-6) {
            return std::to_string(static_cast<int>(time * 1e9)) + "ns";
        }

        if (time < 1e-3) {
            return std::to_string(static_cast<int>(time * 1e6)) + "us";
        }

        if (time < 1.0) {
            return std::to_string(static_cast<int>(time * 1e3)) + "ms";
        }

        Message msg;
        msg.set_precision(2);
        if (time < 60.0) {
            msg << time << "s";
            return msg.ToString();
        }

        time /= 60.0;
        if (time < 60.0) {
            msg << time << "min";
            return msg.ToString();
        }

        time /= 60.0;
        if (time < 24.0) {
            msg << time << "h";
            return msg.ToString();
        }

        time /= 24.0;
        if (time < 30.0) {
            msg << time << " days";
            return msg.ToString();
        }

        if (time < 365.2425) {
            msg << time / 30.43687 << " months";
            return msg.ToString();
        }

        msg << time/ 365.2425 << " years";
        return msg.ToString();
    }

private:
    // True if timer is running.
    bool running_ = false;

    // The elapsed time (in ns).
    std::chrono::duration<double, std::ratio<1, 1000000000>> elapsed_;

    // The time of started.
    std::chrono::steady_clock::time_point started_;
};

} // namespace cl

#endif // CODELIBRARY_BASE_TIMER_H_
