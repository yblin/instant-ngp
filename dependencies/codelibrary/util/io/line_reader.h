//
// Copyright 2019-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_IO_LINE_READER_H_
#define CODELIBRARY_UTIL_IO_LINE_READER_H_

#include <cstdio>
#include <cstring>
#include <memory>

#include "codelibrary/base/log.h"

namespace cl {
namespace io {

/**
 * A class to efficiently read large files line by line.
 *
 * For VS complier, one should add _CRT_SECURE_NO_WARNINGS option.
 */
class LineReader {
    // The size of the buffer.
    // We assume that the line length is no longer than BUFFER_SIZE.
    static const int BUFFER_SIZE = 1 << 24;

public:
    LineReader() {}

    explicit LineReader(const std::string& filename) {
        Open(filename);
    }

    virtual ~LineReader() {
        if (file_) fclose(file_);
    }

    /**
     * Open file for reading.
     */
    bool Open(const std::string filename) {
        if (!OpenFile(filename)) return false;

        Initialize();
        return true;
    }

    /**
     * Close the file.
     */
    void Close() {
        if (file_) {
            fclose(file_);
            file_ = nullptr;
            data_begin_ = data_end_ = 0;
        }
    }

    /**
     * Return true if the file is open.
     */
    bool is_open() const {
        return file_ != nullptr;
    }

    /**
     * Read a line.
     */
    char* ReadLine() {
        if (data_begin_ == data_end_) {
            this->Close();
            return nullptr;
        }

        ++n_line_;

        if (data_begin_ >= BUFFER_SIZE) {
            data_begin_ -= BUFFER_SIZE;
            data_end_   -= BUFFER_SIZE;
            std::memcpy(buffer_.get(), buffer_.get() + BUFFER_SIZE,
                        BUFFER_SIZE);
            size_t n = std::fread(buffer_.get() + BUFFER_SIZE, 1, BUFFER_SIZE,
                                  file_);
            data_end_ += n;
            n_read_bytes_ += n;
        }

        size_t i = data_begin_;
        while (i < data_end_ && buffer_[i] != '\n') ++i;

        if (i - data_begin_ >= BUFFER_SIZE) {
            CHECK(false) << "Reading error in line " << n_line_ << "\n"
                         << "Line length limit exceeded: "
                         << i - data_begin_ << " vs " << BUFFER_SIZE << ".";
        }

        if (i != data_end_ && buffer_[i] == '\n') {
            buffer_[i] = '\0';
        } else {
            // We did not found '\n'.
            ++data_end_;
            buffer_[i] = '\0';
        }

        // Handle \r\n-line breaks.
        if (i != data_begin_ && buffer_[i - 1] == '\r') {
            buffer_[i - 1] = '\0';
        }

        char* result = buffer_.get() + data_begin_;
        data_begin_ = i + 1;
        return result;
    }

    /**
     * Return file handle.
     */
    FILE* file() {
        return file_;
    }

    /**
     * The current number of lines.
     */
    int n_line() const {
        return n_line_;
    }

    /**
     * The current number of bytes read.
     */
    int n_read_bytes() const {
        return n_read_bytes_;
    }

private:
    /**
     * Open file for reading.
     */
    bool OpenFile(const std::string& filename) {
        file_ = std::fopen(filename.c_str(), "rb");

        if (!file_) {
            LOG(INFO) << "Cannot open file '" << filename << "' for reading.";
            return false;
        }

        return true;
    }

    /**
     * Initialize.
     */
    void Initialize() {
        n_line_ = 0;
        data_begin_ = 0;
        if (!buffer_.get()) buffer_.reset(new char[2 * BUFFER_SIZE + 1]);
        data_end_ = std::fread(buffer_.get(), 1, 2 * BUFFER_SIZE, file_);
        n_read_bytes_ = 0;
    }

    // The handle of the file.
    FILE* file_ = nullptr;

    // Data buffer.
    std::unique_ptr<char[]> buffer_;

    // The current number of lines.
    int n_line_ = 0;

    // The current number of bytes read.
    size_t n_read_bytes_ = 0;

    size_t data_begin_ = 0;
    size_t data_end_ = 0;
};

} // namespace io
} // namespace cl

#endif // CODELIBRARY_UTIL_IO_LINE_READER_H_
