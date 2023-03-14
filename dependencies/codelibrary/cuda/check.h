//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_CUDA_ERROR_MEMORY_H_
#define CODELIBRARY_CUDA_ERROR_MEMORY_H_

#include "codelibrary/base/log.h"

/**
 * Check the result of a cudaXXXXXX call.
 */
#define CUDA_CHECK(x)                                               \
    do {                                                            \
        cudaError_t result = x;                                     \
        CHECK(result == cudaSuccess) << cudaGetErrorString(result); \
    } while(0)

#endif // CODELIBRARY_CUDA_ERROR_MEMORY_H_
