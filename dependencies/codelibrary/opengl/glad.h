//
// Copyright 2020-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPENGL_GLAD_H_
#define CODELIBRARY_OPENGL_GLAD_H_

#if __has_include("glad/glad.h")
#include "glad/glad.h"
#else
#include "third_party/glad/glad.h"
#endif

#include "codelibrary/base/log.h"

namespace cl {

static bool GL_LOADED = false;

/**
 * Load GL using GLAD library.
 *
 * Return true if loading succeeded.
 *
 * Why using GLAD?
 *   Modern desktop OpenGL doesn't have a standard portable header file to load
 *   OpenGL function pointers. Helper libraries are often used for this purpose!
 */
inline bool LoadGL() {
    if (GL_LOADED) return true;

    if (gladLoadGL() == 0) {
        LOG(INFO) << "Failed to initialize OpenGL loader!";
        return false;
    }
    GL_LOADED = true;
    return true;
}

/**
 * Return the version of current opengl.
 */
inline int GetGLVersion() {
    GLint major = 0;
    GLint minor = 0;
    glGetIntegerv(GL_MAJOR_VERSION, &major);
    glGetIntegerv(GL_MINOR_VERSION, &minor);
    return major * 100 + minor;
}

} // namespace cl

#endif // CODELIBRARY_OPENGL_GLAD_H_
