//
// Copyright 2020-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPENGL_SHADER_SHADER_H_
#define CODELIBRARY_OPENGL_SHADER_SHADER_H_

#include <string>

#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/opengl/glad.h"
#include "codelibrary/opengl/transform.h"
#include "codelibrary/util/color/rgb32_color.h"

namespace cl {
namespace gl {

/**
 * GLSL Shader.
 */
class Shader {
public:
    Shader() = default;

    /**
     * Load shader from the memory.
     */
    Shader(const std::string& vertex_shader,
           const std::string& fragment_shader)
        : vertex_shader_(vertex_shader), fragment_shader_(fragment_shader) {}

    Shader(const std::string& vertex_shader,
           const std::string& fragment_shader,
           const std::string& geometry_shader)
        : vertex_shader_(vertex_shader),
          fragment_shader_(fragment_shader),
          geometry_shader_(geometry_shader) {}

    virtual ~Shader() {
        if (program_id_) {
            glDeleteProgram(program_id_);
            program_id_ = 0;
        }
    }

    /**
     * Trim the input cstring shader.
     */
    static std::string Trim(const char* str) {
        // Erase the extra spaces.
        bool add_space = true;
        std::string shader;
        for (int i = 0; str[i]; ++i) {
            if (str[i] == ' ' && add_space) {
                shader += str[i];
            } else if (str[i] != ' ') {
                shader += str[i];
            }

            if (str[i] == ';' || str[i] == '{' || str[i] == '}') {
                add_space = false;
            } else if (str[i] != ' ') {
                add_space = true;
            }
        }

        // Add required '\n'.
        std::string res;
        int spacing = 0;
        for (size_t i = 0; i < shader.length(); ++i) {
            res += shader[i];

            if (shader[i] == '{') spacing += 4;
            if (shader[i + 1] == '}') spacing -= 4;

            if (shader[i] == ';' || shader[i] == '{' || shader[i] == '}') {
                res += '\n';
                for (int j = 0; j < spacing; ++j) {
                    res += ' ';
                }
            }
        }
        return res;
    }

    void set_vertex_shader(const std::string& shader) {
        vertex_shader_ = shader;
    }

    void set_fragment_shader(const std::string& shader) {
        fragment_shader_ = shader;
    }

    void set_geometry_shader(const std::string& shader) {
        geometry_shader_ = shader;
    }

    /**
     * Compile the shaders.
     */
    bool Compile() {
        // Vertex shader.
        GLuint v_shader_id = glCreateShader(GL_VERTEX_SHADER);
        const char* v_str = vertex_shader_.c_str();
        glShaderSource(v_shader_id, 1, &v_str, NULL);
        glCompileShader(v_shader_id);
        if (!CheckCompileErrors(v_shader_id, "vertex")) {
            glDeleteShader(v_shader_id);
            return false;
        }

        // Fragment Shader.
        GLuint f_shader_id = glCreateShader(GL_FRAGMENT_SHADER);
        const char* f_str = fragment_shader_.c_str();
        glShaderSource(f_shader_id, 1, &f_str, NULL);
        glCompileShader(f_shader_id);
        if (!CheckCompileErrors(f_shader_id, "fragment")) {
            glDeleteShader(f_shader_id);
            return false;
        }

        GLuint g_shader_id = 0;
        if (!geometry_shader_.empty()) {
            g_shader_id = glCreateShader(GL_GEOMETRY_SHADER);
            const char* g_str = geometry_shader_.c_str();
            glShaderSource(g_shader_id, 1, &g_str, NULL);
            glCompileShader(g_shader_id);
            if (!CheckCompileErrors(g_shader_id, "geometry")) {
                glDeleteShader(g_shader_id);
                return false;
            }
        }

        // Shader program.
        program_id_ = glCreateProgram();
        glAttachShader(program_id_, v_shader_id);
        glAttachShader(program_id_, f_shader_id);
        if (g_shader_id) glAttachShader(program_id_, g_shader_id);
        glLinkProgram(program_id_);
        if (!CheckCompileErrors(program_id_, "program")) return false;

        // Delete the shaders as they're linked into our program now and no
        // longer necessary.
        glDeleteShader(v_shader_id);
        glDeleteShader(f_shader_id);
        if (g_shader_id) glDeleteShader(g_shader_id);
        return true;
    }

    /**
     * Activate the shader
     */
    void Use() const {
        glUseProgram(program_id_);
    }

    /**
     * Set unifrom parameters.
     */
    void SetUniform(int location, bool value) const {
        glUniform1i(location, value);
    }
    void SetUniform(int location, int value) const {
        glUniform1i(location, value);
    }
    void SetUniform(int location, unsigned int value) const {
        glUniform1ui(location, value);
    }
    void SetUniform(int location, float value) const {
        glUniform1f(location, value);
    }
    void SetUniform(int location, const FVector2D& value) const {
        glUniform2f(location, value.x, value.y);
    }
    void SetUniform(int location, const FVector3D& value) const {
        glUniform3f(location, value.x, value.y, value.z);
    }
    void SetUniform(int location, const FPoint3D& value) const {
        glUniform3f(location, value.x, value.y, value.z);
    }
    void SetUniform(int location, const RGB32Color& color) const {
        glUniform4f(location, color.red() / 255.0f,
                    color.green() / 255.0f, color.blue() / 255.0f,
                    color.alpha() / 255.0f);
    }
    void SetUniform(int location, const Transform& value) const {
        glUniformMatrix4fv(location, 1, GL_FALSE, value.data());
    }
    template <typename T>
    void SetUniform(const std::string& name, const T& value) const {
        SetUniform(GetLocation(name), value);
    }

    /**
     * Set the i-th element in uniform array.
     */
    template <typename T>
    void SetUniform(const std::string& name, int i, const T& value) const {
        SetUniform(GetLocation(name + "[" + std::to_string(i) + "]"), value);
    }

    /**
     * Get the location of uniform parameter 'name'.
     */
    int GetLocation(const std::string& name) const {
        return glGetUniformLocation(program_id_, name.c_str());
    }

    /**
     * Get the location of the attribute.
     */
    int GetAttributeLocation(const std::string& name) const {
        return glGetAttribLocation(program_id_, name.c_str());
    }

    const std::string& vertex_shader() const {
        return vertex_shader_;
    }

    const std::string& fragment_shader() const {
        return fragment_shader_;
    }

    GLuint program_id() const {
        return program_id_;
    }

protected:
    /**
     * Utility function for checking shader compilation/linking errors.
     */
    static bool CheckCompileErrors(GLuint shader, const std::string& type) {
        int success;
        char info[1024];

        if (type != "program") {
            glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
            if (!success) {
                GLint log_length = 0;
                glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_length);
                glGetShaderInfoLog(shader, log_length, NULL, info);
                LOG(INFO) << "SHADER_COMPILATION_ERROR of " << type << ":";
                LOG(INFO) << info;
                return false;
            }
        } else {
            glGetProgramiv(shader, GL_LINK_STATUS, &success);
            if (!success) {
                GLint log_length = 0;
                glGetProgramiv(shader, GL_INFO_LOG_LENGTH, &log_length);
                glGetProgramInfoLog(shader, log_length, NULL, info);
                LOG(INFO) << "PROGRAM_LINKING_ERROR of " << type << ":";
                LOG(INFO) << info;
                return false;
            }
        }

        return true;
    }

    GLuint program_id_ = 0;
    std::string vertex_shader_, fragment_shader_, geometry_shader_;
};

} // namespace gl
} // namespace cl

// Macro for easy shader definition.
#define GLSL_SOURCE(str) \
    cl::gl::Shader::Trim((const char*)"#version 460\n" # str)

#endif // CODELIBRARY_OPENGL_SHADER_SHADER_H_
