//
// Copyright 2021-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPENGL_SHADER_FACTORY_H_
#define CODELIBRARY_OPENGL_SHADER_FACTORY_H_

#include <map>

#include "codelibrary/base/pool.h"
#include "codelibrary/opengl/shader.h"

namespace cl {
namespace gl {

/**
 * A singleton global class to manage the shaders.
 */
class ShaderFactory {
    ShaderFactory() = default;

public:
    using Shader = gl::Shader;

    ShaderFactory(const ShaderFactory&) = delete;

    ShaderFactory& operator=(const ShaderFactory&) = delete;

    static ShaderFactory* GetInstance() {
        static ShaderFactory shader_factory;
        return &shader_factory;
    }

    /**
     * Create a specific shader.
     */
    Shader* Create(const std::string& vertex_shader,
                   const std::string& fragment_shader) {
        auto it = shader_map_.find(vertex_shader + fragment_shader);
        if (it != shader_map_.end()) return it->second;

        Shader* shader = shader_pool_.Allocate();
        shader->set_vertex_shader(vertex_shader);
        shader->set_fragment_shader(fragment_shader);
        bool success = shader->Compile();
        CHECK(success) << "Shader compile error.";

        shader_map_[vertex_shader + fragment_shader] = shader;
        return shader;
    }

    Shader* Create(const std::string& vertex_shader,
                   const std::string& fragment_shader,
                   const std::string& geometry_shader) {
        auto it = shader_map_.find(vertex_shader + fragment_shader +
                                   geometry_shader);
        if (it != shader_map_.end()) return it->second;

        Shader* shader = shader_pool_.Allocate();
        shader->set_vertex_shader(vertex_shader);
        shader->set_fragment_shader(fragment_shader);
        shader->set_geometry_shader(geometry_shader);
        bool success = shader->Compile();
        CHECK(success) << "Shader compile error.";

        shader_map_[vertex_shader + fragment_shader + geometry_shader] = shader;
        return shader;
    }

private:
    Pool<Shader> shader_pool_;
    std::map<std::string, Shader*> shader_map_;
};

} // namespace gl
} // namespace cl

#define GL_CREATE_SHADER(a, b) \
    cl::gl::ShaderFactory::GetInstance()->Create(a, b)

#define GL_CREATE_GEOMETRY_SHADER(a, b, c) \
    cl::gl::ShaderFactory::GetInstance()->Create(a, b, c)

#endif // CODELIBRARY_OPENGL_SHADER_FACTORY_H_
