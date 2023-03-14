//
// Copyright 2021-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_MATERIAL_MATERIAL_H_
#define CODELIBRARY_WORLD_MATERIAL_MATERIAL_H_

#include "codelibrary/world/texture/texture_factory.h"

namespace cl {
namespace world {

/**
 * Base material.
 *
 * A material defines the visual appearance of a object.
 */
struct Material {
    // Name of this material.
    std::string name;

    // Use vertex color or albedo.
    bool use_vertex_color = false;

    // Base color with no light information.
    RGB32Color albedo;

    // Albedo texture.
    gl::Texture* albedo_map = nullptr;

    // Set the amount of metal in the microsurface.
    // 1.0 is metal and 0.0 is non metal. Values in between indicate surfaces
    // that cover raw metal, such as dirt.
    float metallic = 0.0f;

    // Metallic texture.
    gl::Texture* metallic_map = nullptr;

    // Describe the microsurface roughness of the object.
    // 1.0 is rough and 0.0 is smooth. Rough microsurface causes the light rays
    // to scatter broadly and appear dimmer.
    float roughness = 0.0f;

    // Roughness texture.
    gl::Texture* roughness_map = nullptr;

    // Define how much of the ambient light is accessible to a surface point.
    // It is a per-pixel shadowing factor between 0.0 (fully shadowed) and 1.0
    // (fully lit).
    float ao = 1.0f;

    // Metallic texture.
    gl::Texture* ao_map = nullptr;

    // Normal map.
    gl::Texture* normal_map = nullptr;

    explicit Material(const std::string& name = "")
        : name(name) {}

    /**
     * Load mesh material from the file.
     */
    void Load(const std::string& name) {
        auto texture_factory = TextureFactory::GetInstance();
        albedo_map = texture_factory->Create(name + "_albedo.png");
        if (!albedo_map) {
            albedo_map = texture_factory->Create(name + "_basecolor.png");
        }
        ao_map        = texture_factory->Create(name + "_ao.png");
        metallic_map  = texture_factory->Create(name + "_metallic.png");
        normal_map    = texture_factory->Create(name + "_normal.png");
        roughness_map = texture_factory->Create(name + "_roughness.png");
    }
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_MATERIAL_MATERIAL_H_
