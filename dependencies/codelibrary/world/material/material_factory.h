//
// Copyright 2021-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_MATERIAL_MATERIAL_FACTORY_H_
#define CODELIBRARY_WORLD_MATERIAL_MATERIAL_FACTORY_H_

#include <unordered_map>

#include "codelibrary/base/pool.h"
#include "codelibrary/world/node.h"
#include "codelibrary/world/material/material.h"
#include "codelibrary/world/texture/texture_factory.h"

namespace cl {
namespace world {

/**
 * Factory to create and manage the life of materials.
 */
class MaterialFactory {
    MaterialFactory() = default;

public:
    static MaterialFactory* GetInstance() {
        static MaterialFactory factory;
        return &factory;
    }

    /**
     * Create a standard material.
     *
     * Parameters:
     *  albedo    - define the perceived color of an object.
     *  metallic  - define whether the surface is a metallic (1.0) or a
     *              non-metallic (0.0) surface.
     *  roughness - control the perceived smoothness of the surface. When
     *              roughness is set to 0, the surface is perfectly smooth and
     *              highly glossy. The rougher a surface is, the “blurrier” the
     *              reflections are.
     *  ao        - define how much of the ambient light is accessible to a
     *              surface point. It is a per-pixel shadowing factor between
     *              0.0 (fully shadowed) and 1.0 (fully lit).
     *              This property only affects diffuse indirect lighting
     *              (image-based light and ambient light), not direct lights
     *              such as directional, point and spot lights, nor specular
     *              lighting.
     */
    Material* Create(const std::string& name,
                     const RGB32Color& albedo,
                     float metallic = 1.0f,
                     float roughness = 0.0f,
                     float ao = 1.0f) {
        return CreateMaterial(name, "Material", albedo, metallic, roughness,
                              ao);
    }
    Material* Create(const RGB32Color& albedo,
                     float metallic = 1.0f,
                     float roughness = 0.0f,
                     float ao = 1.0f) {
        return Create("", albedo, metallic, roughness, ao);
    }

protected:
    /**
     * Create a standard material.
     */
    Material* CreateMaterial(const std::string& name,
                             const std::string& type,
                             const RGB32Color& albedo,
                             float metallic,
                             float roughness,
                             float ao) {
        Material* material = materials_.Allocate();
        material->name = GetID(name, type);
        material->albedo = albedo;
        material->metallic = metallic;
        material->roughness = roughness;
        material->ao = ao;

        return material;
    }

    /**
     * Automatic create names for different types.
     */
    std::string GetID(const std::string& name, const std::string& type) {
        return name.empty() ? type + std::to_string(counts_[type]++) : name;
    }

    // Materials.
    Pool<Material> materials_;

    // Count for each object type.
    std::unordered_map<std::string, int> counts_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_MATERIAL_MATERIAL_FACTORY_H_
