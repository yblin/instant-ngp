//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_LIGHT_LIGHT_FACTORY_H_
#define CODELIBRARY_WORLD_LIGHT_LIGHT_FACTORY_H_

#include <unordered_map>

#include "codelibrary/base/pool.h"
#include "codelibrary/world/node.h"
#include "codelibrary/world/light/light.h"

namespace cl {
namespace world {

/**
 * Factory to create and manage the life of lights.
 */
class LightFactory {
    LightFactory() = default;

public:
    static LightFactory* GetInstance() {
        static LightFactory factory;
        return &factory;
    }

    /**
     * Create a directional light.
     */
    Light* CreateDirectionalLight(const std::string& name,
                                  const FVector3D& direction,
                                  const FVector3D& radiance) {
        return CreateLight(name,
                           "DirectionalLight",
                           Light::DIRECTIONAL,
                           FPoint3D(0.0f, 0.0f, 0.0f),
                           direction,
                           radiance);
    }
    Light* CreateDirectionalLight(const FVector3D& direction,
                                  const FVector3D& radiance) {
        return CreateDirectionalLight("", direction, radiance);
    }

    /**
     * Create a point light.
     */
    Light* CreatePointLight(const std::string& name,
                            const FPoint3D& position,
                            const FVector3D& radiance) {
        return CreateLight(name,
                           "PointLight",
                           Light::POINT,
                           position,
                           FVector3D(0.0f, 0.0f, 0.0f),
                           radiance);
    }
    Light* CreatePointLight(const FPoint3D& position,
                            const FVector3D& radiance) {
        return CreatePointLight("", position, radiance);
    }

    /**
     * Create an ambient light.
     */
    Light* CreateAmbinetLight(const std::string& name,
                              const FVector3D& radiance) {
        return CreateLight(name,
                           "AmbientLight",
                           Light::AMBIENT,
                           FPoint3D(0.0f, 0.0f, 0.0f),
                           FVector3D(0.0f, 0.0f, 0.0f),
                           radiance);
    }
    Light* CreateAmbinetLight(const FVector3D& radiance) {
        return CreateAmbinetLight("", radiance);
    }

protected:
    /**
     * Create a light.
     */
    Light* CreateLight(const std::string& name,
                       const std::string& type,
                       Light::Type light_type,
                       const FPoint3D& position,
                       const FVector3D& direction,
                       const FVector3D& radiance) {
        Light* light = lights_.Allocate();
        light->set_name(GetID(name, type));
        light->set_type(light_type);
        light->set_position(position);
        light->set_direction(direction);
        light->set_radiance(radiance);

        return light;
    }

    /**
     * Automatic create names for different types.
     */
    std::string GetID(const std::string& name, const std::string& type) {
        return name.empty() ? type + std::to_string(counts_[type]++) : name;
    }

    // Lights.
    Pool<Light> lights_;

    // Count for each light type.
    std::unordered_map<std::string, int> counts_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_MATERIAL_MATERIAL_FACTORY_H_
