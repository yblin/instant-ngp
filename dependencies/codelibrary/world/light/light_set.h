//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_LIGHT_LIGHT_SET_H_
#define CODELIBRARY_WORLD_LIGHT_LIGHT_SET_H_

#include <unordered_set>

#include "codelibrary/world/light/image_based_light.h"
#include "codelibrary/world/light/light.h"

namespace cl {
namespace world {

struct LightSet {
    // Maximal number of lights.
    static const int MAX_LIGHTS = 128;

    // All lights.
    Array<Light*> lights;

    // We only allow one directional light to generate shadow due to efficency
    // issue.
    Light* directional_light = nullptr;

    // Ambient light.
    // We only allow one ambient light.
    Light* ambient_light = nullptr;

    // We only allow one image based light.
    ImageBasedLight* image_based_light = nullptr;

    /**
     * Add a light.
     */
    bool Add(Node* node) {
        CHECK(node);

        Light* light = dynamic_cast<Light*>(node);
        if (light) {
            if (lights.size() >= MAX_LIGHTS) {
                LOG(INFO) << "The number of lights exceeds the limit.";
                return false;
            }
            lights.push_back(light);

            if (light->type() == Light::AMBIENT) {
                ambient_light = light;
            } else if (light->type() == Light::DIRECTIONAL) {
                directional_light = light;
            }
            return true;
        }

        ImageBasedLight* ibl = dynamic_cast<ImageBasedLight*>(node);
        if (ibl) {
            image_based_light = ibl;
            return true;
        }

        return false;
    }

    /**
     * Clear all lights.
     */
    void clear() {
        lights.clear();
        directional_light = nullptr;
        image_based_light = nullptr;
        ambient_light = nullptr;
    }
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_LIGHT_LIGHT_SET_H_
