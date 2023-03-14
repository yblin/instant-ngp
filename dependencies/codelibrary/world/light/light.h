//
// Copyright 2020-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_LIGHT_LIGHT_H_
#define CODELIBRARY_WORLD_LIGHT_LIGHT_H_

#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/world/node.h"
#include "codelibrary/world/light/image_based_light.h"

namespace cl {
namespace world {

/**
 * Light data.
 */
class Light : public NoLightingNode {
public:
    // Light type.
    enum Type {
        DIRECTIONAL = 0, // Directional light.
        POINT = 1,       // Point light.
        AMBIENT = 2      // Ambient light.
    };

    explicit Light(const std::string& name = "")
        : NoLightingNode(name) {}

    Light(const std::string& name, Type type, const FPoint3D& position,
          const FVector3D& direction, const FVector3D& radiance)
        : NoLightingNode(name),
          type_(type),
          position_(position),
          direction_(direction),
          radiance_(radiance) {}

    Type type()                  const { return type_;      }
    const FPoint3D& position()   const { return position_;  }
    const FVector3D& direction() const { return direction_; }
    const FVector3D& radiance()  const { return radiance_;  }

    void set_type(Type type) {
        type_ = type;
    }

    void set_position(const FPoint3D& position) {
        position_ = position;
    }

    void set_direction(const FVector3D& direction) {
        direction_ = direction;
    }

    void set_radiance(const FVector3D& radiance) {
        radiance_ = radiance;
    }

private:
    // Type of this light.
    Type type_;

    // Light position. Used for point light.
    FPoint3D  position_;

    // Light direction. Used for directional light.
    FVector3D direction_;

    // Light radiance.
    FVector3D radiance_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_LIGHT_LIGHT_H_
