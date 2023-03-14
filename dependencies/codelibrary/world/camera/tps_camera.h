//
// Copyright 2021-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_CAMERA_TPS_CAMERA_H_
#define CODELIBRARY_WORLD_CAMERA_TPS_CAMERA_H_

#include <algorithm>

#include "codelibrary/base/clamp.h"
#include "codelibrary/opengl/camera.h"
#include "codelibrary/ui/ui.h"

namespace cl {
namespace world {

/**
 * A third-person styled camera for 3D world.
 */
class TPSCamera : public gl::Camera {
public:
    /**
     * Note that we always fix the up direction to +Z axis.
     */
    TPSCamera(const FPoint3D& position, const FPoint3D& target)
        : Camera(position, target, FVector3D(0.0f, 0.0f, 1.0f)) {
        CHECK(position != target);

        z_near_ = 0.1f;
        z_far_ = 1000.0f;

        FVector3D v = position_ - target_;
        distance_ = static_cast<float>(Clamp(Distance(position, target),
                                             double(z_near_),
                                             double(z_far_)));
        this->ResetPosition(target_ + Normalize(v) * distance_);

        min_distance_ = z_near_;
        max_distance_ = z_far_;

        roll_angle_ = std::atan2(v.y, v.x);
        pitch_angle_ = std::asin((position_ - target_).z / distance_);

        this->SetPerspectiveModel(fov_, 1.0f, z_near_, z_far_);
    }

    /**
     * Create a TPS camera with only one parameter.
     */
    TPSCamera(float distance)
        : TPSCamera(FPoint3D(0.0f, -distance, distance),
                    FPoint3D(0.0f, 0.0f, 0.0f)) {}

    /**
     * Transform the camera by UI.
     */
    void Update(const cl::UI& ui) {
        // Mouse wheel.
        this->Zoom(ui.mouse_wheel());

        // Move the camera.
        if (ui.mouse_pressed(ImGuiMouseButton_Left)) {
            FVector2D v = ui.mouse_delta();
            this->HorizentalRotate(-rotation_sensitive_ * v.x);
            this->VerticalRotate(rotation_sensitive_ * v.y);
        } else if (ui.mouse_pressed(ImGuiMouseButton_Right)) {
            FVector2D v = ui.mouse_delta() * distance_;

            this->HorizentalMove(-translate_sensitive_ * v.x);
            this->VerticalMove(translate_sensitive_ * v.y);
        }
    }

    /**
     * Move the camera along the given vector.
     */
    void Move(const FVector3D& v) {
        this->position_ += v;
        this->LookAt(this->target() + v);
    }

    /**
     * Rotate the camera horizontally around the target.
     */
    void HorizentalRotate(float radian) {
        if (radian == 0.0f) return;

        float pi2 = static_cast<float>(M_PI) * 2.0f;

        roll_angle_ += radian;
        if (roll_angle_ >  pi2) roll_angle_ -= pi2;
        if (roll_angle_ < -pi2) roll_angle_ += pi2;
        UpdatePosition();
    }

    /**
     * Rotate the camera vertically around the target.
     */
    void VerticalRotate(float radian) {
        if (radian == 0.0f) return;

        pitch_angle_ += radian;
        // Make sure that when pitch is out of bounds, screen doesn't get
        // flipped.
        pitch_angle_ = Clamp(pitch_angle_, -float(M_PI_2) + 0.1f,
                             float(M_PI_2) - 0.1f);
        UpdatePosition();
    }

    /**
     * Move the camera horizontally relative to the target.
     */
    void HorizentalMove(float distance) {
        if (distance == 0.0f) return;
        FVector3D v = CrossProduct(this->direction(), this->up_direction());
        v = v.Normalize() * distance;
        this->Move(v);
    }

    /**
     * Move the camera vertically relative to the target.
     */
    void VerticalMove(float distance) {
        if (distance == 0.0f) return;

        FVector3D v = CrossProduct(this->direction(), this->up_direction());
        FVector3D v1 = CrossProduct(v, this->direction());
        v1 = v1.Normalize() * distance;
        this->Move(v1);
    }

    /**
     * Zoom in/ out of the camera.
     */
    void Zoom(float step) {
        if (step == 0.0f) return;

        float distance = (max_distance_ - min_distance_) / wheel_steps_ * step;
        distance_ += distance;
        distance_ = Clamp(distance_, min_distance_, max_distance_);
        UpdatePosition();
    }

    /**
     * Return the distance from camera to the target.
     */
    float distance() const {
        return distance_;
    }

    void set_min_distance(float d) {
        min_distance_ = std::max(d, z_near_);
    }

    void set_max_distance(float d) {
        max_distance_ = std::min(d, z_far_);
    }

protected:
    /**
     * Update camera's position according to roll angle and pitch angle.
     */
    void UpdatePosition() {
        float c = std::cos(pitch_angle_);

        FPoint3D pos(target_.x + std::cos(roll_angle_) * c * distance_,
                     target_.y + std::sin(roll_angle_) * c * distance_,
                     target_.z + std::sin(pitch_angle_) * distance_);
        this->ResetPosition(pos);
    }

    // Angle for rotation.
    float roll_angle_, pitch_angle_;

    // Distance from camera's position to target.
    float distance_;

    // Minimum and maximum allowed distance from camera's position to target.
    float min_distance_ = 0.0f, max_distance_ = FLT_MAX;

    // The ratio between the mouse movement distance and the actual camera
    // rotation angle.
    float rotation_sensitive_ = 0.005f;

    // The ratio between the mouse movement distance and the actual camera
    // movement distance.
    float translate_sensitive_ = 0.001f;

    // Number of wheels from maximum distance to minimum distance.
    int wheel_steps_ = 50;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_CAMERA_TPS_CAMERA_H_
