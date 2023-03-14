//
// Copyright 2021-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_KERNEL_TRANSFORMABLE_H_
#define CODELIBRARY_WORLD_KERNEL_TRANSFORMABLE_H_

#include <cfloat>

#include "codelibrary/base/clamp.h"
#include "codelibrary/geometry/quaternion.h"
#include "codelibrary/opengl/transform.h"

namespace cl {
namespace world {

/**
 * Class to implement transformation. It implements translation, rotation, and
 * scaling operations.
 */
struct Transformable {
    using Transform = gl::Transform;

public:
    Transformable() = default;

    Transformable(const Transformable&) = delete;

    Transformable& operator=(const Transformable&) = delete;

    /**
     * Translate this node.
     */
    void Translate(const FVector3D& v) {
        this->Translate(v.x, v.y, v.z);
    }
    void Translate(float x, float y, float z) {
        FVector3D v = transform_.GetTranslation();
        this->SetTranslation(v.x + x, v.y + y, v.z + z);
    }

    /**
     * Get translation vector.
     */
    FVector3D GetTranslation() const {
        return transform_.GetTranslation();
    }

    /**
     * Set translation vector.
     */
    void SetTranslation(const FVector3D& v) {
        this->SetTranslation(v.x, v.y, v.z);
    }
    void SetTranslation(float x, float y, float z) {
        transform_.SetTranslation(Clamp(x, min_x_, max_x_),
                                  Clamp(y, min_y_, max_y_),
                                  Clamp(z, min_z_, max_z_));
    }

    /**
     * Counterclockwise rotation about the local X-axis.
     */
    void Pitch(float degree) {
        degree = Clamp(degree, min_pitch_ - pitch_, max_pitch_ - pitch_);
        if (degree == 0.0f) return;

        pitch_ += degree;
        transform_.Pitch(degree);
    }

    /**
     * Counterclockwise rotation about the local Y-axis.
     */
    void Yaw(float degree) {
        degree = Clamp(degree, min_yaw_ - yaw_, max_yaw_ - yaw_);
        if (degree == 0.0f) return;

        yaw_ += degree;
        transform_.Yaw(degree);
    }

    /**
     * Counterclockwise rotation about the local Z-axis.
     */
    void Roll(float degree) {
        degree = Clamp(degree, min_roll_ - roll_, max_roll_ - roll_);
        if (degree == 0.0f) return;

        roll_ += degree;
        transform_.Roll(degree);
    }

    /**
     * Counterclockwise rotation about the global X-axis.
     */
    void PitchGlobal(float degree) {
        degree = Clamp(degree, min_pitch_ - pitch_, max_pitch_ - pitch_);
        if (degree == 0.0f) return;

        transform_.Roll(-roll_);
        transform_.Yaw(-yaw_);

        this->Pitch(degree);

        transform_.Yaw(yaw_);
        transform_.Roll(roll_);
    }

    /**
     * Counterclockwise rotation about the global Y-axis.
     */
    void YawGlobal(float degree) {
        degree = Clamp(degree, min_yaw_ - yaw_, max_yaw_ - yaw_);
        if (degree == 0.0f) return;

        transform_.Roll(-roll_);
        transform_.Yaw(-yaw_);
        transform_.Pitch(-pitch_);

        this->Yaw(degree);

        transform_.Pitch(pitch_);
        transform_.Yaw(yaw_);
        transform_.Roll(roll_);
    }

    /**
     * Counterclockwise rotation about the global Z-axis.
     */
    void RollGlobal(float degree) {
        degree = Clamp(degree, min_roll_ - roll_, max_roll_ - roll_);
        if (degree == 0.0f) return;

        transform_.Roll(-roll_);
        transform_.Yaw(-yaw_);
        transform_.Pitch(-pitch_);

        this->Roll(degree);

        transform_.Pitch(pitch_);
        transform_.Yaw(yaw_);
        transform_.Roll(roll_);
    }

    /**
     * Get rotation matrix.
     */
    void GetRotation(FMatrix3* rot) const {
        CHECK(rot);

        transform_.GetRotation(rot);
    }

    /**
     * Get rotation quaternion.
     */
    void GetRotation(FQuaternion* rot) const {
        CHECK(rot);

        transform_.GetRotation(rot);
    }

    /**
     * Rotate from vector v1 to v2.
     */
    void Rotate(const FVector3D& v1, const FVector3D& v2) {
        transform_.Rotate(v1, v2);
        UpdateEulerAngles();
    }

    /**
     * Reset the transform to unit.
     */
    void ResetTransform() {
        transform_ = gl::Transform();
        scale_x_ = 1.0f;
        scale_y_ = 1.0f;
        scale_z_ = 1.0f;
        pitch_ = Clamp(0.0f, min_pitch_, max_pitch_);
        yaw_   = Clamp(0.0f, min_yaw_, max_yaw_);
        roll_  = Clamp(0.0f, min_roll_, max_roll_);

        transform_.Pitch(pitch_);
        transform_.Yaw(yaw_);
        transform_.Roll(roll_);
    }

    /**
     * Get the transform of this node
     */
    Transform GetTransform() const {
        Transform t = transform_;
        t.Scale(scale_x_, scale_y_, scale_z_);
        return t;
    }

    /**
     * Set scaling factor of this node.
     */
    void SetScale(float scale) {
        CHECK(scale > 0.0f);

        scale_x_ = scale;
        scale_y_ = scale;
        scale_z_ = scale;
    }
    void SetScale(float scale_x, float scale_y, float scale_z) {
        CHECK(scale_x > 0.0f);
        CHECK(scale_y > 0.0f);
        CHECK(scale_z > 0.0f);

        scale_x_ = scale_x;
        scale_y_ = scale_y;
        scale_z_ = scale_z;
    }

    void set_pitch_range(float min, float max) {
        CHECK(min <= max);

        min_pitch_ = min;
        max_pitch_ = max;
    }

    void set_yaw_range(float min, float max) {
        CHECK(min <= max);

        min_yaw_ = min;
        max_yaw_ = max;
    }

    void set_roll_range(float min, float max) {
        CHECK(min <= max);

        min_roll_ = min;
        max_roll_ = max;
    }

    void set_x_range(float min, float max) {
        CHECK(min <= max);

        min_x_ = min;
        max_x_ = max;
    }

    void set_y_range(float min, float max) {
        CHECK(min <= max);

        min_y_ = min;
        max_y_ = max;
    }

    void set_z_range(float min, float max) {
        CHECK(min <= max);

        min_z_ = min;
        max_z_ = max;
    }

    void set_scale_x(float scale) {
        CHECK(scale > 0.0f);

        scale_x_ = scale;
    }

    void set_scale_y(float scale) {
        CHECK(scale > 0.0f);

        scale_y_ = scale;
    }

    void set_scale_z(float scale) {
        CHECK(scale > 0.0f);

        scale_z_ = scale;
    }

    // O(1) functions to access the members.
    float pitch()   const { return pitch_;   }
    float yaw()     const { return yaw_;     }
    float roll()    const { return roll_;    }
    float scale_x() const { return scale_x_; }
    float scale_y() const { return scale_y_; }
    float scale_z() const { return scale_z_; }

protected:
    /**
     * Update euler angles.
     */
    void UpdateEulerAngles() {
        FQuaternion rot;
        transform_.GetRotation(&rot);
        float pitch, yaw, roll;
        rot.ToEulerAngles(&pitch, &yaw, &roll);

        pitch = Clamp(RadianToDegree(pitch), min_pitch_, max_pitch_);
        yaw   = Clamp(RadianToDegree(yaw), min_yaw_, max_yaw_);
        roll  = Clamp(RadianToDegree(roll), min_roll_, max_roll_);

        transform_.Pitch(pitch - pitch_);
        transform_.Yaw(yaw - yaw_);
        transform_.Roll(roll - roll_);
        pitch_ = pitch;
        yaw_ = yaw;
        roll_ = roll;
    }

    // Three euler angles in degrees.
    float pitch_ = 0.0f, yaw_ = 0.0f, roll_ = 0.0f;

    // Pitch angle range.
    float min_pitch_ = -FLT_MAX, max_pitch_ = FLT_MAX;

    // Roll angle range.
    float min_roll_ = -FLT_MAX, max_roll_ = FLT_MAX;

    // Yaw angle range.
    float min_yaw_ = -FLT_MAX, max_yaw_ = FLT_MAX;

    // The bounding range for translation.
    float min_x_ = -FLT_MAX, max_x_ = FLT_MAX;
    float min_y_ = -FLT_MAX, max_y_ = FLT_MAX;
    float min_z_ = -FLT_MAX, max_z_ = FLT_MAX;

    // Scaling.
    float scale_x_ = 1.0f, scale_y_ = 1.0f, scale_z_ = 1.0f;

private:
    // Translation and rotation transform.
    // The complete transform equals to:
    //    transform_ * [scale_x, scale_y, scale_z, 1].
    Transform transform_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_KERNEL_TRANSFORMABLE_H_
