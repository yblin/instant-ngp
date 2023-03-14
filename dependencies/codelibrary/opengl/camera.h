//
// Copyright 2021-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPENGL_CAMERA_H_
#define CODELIBRARY_OPENGL_CAMERA_H_

#include <algorithm>
#include <cfloat>
#include <cmath>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/angle.h"
#include "codelibrary/geometry/distance_3d.h"
#include "codelibrary/geometry/intersect_3d.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/geometry/ray_3d.h"
#include "codelibrary/opengl/glad.h"
#include "codelibrary/opengl/transform.h"

namespace cl {
namespace gl {

/**
 * 3D camera for OpenGL.
 *
 * The pipeline:
 *           World Coordinates
 *                   |
 *                   | [ Viewing Matrix ]
 *                   |
 *           Camera Coordinates
 *                   |
 *                   | [ Projection Matrix ]
 *                   |
 *       Normalized Device Coordinates: [-1, 1] * [-1, 1] * [-1, 1]
 *
 * When computing the viewing matrix, we adopt right handed coordinate system.
 */
class Camera {
public:
    // The type of camera.
    enum ProjectionType {
        UNKNOWN,
        PERSPECTIVE,
        ORTHO
    };

    /**
     * Construct a camera.
     *
     * Parameters:
     *  position     - position of the camera.
     *  target       - position of the target (where the camera looks at).
     *  up_direction - the up direction of the camear (the default value is +Y).
     */
    explicit Camera(const FPoint3D& position = FPoint3D(0.0f, 0.0f, 1.0f),
                    const FPoint3D& target   = FPoint3D(0.0f, 0.0f, 0.0f),
                    const FVector3D& up_direction = FVector3D(0.0f, 1.0f, 0.0f))
        : position_(position),
          target_(target),
          up_direction_(up_direction) {
        this->LookAt(target);
    }

    /**
     * Reset position of the camera. The target point is not changed.
     */
    void ResetPosition(const FPoint3D& position) {
        position_ = position;
        LookAt(target_);
    }

    /**
     * Create a viewing matrix.
     *
     * The viewing matrix maps the target(reference) point to the negative z
     * axis and the camera position(eye point) to the origin.
     *
     * When a typical projection matrix is used, the center of the scene
     * therefore maps to the center of the viewport.
     *
     * Similarly, the direction described by the UP vector projected onto the
     * viewing plane is mapped to the positive y axis so that it points upward
     * in the viewport.
     */
    void LookAt(const FPoint3D& target) {
        CHECK(target != position_);

        target_ = target;

        FVector3D direction = Normalize(position_ - target_);
        FVector3D right = Normalize(CrossProduct(up_direction_, direction));
        FVector3D up = CrossProduct(direction, right);

        FVector3D translate = -position_.ToVector();
        float t1 = DotProduct(right, translate);
        float t2 = DotProduct(up, translate);
        float t3 = DotProduct(direction, translate);
        viewing_ = {
            right.x,  up.x, direction.x, 0.0f,
            right.y,  up.y, direction.y, 0.0f,
            right.z,  up.z, direction.z, 0.0f,
            t1,       t2,   t3,          1.0f
        };
        inverse_matrices_updated_ = true;
    }

    /**
     * Set this camera to perspective model.
     *
     * Parameters:
     *  fov    - the field of view angle (in degrees), in the z direction.
     *  aspect - the ratio of width to height.
     *  z_near - distance to the near clipping plane.
     *  z_far  - distance to the far clipping plane.
     */
    void SetPerspectiveModel(float fov, float aspect, float z_near,
                             float z_far) {
        CHECK(fov > 0.0f && fov < 180.0f);
        CHECK(aspect > 0.0f);
        CHECK(z_near > 0.0f);
        CHECK(z_near <= z_far);

        projection_type_ = PERSPECTIVE;

        // height = 2 * ht * z_near.
        float ht = std::tan(DegreeToRadian(fov) * 0.5f);
        float wt = aspect * ht;
        fov_    = fov;
        aspect_ = aspect;
        z_near_ = z_near;
        z_far_  = z_far;

        float zl = z_far - z_near;
        if (ht == 0.0f || wt == 0.0f || zl == 0.0f) return;

        float fn2 = 2.0f * z_near_ * z_far_;
        projection_ = {
            1.0f / wt, 0.0f,      0.0f,                     0.0f,
            0.0f,      1.0f / ht, 0.0f,                     0.0f,
            0.0f,      0.0f,      -(z_far_ + z_near_) / zl, -1.0,
            0.0f,      0.0f,      -fn2 / zl,                0.0f};
        inverse_matrices_updated_ = true;
    }

    /**
     * Set the ortho model.
     *
     * Parameters:
     *  left   - The coordinates for the left vertical clipping plane.
     *  right  - The coordinates for the right vertical clipping plane.
     *  bottom - The coordinates for the bottom horizontal clipping plane.
     *  top    - The coordinates for the top horizontal clipping plans.
     *  z_near - Distance to the near clipping plane.
     *  z_far  - Distance to the far clipping plane.
     */
    void SetOrthoModel(float left, float right, float bottom, float top,
                       float z_near, float z_far) {
        CHECK(left <= right);
        CHECK(bottom <= top);
        CHECK(z_near <= z_far);

        projection_type_ = ORTHO;

        ortho_clipping_plane_ = FBox2D(left, right, bottom, top);
        float height   = top - bottom;
        float width    = right - left;
        float zl = z_far - z_near;
        z_near_        = z_near;
        z_far_         = z_far;
        fov_           = 0.0f;
        if (height == 0.0f || width == 0.0f || zl == 0.0f) return;

        aspect_ = width / height;

        float w1 = -(right + left) / width;
        float h1 = -(top + bottom) / height;
        projection_ = {
            2.0f / width, 0.0f,          0.0f,                     0.0f,
            0.0f,         2.0f / height, 0.0f,                     0.0f,
            0.0f,         0.0f,          -2.0f / zl,               0.0f,
            w1,           h1,            -(z_far_ + z_near_) / zl, 1.0f};
        inverse_matrices_updated_ = true;
    }

    /**
     * Reset camera according to the given viewport.
     */
    void SetViewport(const FBox2D& viewport) {
        if (viewport.empty()) return;

        float w = viewport.x_length(), h = viewport.y_length();
        if (w == 0.0 || h == 0.0) return;

        if (projection_type_ == PERSPECTIVE) {
            float aspect = w / h;
            if (aspect != aspect_) {
                this->SetPerspectiveModel(fov_, aspect, z_near_, z_far_);
            }
        }
    }

    /**
     * Get NDC coordinate of the given real world coordinate.
     */
    FPoint3D Project(const FPoint3D& p) const {
        CHECK(projection_type_ != UNKNOWN);

        FVector4D in(p.x, p.y, p.z, 1.0f);
        in = in * viewing_;
        in = in * projection_;
        return FPoint3D(in.x / in.w, in.y / in.w, in.z / in.w);
    }

    /**
     * Get the project vector of 'v'.
     */
    FVector3D Project(const FVector3D& v) const {
        CHECK(projection_type_ != UNKNOWN);

        FVector4D in(v.x, v.y, v.z, 0.0f);
        in = in * viewing_;
        in.w = 0.0f;

        in = in * projection_;
        return FVector3D(in.x, in.y, in.z);
    }

    /**
     * Get the unproject point of NDC coordinate.
     *
     * Note that the output point is in 4D homogeneous coordinate.
     */
    FPoint3D Unproject(const FPoint3D& p) const {
        CHECK(projection_type_ != UNKNOWN);

        FVector4D in(p.x, p.y, p.z, 1.0f);
        if (inverse_matrices_updated_) {
            inverse_matrices_updated_ = false;
            inverse_viewing_ = viewing_.Inverse();
            inverse_projection_ = projection_.Inverse();
        }
        in = in * inverse_projection_;
        in = in * inverse_viewing_;
        return FPoint3D(in[0] / in[3], in[1] / in[3], in[2] / in[3]);
    }

    /**
     * Compute the frustum of the camera with given z_near and z_far.
     *
     * The frustum is composed of 8 vertices.
     */
    void GetFrustum(float z_near, float z_far,
                    Array<FPoint3D>* vertices) const {
        CHECK(z_near <= z_far);
        CHECK(vertices);
        CHECK(projection_type_ != UNKNOWN)
            << "Need to call SetPerspectiveModel() or SetOrthoModel() first.";

        vertices->resize(8);
        FVector3D direction = Normalize(target_ - position_);
        FVector3D right     = Normalize(CrossProduct(direction, up_direction_));
        FVector3D up        = CrossProduct(right, direction);

        // Near center and far center.
        FPoint3D nc = position_ + direction * z_near;
        FPoint3D fc = position_ + direction * z_far;

        if (projection_type_ == PERSPECTIVE) {
            // These heights and widths are half the heights and widths of
            // the near and far plane rectangles.
            float near_height = tanf(fov_ / 2.0f) * z_near;
            float near_width  = near_height * aspect_;
            float far_height  = tanf(fov_ / 2.0f) * z_far;
            float far_width   = far_height * aspect_;

            // Near plane.
            (*vertices)[0] = nc - right * near_width - up * near_height;
            (*vertices)[1] = nc + right * near_width - up * near_height;
            (*vertices)[2] = nc + right * near_width + up * near_height;
            (*vertices)[3] = nc - right * near_width + up * near_height;
            // Far plane.
            (*vertices)[4] = fc - right * far_width - up * far_height;
            (*vertices)[5] = fc + right * far_width - up * far_height;
            (*vertices)[6] = fc + right * far_width + up * far_height;
            (*vertices)[7] = fc - right * far_width + up * far_height;
        } else {
            float x_min = ortho_clipping_plane_.x_min();
            float x_max = ortho_clipping_plane_.x_max();
            float y_min = ortho_clipping_plane_.y_min();
            float y_max = ortho_clipping_plane_.y_max();

            // Near plane.
            (*vertices)[0] = nc + right * x_min + up * y_min;
            (*vertices)[1] = nc + right * x_max + up * y_min;
            (*vertices)[2] = nc + right * x_max + up * y_max;
            (*vertices)[3] = nc + right * x_min + up * y_max;
            // Far plane.
            (*vertices)[4] = fc + right * x_min + up * y_min;
            (*vertices)[5] = fc + right * x_max + up * y_min;
            (*vertices)[6] = fc + right * x_max + up * y_max;
            (*vertices)[7] = fc + right * x_min + up * y_max;
        }
    }

    /**
     * Compute the frustum of the camera.
     *
     * The frustum is composed of 8 vertices.
     */
    void GetFrustum(Array<FPoint3D>* vertices) const {
        GetFrustum(z_near_, z_far_, vertices);
    }

    /**
     * Get camera ray through the current screen position.
     */
    FRay3D Ray(const FPoint2D& pos) const {
        GLint v[4];
        glGetIntegerv(GL_VIEWPORT, v);

        // Convert screen coordinate to normalized device coordinate.
        float x = 2.0f * (pos.x - v[0]) / v[2] - 1.0f;
        float y = 1.0f - 2.0f * (pos.y - v[1]) / v[3];

        FPoint3D dir = this->Unproject(FPoint3D(x, y, -1.0f));

        return FRay3D(position_, dir);
    }

    /**
     * Compute near plane of the camera.
     */
    FPlane3D near_plane() const {
        FVector3D v = Normalize(direction());

        return FPlane3D(position_ + z_near_ * v, v);
    }

    /**
     * Compute far plane of the camera.
     */
    FPlane3D far_plane() const {
        FVector3D v = Normalize(direction());

        return FPlane3D(position_ + z_far_ * v, v);
    }

    /**
     * Return the projection type of this camera.
     */
    const ProjectionType& type() const {
        return projection_type_;
    }

    /**
     * Return the camera's position.
     */
    const FPoint3D& position() const {
        return position_;
    }

    /**
     * Return the target that the camera looks at.
     */
    const FPoint3D& target() const {
        return target_;
    }

    /**
     * Return the direction from camera's position to target.
     */
    const FVector3D direction() const {
        return target_ - position_;
    }

    /**
     * Return the up direction of the camera.
     */
    const FVector3D& up_direction() const {
        return up_direction_;
    }

    /**
     * Return the viewing matrix (world to camera space).
     */
    const Transform& viewing() const {
        return viewing_;
    }

    /**
     * Return the projection matrix (camera space to NDC space).
     */
    const Transform& projection() const {
        return projection_;
    }

    /**
     * The distance to the near clipping plane.
     */
    float z_near() const {
        return z_near_;
    }

    /**
     * The distance to the far clipping plane.
     */
    float z_far() const {
        return z_far_;
    }

    /**
     * Camera fov.
     */
    float fov() const {
        return fov_;
    }

    /**
     * Return the value of width / height.
     */
    float aspect() const {
        return aspect_;
    }

protected:
    // The projection type of the camera.
    ProjectionType projection_type_ = UNKNOWN;

    // Camera fov.
    float fov_ = 45.0f;

    // width / height.
    float aspect_ = 0.0f;

    // The distance to the near clipping plane.
    float z_near_ = 0.0f;

    // The distance to the far clipping plane.
    float z_far_ = 0.0f;

    // Clipping plane for ortho camera (in camera coordinate system).
    FBox2D ortho_clipping_plane_;

    // The position of the camera (in world coordinate system).
    FPoint3D position_;

    // The target point (where the camera looks at).
    FPoint3D target_;

    // Up direction of the camera.
    FVector3D up_direction_;

    // The viewing matrix to transform the world space into camera space.
    Transform viewing_;

    // The projection matrix to transform the camera space into NDC
    // (Normalized Device Coordinates) space.
    Transform projection_;

    // The inverse matrix of viewing matrix.
    mutable gl::Transform inverse_viewing_;

    // The inverse matrix of projection matrix.
    mutable gl::Transform inverse_projection_;

    // Need to update inverse matrices or not.
    mutable bool inverse_matrices_updated_ = true;
};

} // namespace gl
} // namespace cl

#endif // CODELIBRARY_OPENGL_CAMERA_H_
