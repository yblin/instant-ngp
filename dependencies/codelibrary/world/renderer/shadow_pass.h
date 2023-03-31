//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_RENDERER_SHADOW_PASS_H_
#define CODELIBRARY_WORLD_RENDERER_SHADOW_PASS_H_

#include "codelibrary/geometry/intersect_2d.h"
#include "codelibrary/geometry/intersect_3d.h"
#include "codelibrary/opengl/depth_texture.h"
#include "codelibrary/world/scene.h"

namespace cl {
namespace world {

/**
 * Cascade shadow map.
 */
struct CascadeShadowMap {
    /**
     * Splitted light frustum.
     */
    struct LightFrustum {
        // Z range in camera space.
        float camera_z_near, camera_z_far;

        // Light space matrix.
        gl::Transform projection_view;

        // Depth texture.
        GLuint depth_texture = 0;

        // Scene nodes that intersected with this frustum.
        Array<Node*> nodes;
    };

    // Number of cascade levels.
    int n_cascade_levels = 0;

    // Resolution of the shadow map.
    int resolution = 0;

    // Shadow light direction.
    FVector3D direction;

    // Depth textures in light view space.
    Array<gl::DepthTexture> depth_textures;

    // Splitted light frustums.
    Array<LightFrustum> light_frustums;
};

/**
 * Shadow pass.
 */
class ShadowPass {
    // Default shadow map resolution.
    // Only for directional light currently.
    static const int SHADOW_MAP_RESOLUTION = 2048;

    // Cascade number.
    static const int MAX_N_CASCADE_LEVELS = 8;

    ShadowPass() {
        InitializeShader();
        ResetCascadeLevels(3);
        ResetResolution(SHADOW_MAP_RESOLUTION);
    }

public:
    static ShadowPass* GetInstance() {
        static ShadowPass instance;
        return &instance;
    }

    /**
     * Input scene, output shadow map.
     *
     * Return false if no shadow map is created.
     */
    bool Pass(const gl::Camera& camera, Scene* scene) {
        CHECK(scene);

        auto light = scene->lights().directional_light;
        if (!light) return false;
        if (!scene->show_shadow()) return false;

        Array<Node*> shadow_nodes;
        for (Node* n : scene->nodes()) {
            if (n->is_visible() && (n->cast_shadow() || n->receive_shadow())) {
                shadow_nodes.push_back(n);
            }
        }

        CreateShadowMap(camera, shadow_nodes, light->direction());
        return true;
    }

    /**
     * Reset the resolution of the shadow map.
     */
    void ResetResolution(int resolution) {
        CHECK(resolution > 0);
        CHECK(resolution <= 32768);

        if (shadow_map_.resolution == resolution) return;

        shadow_map_.resolution = resolution;
        for (int i = 0; i < shadow_map_.n_cascade_levels; ++i) {
            shadow_map_.depth_textures[i].Reset(resolution, resolution);
        }
        framebuffer_.Reset(resolution, resolution);
    }

    /**
     * Reset the number of cascade levels.
     */
    void ResetCascadeLevels(int n_cascade_levels) {
        CHECK(n_cascade_levels > 0);
        CHECK(n_cascade_levels <= MAX_N_CASCADE_LEVELS);

        if (shadow_map_.n_cascade_levels == n_cascade_levels) return;

        shadow_map_.n_cascade_levels = n_cascade_levels;
        shadow_map_.depth_textures.resize(n_cascade_levels);
        for (int i = 0; i < n_cascade_levels; ++i) {
            shadow_map_.depth_textures[i].Reset(shadow_map_.resolution,
                                                shadow_map_.resolution);
        }
    }

    /**
     * Set bias to fine-tune the position and definition of the shadow.
     */
    void set_bias(float bias) {
        CHECK(bias >= 0.0f);

        bias_ = bias;
    }

    /**
     * Get a reasonable CSM split values by adjusting lambda value.
     */
    void set_split_lambda(float lambda) {
        CHECK(lambda > 0.1f && lambda <= 1.0f);

        split_lambda_ = lambda;
    }

    /**
     * Return the computed shadow map.
     */
    const CascadeShadowMap& shadow_map() const {
        return shadow_map_;
    }

    /**
     * return the bias value of the shadow.
     */
    float bias() const {
        return bias_;
    }

private:
    /**
     * Initialize shader for computing shadow map.
     */
    void InitializeShader() {
        static std::string vertex_shader = GLSL_SOURCE(
            layout (location = 0) in vec3 pos;
            layout (location = 4) in mat4 instance_matrix;

            uniform bool is_instance;
            uniform mat4 projection_viewing;
            uniform mat4 model;

            void main() {
                if (is_instance) {
                    gl_Position = projection_viewing * model * instance_matrix *
                                  vec4(pos, 1);
                } else {
                    gl_Position = projection_viewing * model * vec4(pos, 1);
                }
            }
        );

        static std::string fragment_shader = GLSL_SOURCE(
            void main() {}
        );

        shader_ = GL_CREATE_SHADER(vertex_shader, fragment_shader);
    }

    /**
     * Create the directional shadow map.
     */
    void CreateShadowMap(const gl::Camera& camera, const Array<Node*>& nodes,
                         const FVector3D& direction) {
        if (camera.type() == gl::Camera::UNKNOWN) {
            LOG(INFO) << "Camera has not been set up.";
            return;
        }

        shadow_map_.direction = direction;
        ComputeLightFrutums(camera, nodes, direction);

        // Store viewport.
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);
        glViewport(0, 0, shadow_map_.resolution, shadow_map_.resolution);
        glEnable(GL_DEPTH_CLAMP);

        framebuffer_.Bind();

        for (int i = 0; i < shadow_map_.n_cascade_levels; ++i) {
            framebuffer_.AttachDepthTexture(shadow_map_.depth_textures[i].id());
            const auto& frustum = shadow_map_.light_frustums[i];

            // Render to the depth map.
            glClear(GL_DEPTH_BUFFER_BIT);
            glCullFace(GL_FRONT);

            shader_->Use();
            shader_->SetUniform("projection_viewing", frustum.projection_view);
            for (Node* node : frustum.nodes) {
                shader_->SetUniform("model", node->global_transform());
                shader_->SetUniform("is_instance", node->is_instance());
                node->Render();
            }
            glCullFace(GL_BACK);
        }
        framebuffer_.Unbind();

        // Reset viewport.
        glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
        glDisable(GL_DEPTH_CLAMP);

        return;
    }

    /**
     * Split light's frustum according to the distance from scene to the
     * camera.
     */
    void ComputeLightFrutums(const gl::Camera& camera,
                             const Array<Node*>& nodes,
                             FVector3D direction) {
        Array<float> split_z(shadow_map_.n_cascade_levels + 1);
        split_z[0] = camera.z_near();
        for (int i = 1; i <= shadow_map_.n_cascade_levels; ++i) {
            float p = float(i) / shadow_map_.n_cascade_levels;
            float z1 = camera.z_near() * std::pow(camera.z_far() /
                                                  camera.z_near(),
                                                  p);
            float z2 = camera.z_near() + (camera.z_far() - camera.z_near()) * p;
            float z = split_lambda_ * (z1 - z2) + z2;
            split_z[i] = Clamp(z, camera.z_near(), camera.z_far());
        }

        // Compute light view.
        FPoint3D origin(0.0f, 0.0f, 0.0f);
        FPlane3D plane(origin, direction);
        FVector3D z_axis(0.0f, 0.0f, 1.0f);
        FVector3D right = Normalize(CrossProduct(direction, z_axis));
        if (right.norm() < FLT_EPSILON) {
            right = FVector3D(1.0f, 0.0f, 0.0f);
        }
        FVector3D up = CrossProduct(right, direction);

        // Important! The nodes in light frustums must be cleared.
        shadow_map_.light_frustums.clear();
        shadow_map_.light_frustums.resize(shadow_map_.n_cascade_levels);
        for (int i = 0; i < shadow_map_.n_cascade_levels; ++i) {
            auto& light_frustum = shadow_map_.light_frustums[i];
            light_frustum.camera_z_near = split_z[i];
            light_frustum.camera_z_far = split_z[i + 1];
            light_frustum.depth_texture = shadow_map_.depth_textures[i].id();

            Array<FPoint3D> camera_frustum;
            camera.GetFrustum(split_z[i], split_z[i + 1], &camera_frustum);
            if (camera_frustum.empty()) continue;

            // Convert camera frustum to viewing box in light space.
            Array<FPoint3D> projection;
            for (const FPoint3D& p : camera_frustum) {
                FPoint3D q = geometry::Project(p, plane);
                double dis = SignedDistance(p, plane);
                projection.emplace_back(DotProduct(q.ToVector(), right),
                                        DotProduct(q.ToVector(), up),
                                        static_cast<float>(dis));
            }
            FBox3D camera_box(projection.begin(), projection.end());
            CHECK(!camera_box.empty());

            // Expanded camera viewing box.
            // Nodes that are not in the camera frustum may also cast shadows.
            FBox3D expanded_camera_box =
                    FBox3D(camera_box.x_min(), camera_box.x_max(),
                           camera_box.y_min(), camera_box.y_max(),
                           -FLT_MAX,           camera_box.z_max());

            FBox3D cast_box, receive_box;
            for (Node* node : nodes) {
                FBox3D box = node->GetBoundingBox();
                if (box.empty()) continue;

                Array<FPoint3D> ps;
                for (int i = 0; i < 2; ++i) {
                    float x = i == 0 ? box.x_min() : box.x_max();
                    for (int j = 0; j < 2; ++j) {
                        float y = j == 0 ? box.y_min() : box.y_max();
                        for (int k = 0; k < 2; ++k) {
                            float z = k == 0 ? box.z_min() : box.z_max();
                            FPoint3D p(x, y, z);
                            p = node->global_transform()(p);
                            FPoint3D q = geometry::Project(p, plane);
                            double dis = SignedDistance(p, plane);
                            ps.emplace_back(DotProduct(q.ToVector(), right),
                                            DotProduct(q.ToVector(), up),
                                            static_cast<float>(dis));
                        }
                    }
                }

                // Compute node's bounding box in light space.
                box = FBox3D(ps.begin(), ps.end());
                if (node->cast_shadow() &&
                    geometry::Intersect(box, expanded_camera_box)) {
                    light_frustum.nodes.push_back(node);
                    cast_box.Join(box);
                }

                if (node->receive_shadow() &&
                    geometry::Intersect(box, camera_box)) {
                    receive_box.Join(box);
                }
            }
            if (cast_box.empty() || receive_box.empty()) {
                light_frustum.nodes.clear();
                continue;
            }

            FBox2D camera_box_2d(camera_box.x_min(), camera_box.x_max(),
                                 camera_box.y_min(), camera_box.y_max());
            FBox2D cast_box_2d(cast_box.x_min(), cast_box.x_max(),
                               cast_box.y_min(), cast_box.y_max());
            FBox2D receive_box_2d(receive_box.x_min(), receive_box.x_max(),
                                  receive_box.y_min(), receive_box.y_max());
            FBox2D viewing;
            if (!geometry::Intersect(cast_box_2d, receive_box_2d, &viewing) ||
                !geometry::Intersect(camera_box_2d, viewing, &viewing)) {
                light_frustum.nodes.clear();
                continue;
            }

            // Note that light frustum should cover:
            // 1. the camera's frustum;
            // 2. the cast shadow nodes that in front of the camera's frustum in
            //    light view space.
            // 3. the receive shadow nodes that in the camera's frustum.
            float z_near = cast_box.z_min();
            float z_far  = std::min(camera_box.z_max(), receive_box.z_max());

            if (z_near >= z_far) continue;

            FPoint3D pos = origin + direction * (z_near - 0.1f);
            z_far -= (z_near - 0.1f);
            z_near = 0.1f;

            gl::Camera light_view(pos, pos + direction, up);
            light_view.SetOrthoModel(viewing.x_min(),
                                     viewing.x_max(),
                                     viewing.y_min(),
                                     viewing.y_max(),
                                     z_near,
                                     z_far);
            light_frustum.projection_view = light_view.projection() *
                                            light_view.viewing();
        }
    }

    // Used to alter the split value.
    float split_lambda_ = 0.9f;

    // Bias for shadow.
    float bias_ = 1.0f;

    // Shader for computing the shadow map.
    gl::Shader* shader_ = nullptr;

    // Framebuffer to draw the depth map.
    gl::Framebuffer framebuffer_;

    // Shadow map.
    CascadeShadowMap shadow_map_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_RENDERER_SHADOW_PASS_H_
