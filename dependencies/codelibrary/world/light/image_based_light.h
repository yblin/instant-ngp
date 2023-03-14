//
// Copyright 2021-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_LIGHT_IMAGE_BASED_LIGHT_H_
#define CODELIBRARY_WORLD_LIGHT_IMAGE_BASED_LIGHT_H_

#include "codelibrary/opengl/framebuffer.h"
#include "codelibrary/opengl/shader_factory.h"
#include "codelibrary/opengl/texture.h"
#include "codelibrary/world/cubemap.h"
#include "codelibrary/world/no_lighting_node.h"

namespace cl {
namespace world {

/**
 * Image-based lighting (IBL).
 */
class ImageBasedLight : public NoLightingNode {
    static const int IRRADIANCE_MAP_RESOLUTION = 32;
    static const int PREFILTER_MAP_RESOLUTION = 256;
    static const int BRDF_MAP_RESOLUTION = 512;

public:
    ImageBasedLight(const std::string& name = "ImageBasedLight")
        : NoLightingNode(name) {
        Initialize();
    }

    /**
     * Construct IBL from environment cubemap.
     */
    ImageBasedLight(const Cubemap& cubemap)
        : NoLightingNode(cubemap.name().empty() ? "ImageBasedLight"
                                                : cubemap.name() + "_light") {
        Initialize();
        Reset(cubemap);
    }

    /**
     * Reset image based light from environment cubemap.
     */
    void Reset(const Cubemap& environment_map) {
        glEnable(GL_DEPTH_TEST);
        // Set depth function to less than and equal for skybox depth trick.
        glDepthFunc(GL_LEQUAL);
        // Enable seamless cubemap sampling for lower mip levels in the
        // pre-filter map.
        glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);

        ComputeIrradianceMap(environment_map);
        ComputePrefilterMap(environment_map);
    }

    /**
     * Return the convoluted irradiance map.
     */
    const Cubemap& irradiance_map() const { return irradiance_map_; }

    /**
     * Return the prefilter cubemap.
     */
    const Cubemap& prefilter_map() const { return prefilter_map_; }

    /**
     * Return the texture for BRDF.
     */
    const gl::Texture& brdf_map() const { return brdf_map_; }

protected:
    /**
     * Initialize irradiance shader.
     */
    void InitializeIrradianceShader() {
        std::string vertex_shader = GLSL_SOURCE(
            layout (location = 0) in vec3 pos;

            uniform mat4 view;
            uniform mat4 projection;

            out vec3 world_pos;

            void main() {
                world_pos = pos;
                gl_Position = projection * view * vec4(pos, 1);
            }
        );

        std::string fragment_shader = GLSL_SOURCE(
            out vec4 out_color;
            in vec3 world_pos;

            uniform samplerCube environment_map;

            const float PI = 3.14159265359;

            void main() {
                // The world vector acts as the normal of a tangent surface
                // from the origin, aligned to world_pos. Given this normal,
                // calculate all incoming radiance of the environment. The
                // result of this radiance is the radiance of light coming from
                // -Normal direction, which is what we use in the PBR shader to
                // sample irradiance.
                vec3 normal = normalize(world_pos);

                vec3 irradiance = vec3(0.0);

                // Tangent space calculation from origin point.
                vec3 up    = vec3(0.0, 1.0, 0.0);
                vec3 right = normalize(cross(up, normal));
                up         = normalize(cross(normal, right));

                float sample_delta = 0.025;
                float nr_samples = 0.0;
                for (float phi = 0.0; phi < 2.0 * PI; phi += sample_delta) {
                    for (float theta = 0.0; theta < 0.5 * PI;
                         theta += sample_delta) {
                        // Spherical to cartesian (in tangent space).
                        vec3 tangent_sample = vec3(sin(theta) * cos(phi),
                                                   sin(theta) * sin(phi),
                                                   cos(theta));

                        // Tangent space to world.
                        vec3 sample_vec = tangent_sample.x * right +
                                          tangent_sample.y * up +
                                          tangent_sample.z * normal;

                        vec3 c = texture(environment_map, sample_vec).rgb;
                        irradiance += c * cos(theta) * sin(theta);
                        ++nr_samples;
                    }
                }
                irradiance = PI * irradiance * (1.0 / nr_samples);

                out_color = vec4(irradiance, 1.0);
            }
        );

        irradiance_shader_ = GL_CREATE_SHADER(vertex_shader, fragment_shader);
    }

    /**
     * Initialize prefilter shader.
     */
    void InitializePrefilterShader() {
        std::string vertex_shader = GLSL_SOURCE(
            layout (location = 0) in vec3 pos;

            uniform mat4 view;
            uniform mat4 projection;

            out vec3 world_pos;

            void main() {
                world_pos = pos;
                gl_Position = projection * view * vec4(pos, 1);
            }
        );

        std::string fragment_shader = GLSL_SOURCE(
            out vec4 out_color;
            in vec3 world_pos;

            uniform samplerCube environment_map;
            uniform int resolution;
            uniform float roughness;

            const float PI = 3.14159265359;

            float DistributionGGX(vec3 n, vec3 h, float roughness) {
                float a = roughness * roughness;
                float a2 = a * a;
                float n_dot_h = max(dot(n, h), 0.0);
                float n_dot_h2 = n_dot_h * n_dot_h;

                float nom   = a2;
                float denom = (n_dot_h2 * (a2 - 1.0) + 1.0);
                denom = PI * denom * denom;

                return nom / denom;
            }

            // Generate points on the hemisphere.
            /**
             * Efficient VanDerCorpus calculation.
             */
            float RadicalInverseVdC(uint bits) {
                 bits = (bits << 16u) | (bits >> 16u);
                 bits = ((bits & 0x55555555u) << 1u) |
                         ((bits & 0xAAAAAAAAu) >> 1u);
                 bits = ((bits & 0x33333333u) << 2u) |
                         ((bits & 0xCCCCCCCCu) >> 2u);
                 bits = ((bits & 0x0F0F0F0Fu) << 4u) |
                         ((bits & 0xF0F0F0F0u) >> 4u);
                 bits = ((bits & 0x00FF00FFu) << 8u) |
                         ((bits & 0xFF00FF00u) >> 8u);
                 return float(bits) * 2.3283064365386963e-10; // 0x100000000
            }

            vec2 Hammersley(uint i, uint n) {
                return vec2(float(i) / float(n), RadicalInverseVdC(i));
            }

            vec3 ImportanceSampleGGX(vec2 xi, vec3 n, float roughness) {
                float a = roughness * roughness;

                float phi = 2.0 * PI * xi.x;
                float cos_theta = sqrt((1.0 - xi.y) /
                                       (1.0 + (a * a - 1.0) * xi.y));
                float sin_theta = sqrt(1.0 - cos_theta * cos_theta);

                // From spherical coordinates to cartesian coordinates - halfway
                // vector
                vec3 h;
                h.x = cos(phi) * sin_theta;
                h.y = sin(phi) * sin_theta;
                h.z = cos_theta;

                // from tangent-space H vector to world-space sample vector
                vec3 up        = abs(n.z) < 0.999 ? vec3(0.0, 0.0, 1.0) :
                                                    vec3(1.0, 0.0, 0.0);
                vec3 tangent   = normalize(cross(up, n));
                vec3 bitangent = cross(n, tangent);

                vec3 sample_vec = tangent * h.x + bitangent * h.y + n * h.z;
                return normalize(sample_vec);
            }

            void main() {
                vec3 n = normalize(world_pos);

                // Make the simplyfying assumption that V equals R equals the
                // normal.
                vec3 r = n;
                vec3 v = r;

                const int SAMPLE_COUNT = 1024;
                vec3 prefiltered_color = vec3(0.0);
                float total_weight = 0.0;

                for (int i = 0; i < SAMPLE_COUNT; ++i) {
                    // Generate a sample vector that's biased towards the
                    // preferred alignment direction (importance sampling).
                    vec2 xi = Hammersley(i, SAMPLE_COUNT);
                    vec3 h  = ImportanceSampleGGX(xi, n, roughness);
                    vec3 l  = normalize(2.0 * dot(v, h) * h - v);

                    float n_dot_l = max(dot(n, l), 0.0);
                    if (n_dot_l > 0.0) {
                        // Sample from the environment's mip level based on
                        // roughness/pdf
                        float d = DistributionGGX(n, h, roughness);
                        float n_dot_h = max(dot(n, h), 0.0);
                        float h_dot_v = max(dot(h, v), 0.0);
                        float pdf = d * n_dot_h / (4.0 * h_dot_v + 0.0001);

                        // Resolution of source cubemap (per face).
                        float sa_texel  = 4.0 * PI /
                                         (6.0 * resolution * resolution);
                        float sa_sample = 1.0 / (float(SAMPLE_COUNT) * pdf +
                                                 0.0001);

                        float mip_level =
                                roughness == 0.0 ? 0.0
                                                 : 0.5 *
                                                   log2(sa_sample / sa_texel);
                        vec3 c = textureLod(environment_map, l, mip_level).rgb;
                        prefiltered_color += c * n_dot_l;
                        total_weight      += n_dot_l;
                    }
                }

                prefiltered_color = prefiltered_color / total_weight;
                out_color = vec4(prefiltered_color, 1.0);
            }
        );

        prefilter_shader_ = GL_CREATE_SHADER(vertex_shader, fragment_shader);
    }

    /**
     * Initialize BRDF shader.
     */
    void InitializeBRDFShader() {
        std::string vertex_shader = GLSL_SOURCE(
            layout (location = 0) in vec3 pos;
            layout (location = 3) in vec2 tex_coord;

            out vec2 texture_coord;

            void main() {
                texture_coord = tex_coord;
                gl_Position = vec4(pos, 1.0);
            }
        );

        std::string fragment_shader = GLSL_SOURCE(
            out vec2 out_color;
            in vec2 texture_coord;

            const float PI = 3.14159265359;

            // Generate points on the hemisphere.
            /**
             * Efficient VanDerCorpus calculation.
             */
            float RadicalInverseVdC(uint bits) {
                 bits = (bits << 16u) | (bits >> 16u);
                 bits = ((bits & 0x55555555u) << 1u) |
                         ((bits & 0xAAAAAAAAu) >> 1u);
                 bits = ((bits & 0x33333333u) << 2u) |
                         ((bits & 0xCCCCCCCCu) >> 2u);
                 bits = ((bits & 0x0F0F0F0Fu) << 4u) |
                         ((bits & 0xF0F0F0F0u) >> 4u);
                 bits = ((bits & 0x00FF00FFu) << 8u) |
                         ((bits & 0xFF00FF00u) >> 8u);
                 return float(bits) * 2.3283064365386963e-10; // / 0x100000000
            }
            vec2 Hammersley(uint i, uint n) {
                return vec2(float(i) / float(n), RadicalInverseVdC(i));
            }
            vec3 ImportanceSampleGGX(vec2 xi, vec3 n, float roughness) {
                float a = roughness * roughness;

                float phi = 2.0 * PI * xi.x;
                float cos_theta = sqrt((1.0 - xi.y) /
                                       (1.0 + (a * a - 1.0) * xi.y));
                float sin_theta = sqrt(1.0 - cos_theta * cos_theta);

                // From spherical coordinates to cartesian coordinates - halfway
                // vector.
                vec3 h;
                h.x = cos(phi) * sin_theta;
                h.y = sin(phi) * sin_theta;
                h.z = cos_theta;

                // From tangent-space H vector to world-space sample vector.
                vec3 up        = abs(n.z) < 0.999 ? vec3(0.0, 0.0, 1.0) :
                                                    vec3(1.0, 0.0, 0.0);
                vec3 tangent   = normalize(cross(up, n));
                vec3 bitangent = cross(n, tangent);

                vec3 sample_vec = tangent * h.x + bitangent * h.y + n * h.z;
                return normalize(sample_vec);
            }

            float GeometrySchlickGGX(float n_dot_v, float roughness) {
                // Note that we use a different k for IBL.
                float a = roughness;
                float k = (a * a) / 2.0;

                float nom   = n_dot_v;
                float denom = n_dot_v * (1.0 - k) + k;

                return nom / denom;
            }

            float GeometrySmith(vec3 n, vec3 v, vec3 l, float roughness) {
                float n_dot_v = max(dot(n, v), 0.0);
                float n_dot_l = max(dot(n, l), 0.0);
                float ggx2 = GeometrySchlickGGX(n_dot_v, roughness);
                float ggx1 = GeometrySchlickGGX(n_dot_l, roughness);

                return ggx1 * ggx2;
            }

            vec2 IntegrateBRDF(float n_dot_v, float roughness) {
                vec3 v;
                v.x = sqrt(1.0 - n_dot_v * n_dot_v);
                v.y = 0.0;
                v.z = n_dot_v;

                float a = 0.0;
                float b = 0.0;

                vec3 n = vec3(0.0, 0.0, 1.0);

                const int SAMPLE_COUNT = 1024;
                for (int i = 0; i < SAMPLE_COUNT; ++i) {
                    // Generates a sample vector that's biased towards the
                    // preferred alignment direction (importance sampling).
                    vec2 xi = Hammersley(i, SAMPLE_COUNT);
                    vec3 h = ImportanceSampleGGX(xi, n, roughness);
                    vec3 l = normalize(2.0 * dot(v, h) * h - v);

                    float n_dot_l = max(l.z, 0.0);
                    float n_dot_h = max(h.z, 0.0);
                    float v_dot_h = max(dot(v, h), 0.0);

                    if (n_dot_l > 0.0) {
                        float g = GeometrySmith(n, v, l, roughness);
                        float g_vis = (g * v_dot_h) / (n_dot_h * n_dot_v);
                        float fc = pow(1.0 - v_dot_h, 5.0);

                        a += (1.0 - fc) * g_vis;
                        b += fc * g_vis;
                    }
                }
                a /= float(SAMPLE_COUNT);
                b /= float(SAMPLE_COUNT);
                return vec2(a, b);
            }

            void main() {
                out_color = IntegrateBRDF(texture_coord.x, texture_coord.y);
            }
        );
        brdf_shader_ = GL_CREATE_SHADER(vertex_shader, fragment_shader);
    }

    /**
     * Initialization, just call once.
     */
    void Initialize() {
        InitializeIrradianceShader();
        InitializePrefilterShader();
        InitializeBRDFShader();

        cube_.SetRenderData(Cube());
        quad_.SetRenderData(Quad());

        irradiance_map_.Reset(IRRADIANCE_MAP_RESOLUTION);
        prefilter_map_.Reset(PREFILTER_MAP_RESOLUTION, true);
        brdf_map_.CreateF(BRDF_MAP_RESOLUTION, BRDF_MAP_RESOLUTION, 2);

        // Setup cameras for 6 faces.
        FPoint3D c(0.0f, 0.0f, 0.0f);
        cameras_[0] = gl::Camera(c, { 1.0f, 0.0f,  0.0f}, {0.0f, -1.0f,  0.0f});
        cameras_[1] = gl::Camera(c, {-1.0f, 0.0f,  0.0f}, {0.0f, -1.0f,  0.0f});
        cameras_[2] = gl::Camera(c, { 0.0f, 1.0f,  0.0f}, {0.0f,  0.0f,  1.0f});
        cameras_[3] = gl::Camera(c, { 0.0f, -1.0f, 0.0f}, {0.0f,  0.0f, -1.0f});
        cameras_[4] = gl::Camera(c, { 0.0f, 0.0f,  1.0f}, {0.0f, -1.0f,  0.0f});
        cameras_[5] = gl::Camera(c, { 0.0f, 0.0f, -1.0f}, {0.0f, -1.0f,  0.0f});
        for (int i = 0; i < 6; ++i) {
            cameras_[i].SetPerspectiveModel(90.0f, 1.0f, 0.1f, 10.0f);
        }

        ComputeBRDFMap();
    }

    /**
     * Compute irradiance map.
     */
    void ComputeIrradianceMap(const Cubemap& environment_map) {
        GLint v[4];
        glGetIntegerv(GL_VIEWPORT, v);
        glViewport(0, 0, IRRADIANCE_MAP_RESOLUTION, IRRADIANCE_MAP_RESOLUTION);

        gl::Framebuffer framebuffer(IRRADIANCE_MAP_RESOLUTION,
                                    IRRADIANCE_MAP_RESOLUTION);
        framebuffer.Bind();

        environment_map.Bind(0);
        irradiance_shader_->Use();
        irradiance_shader_->SetUniform("environment_map", 0);
        for (int i = 0; i < 6; ++i) {
            irradiance_shader_->SetUniform("projection",
                                           cameras_[i].projection());
            irradiance_shader_->SetUniform("view", cameras_[i].viewing());
            framebuffer.AttachColorTexture(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
                                           irradiance_map_.texture_id());
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            cube_.Render();
        }
        glViewport(v[0], v[1], v[2], v[3]);
    }

    /**
     * Compute prefilter map.
     */
    void ComputePrefilterMap(const Cubemap& environment_map) {
        gl::Framebuffer framebuffer;
        framebuffer.Bind();

        GLint v[4];
        glGetIntegerv(GL_VIEWPORT, v);

        environment_map.Bind(0);
        prefilter_shader_->Use();
        prefilter_shader_->SetUniform("environment_map", 0);
        prefilter_shader_->SetUniform("resolution",
                                      environment_map.resolution());

        const int max_mip_levels = 5;
        int prefitted_r = PREFILTER_MAP_RESOLUTION;
        for (int mip = 0; mip < max_mip_levels; ++mip, prefitted_r >>= 1) {
            // Resize framebuffer according to mip-level size.
            framebuffer.Reset(prefitted_r, prefitted_r);
            framebuffer.CreateDepthBuffer(GL_DEPTH_COMPONENT24);
            glViewport(0, 0, prefitted_r, prefitted_r);

            float roughness = static_cast<float>(mip) / (max_mip_levels - 1);
            prefilter_shader_->SetUniform("roughness", roughness);
            for (int i = 0; i < 6; ++i) {
                prefilter_shader_->SetUniform("projection",
                                              cameras_[i].projection());
                prefilter_shader_->SetUniform("view", cameras_[i].viewing());

                framebuffer.AttachColorTexture(
                            GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
                            prefilter_map_.texture_id(), mip);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                cube_.Render();
            }
        }
        glViewport(v[0], v[1], v[2], v[3]);
    }

    /**
     * Generate a 2D LUT from the BRDF equations used.
     */
    void ComputeBRDFMap() {
        GLint v[4];
        glGetIntegerv(GL_VIEWPORT, v);
        glViewport(0, 0, BRDF_MAP_RESOLUTION, BRDF_MAP_RESOLUTION);

        brdf_buffer_.Reset(BRDF_MAP_RESOLUTION, BRDF_MAP_RESOLUTION);
        brdf_buffer_.Bind();
        brdf_buffer_.CreateDepthBuffer(GL_DEPTH_COMPONENT24);
        brdf_buffer_.AttachColorTexture(GL_TEXTURE_2D, brdf_map_.id());

        brdf_shader_->Use();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        quad_.Render();

        brdf_buffer_.Unbind();
        glViewport(v[0], v[1], v[2], v[3]);
    }

    // Shader for converting cubebox to irradiance map.
    gl::Shader* irradiance_shader_ = nullptr;

    // Shader for computing prefilter irradiance map.
    gl::Shader* prefilter_shader_ = nullptr;

    // Shader for computing BRDF.
    gl::Shader* brdf_shader_ = nullptr;

    // Objects used to framebuffer.
    RenderObject cube_, quad_;

    // Convoluted irradiance map.
    Cubemap irradiance_map_;

    // Pre-filter a roughness map to reduce aliasingcubemap.
    Cubemap prefilter_map_;

    // Texture for BRDF.
    gl::Texture brdf_map_;

    // Framebuffer for BRDF.
    gl::Framebuffer brdf_buffer_;

    // Camera for capturing data onto the 6 cubemap face directions.
    gl::Camera cameras_[6];
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_LIGHT_IMAGE_BASED_LIGHT_H_
