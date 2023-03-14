//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_RENDERER_LIGHT_PASS_H_
#define CODELIBRARY_WORLD_RENDERER_LIGHT_PASS_H_

#include "codelibrary/world/scene.h"
#include "codelibrary/world/renderer/shadow_pass.h"

namespace cl {
namespace world {

/**
 * Forward light pass.
 */
class LightPass {
    LightPass() {
        InitializeShader();
    }

public:
    static LightPass* GetInstance() {
        static LightPass instance;
        return &instance;
    }

    /**
     * Input scene, output light texture.
     *
     * Here, we're only dealing with single image-based light, single
     * directional light, and ambient light.
     *
     * For other multiple lights, such as point lights, additional passes are
     * required to speed up processing.
     */
    void Pass(const gl::Camera& camera, Scene* scene) {
        CHECK(scene);

        // Create shadow map before frustum culling, because objects outside the
        // camera's frustum may also contribute to the shadows.
        this->SetupCommonParameters(scene->lights(), camera);

        // Shadow pass.
        auto shadow_pass = ShadowPass::GetInstance();
        if (shadow_pass->Pass(camera, scene)) {
            this->SetupShadowMapParameters(shadow_pass->shadow_map());
        }

        // Objects that are not within the camera frustum will not be used.
        scene->FrustumCull(camera);

        for (Node* node : scene->nodes()) {
            SetupNodeParameters(node);
            node->Render();
            node->RenderWithoutLight(camera);
        }
    }

private:
    /**
     * Setup common parameters for the shader.
     */
    void SetupCommonParameters(const LightSet& light_set,
                               const gl::Camera& camera) {
        CHECK(light_set.lights.size() <= LightSet::MAX_LIGHTS);

        if (light_set.image_based_light) {
            light_set.image_based_light->irradiance_map().Bind(13);
            light_set.image_based_light->prefilter_map().Bind(14);
            light_set.image_based_light->brdf_map().Bind(15);
        }

        auto light = light_set.directional_light;
        auto ambient_light = light_set.ambient_light;

        shader_->Use();
        shader_->SetUniform("has_directional_light", light != nullptr);
        if (light) {
            shader_->SetUniform("light_direction", light->global_transform() *
                                                   light->direction());
            shader_->SetUniform("light_radiance", light->radiance());
        }
        shader_->SetUniform("projection", camera.projection());
        shader_->SetUniform("view", camera.viewing());
        shader_->SetUniform("view_pos", camera.position());
        if (ambient_light) {
            shader_->SetUniform("ambient_light", ambient_light->radiance());
        }
        shader_->SetUniform("has_ibl", light_set.image_based_light != nullptr);
        shader_->SetUniform("has_shadow_map", false);
    }

    /**
     * Setup shadow map parameters for lighting shader.
     */
    void SetupShadowMapParameters(const CascadeShadowMap& shadow_map) {
        const auto& light_frustums = shadow_map.light_frustums;

        shader_->Use();
        shader_->SetUniform("has_shadow_map", true);
        shader_->SetUniform("shadow_map.n_levels", light_frustums.size());
        for (int i = 0; i < light_frustums.size(); ++i) {
            shader_->SetUniform("shadow_map.light_space_matrix", i,
                                light_frustums[i].projection_view);
            shader_->SetUniform("shadow_map.split_z", i,
                                light_frustums[i].camera_z_far);
            shader_->SetUniform("shadow_map.depth_map", i, 16 + i);
            glActiveTexture(GL_TEXTURE16 + i);
            glBindTexture(GL_TEXTURE_2D, light_frustums[i].depth_texture);
        }

        auto shadow_pass = ShadowPass::GetInstance();
        shader_->SetUniform("shadow_map.bias", shadow_pass->bias());
        shader_->SetUniform("shadow_map.direction", shadow_map.direction);
    }

    /**
     * Setup parameters for the given node.
     */
    void SetupNodeParameters(const Node* node) {
        CHECK(node);

        shader_->Use();
        shader_->SetUniform("model", node->global_transform());
        shader_->SetUniform("is_instance", node->is_instance());

        // Set material parameters.
        const Material* m = node->material();
        if (!m) {
            shader_->SetUniform(m_locations_[0], true);
            shader_->SetUniform(m_locations_[2], false);
            shader_->SetUniform(m_locations_[3], 0.0f);
            shader_->SetUniform(m_locations_[4], false);
            shader_->SetUniform(m_locations_[5], 0.0f);
            shader_->SetUniform(m_locations_[6], false);
            shader_->SetUniform(m_locations_[7], 0.0f);
            shader_->SetUniform(m_locations_[8], false);
            shader_->SetUniform(m_locations_[9], false);
        } else {
            shader_->SetUniform(m_locations_[0], m->use_vertex_color);
            shader_->SetUniform(m_locations_[1], m->albedo);
            shader_->SetUniform(m_locations_[2], m->albedo_map != nullptr);
            shader_->SetUniform(m_locations_[3], m->metallic);
            shader_->SetUniform(m_locations_[4], m->metallic_map != nullptr);
            shader_->SetUniform(m_locations_[5], m->roughness);
            shader_->SetUniform(m_locations_[6], m->roughness_map != nullptr);
            shader_->SetUniform(m_locations_[7], m->ao);
            shader_->SetUniform(m_locations_[8], m->ao_map != nullptr);
            shader_->SetUniform(m_locations_[9], m->normal_map != nullptr);

            if (m->albedo_map)    m->albedo_map->Bind(0);
            if (m->roughness_map) m->roughness_map->Bind(1);
            if (m->metallic_map)  m->metallic_map->Bind(2);
            if (m->ao_map)        m->ao_map->Bind(3);
            if (m->normal_map)    m->normal_map->Bind(4);
        }
    }

    /**
     * Initialize rendering shader.
     */
    void InitializeShader() {
        static std::string vertex_shader = GLSL_SOURCE(
            layout (location = 0) in vec3 pos;
            layout (location = 1) in vec4 color;
            layout (location = 2) in vec3 norm;
            layout (location = 3) in vec2 tex_coord;
            layout (location = 4) in mat4 instance_matrix;

            uniform mat4 model;
            uniform mat4 view;
            uniform mat4 projection;
            uniform bool is_instance;

            out vec4 frag_color;
            out vec3 frag_pos;
            out vec3 normal;
            out vec2 uv;

            void main() {
                mat4 model_t = is_instance ? model * instance_matrix
                                           : model;
                vec4 world_pos = model_t * vec4(pos, 1.0);
                frag_color = color;
                normal = normalize(mat3(model_t) * norm);
                frag_pos = world_pos.xyz;
                uv = tex_coord;
                gl_Position = projection * view * world_pos;
            }
        );

        static std::string fragment_shader = GLSL_SOURCE(
            in vec3 frag_pos;
            in vec4 frag_color;
            in vec3 normal;
            in vec2 uv;

            layout (location = 0) out vec4 out_color;
            layout (location = 1) out vec4 bright_color;

            // Camera paramters.
            uniform vec3 view_pos;
            uniform mat4 view;

            // Material parameters.
            struct Material {
                bool use_vertex_color;
                vec4 albedo;
                bool has_albedo_map;
                sampler2D albedo_map;
                float metallic;
                bool has_metallic_map;
                sampler2D metallic_map;
                float roughness;
                bool has_roughness_map;
                sampler2D roughness_map;
                float ao;
                bool has_ao_map;
                sampler2D ao_map;
                bool has_normal_map;
                sampler2D normal_map;
            };
            uniform Material material;

            // Shadow map for directional light.
            uniform bool has_shadow_map;
            struct ShadowMap {
                int n_levels;
                float split_z[8];
                sampler2D depth_map[8];
                mat4 light_space_matrix[8];
                float bias;
                vec3 direction;
            };
            uniform ShadowMap shadow_map;

            // Directional light.
            uniform bool has_directional_light;
            uniform vec3 light_direction;
            uniform vec3 light_radiance;

            // Ambient light.
            uniform vec3 ambient_light;

            // Image based lighting.
            uniform bool has_ibl;
            uniform samplerCube irradiance_map;
            uniform samplerCube prefilter_map;
            uniform sampler2D brdf_map;

            const float PI = 3.14159265359;

            /**
             * Compute shadow.
             */
            float ShadowCalculation(vec3 frag_pos, vec3 n) {
                // Select cascade layer.
                vec4 frag_pos_view_space = view * vec4(frag_pos, 1.0);
                float depth = abs(frag_pos_view_space.z);

                int layer = shadow_map.n_levels - 1;
                for (int i = 0; i < shadow_map.n_levels; ++i) {
                    if (depth < shadow_map.split_z[i]) {
                        layer = i;
                        break;
                    }
                }

                vec4 pos = shadow_map.light_space_matrix[layer] *
                           vec4(frag_pos, 1.0);

                // Perform perspective divide.
                vec3 proj_coord = pos.xyz / pos.w;
                // Transform to [0,1] range.
                proj_coord = proj_coord * 0.5 + 0.5;
                // Get depth of current fragment from light's perspective.
                float current_depth = proj_coord.z;

                // Keep the shadow at 0.0 when outside the far_plane region of
                // the light's frustum.
                if (current_depth > 1.0) return 0.0;

                // Calculate bias (based on depth map resolution and slope).
                vec3 dir = -normalize(shadow_map.direction);
                float bias = max(0.05 * (1.0 - dot(n, dir)), 0.002);
                bias *= shadow_map.bias / log(shadow_map.split_z[layer]);

                // PCF
                float shadow = 0.0;
                vec2 texel_size = 1.0 / textureSize(shadow_map.depth_map[layer],
                                                    0);
                for (int x = -1; x <= 1; ++x) {
                    for (int y = -1; y <= 1; ++y) {
                        vec2 uv = proj_coord.xy + vec2(x, y) * texel_size;
                        float pcf_depth = texture(shadow_map.depth_map[layer],
                                                  uv).r;
                        shadow += (current_depth - bias) > pcf_depth ? 1.0
                                                                     : 0.0;
                    }
                }
                shadow /= 9.0;
                return shadow;
            }

            /**
             * Return normal vector from normal_map in world space.
             */
            vec3 GetNormalFromMap() {
                vec3 n = normalize(normal);
                if (!material.has_normal_map) return n;

                vec3 tangent_normal = texture(material.normal_map, uv).xyz *
                                      2.0 - 1.0;

                vec3 q1  = dFdx(frag_pos);
                vec3 q2  = dFdy(frag_pos);
                vec2 st1 = dFdx(uv);
                vec2 st2 = dFdy(uv);

                vec3 t = normalize(q1 * st2.t - q2 * st1.t);
                vec3 b = -normalize(cross(n, t));
                mat3 tbn = mat3(t, b, n);

                return normalize(tbn * tangent_normal);
            }

            /**
             * Normal distribution: Trowbridge-Reitz GGX.
             *
             * Reference:
             *   Walter B, Marschner SR, Li H, Torrance KE. Microfacet Models
             *   for Refraction through Rough Surfaces. Rendering techniques,
             *   2007, 18th.
             */
            float DistributionGGX(vec3 n, vec3 h, float roughness) {
                float a = roughness * roughness;
                float a2 = a * a;
                float n_dot_h = max(dot(n, h), 0.0);
                float n_dot_h2 = n_dot_h * n_dot_h;

                float nom   = a2;
                float denom = (n_dot_h2 * (a2 - 1.0) + 1.0);
                denom = PI * denom * denom;

                // Prevent divide by zero for roughness=0.0 and NdotH=1.0
                return nom / max(denom, 0.0000001);
            }

            /**
             * Geometry distribution: Schlick-Beckmann GGX.
             *
             * Reference:
             *   Schlick C. An inexpensive BRDF model for physically‐based
             *   rendering. In Computer graphics forum, 1994, 13(3): 233-246.
             */
            float GeometrySchlickGGX(float n_dot_v, float roughness) {
                float r = (roughness + 1.0);
                float k = (r * r) / 8.0;
                float nom   = n_dot_v;
                float denom = n_dot_v * (1.0 - k) + k;

                return nom / denom;
            }

            /**
             * Geometric function use Smith's method.
             *
             * Reference:
             *   Smith B. Geometrical shadowing of a random rough surface. IEEE
             *   transactions on antennas and propagation, 1967, 15(5), 668-671.
             */
            float GeometrySmith(vec3 n, vec3 v, vec3 l, float roughness) {
                float n_dot_v = max(dot(n, v), 0.0);
                float n_dot_l = max(dot(n, l), 0.0);
                float ggx2 = GeometrySchlickGGX(n_dot_v, roughness);
                float ggx1 = GeometrySchlickGGX(n_dot_l, roughness);

                return ggx1 * ggx2;
            }

            /**
             * Fresnel function.
             *
             * Reference:
             *   Schlick C. An inexpensive BRDF model for physically‐based
             *   rendering. In Computer graphics forum, 1994, 13(3): 233-246.
             */
            vec3 FresnelSchlick(float cos_theta, vec3 f0) {
                return f0 + (1.0 - f0) * pow(max(1.0 - cos_theta, 0.0), 5.0);
            }

            /**
             * Fresnel function with roughness.
             */
            vec3 FresnelSchlickRoughness(float cos_theta, vec3 f0,
                                         float roughness) {
                return f0 + (max(vec3(1.0 - roughness), f0) - f0) *
                            pow(max(1.0 - cos_theta, 0.0), 5.0);
            }

            void main() {
                vec3 albedo = material.use_vertex_color ? frag_color.rgb :
                                material.has_albedo_map ?
                                  texture(material.albedo_map, uv).rgb :
                                  material.albedo.rgb;
                albedo = pow(albedo, vec3(2.2));

                float metallic = material.has_metallic_map ?
                                    texture(material.metallic_map, uv).x :
                                    material.metallic;
                float roughness = material.has_roughness_map ?
                                    texture(material.roughness_map, uv).x :
                                    material.roughness;
                float ao = material.has_ao_map ? texture(material.ao_map, uv).x
                                               : material.ao;

                vec3 n = GetNormalFromMap();
                vec3 v = normalize(view_pos - frag_pos);
                // Specular reflection vector.
                vec3 r = reflect(-v, n);

                // Calculate reflectance at normal incidence.
                // If it is dia-electric (like plastic), use F0 of 0.04.
                // If it is a metal, use the albedo color as F0 (metallic
                // workflow).
                vec3 f0 = vec3(0.04);
                f0 = mix(f0, albedo, metallic);

                // Reflectance equation.
                vec3 lo = vec3(0.0);

                if (has_directional_light) {
                    // Calculate directional light radiance.
                    vec3 l = -normalize(light_direction);
                    vec3 h = normalize(v + l);
                    vec3 radiance = light_radiance;

                    // Cook-Torrance BRDF.
                    //
                    //           D(h)F(v,h)G(l,v,h)
                    // f(l,v) = --------------------
                    //           4(n dot l)(n dot v)
                    //
                    float ndf = DistributionGGX(n, h, roughness);
                    float g   = GeometrySmith(n, v, l, roughness);
                    vec3 f    = FresnelSchlick(max(dot(h, v), 0.0), f0);

                    vec3 numerator    = ndf * g * f;
                    float denominator = 4.0 * max(dot(n, v), 0.0) *
                                              max(dot(n, l), 0.0) + 0.0001;
                    vec3 specular = numerator / denominator;

                    // kS is equal to Fresnel.
                    vec3 ks = f;

                    // For energy conservation, the diffuse and specular light
                    // can't be above 1.0 (unless the surface emits light); to
                    // preserve this relationship the diffuse component (kD)
                    // should equal 1.0 - kS.
                    vec3 kd = vec3(1.0) - ks;

                    // Multiply kD by the inverse metalness such that only
                    // non-metals have diffuse lighting, or a linear blend if
                    // partly metal (pure metals have no diffuse light).
                    kd *= 1.0 - metallic;

                    // Scale light by NL.
                    float n_dot_l = max(dot(n, l), 0.0);

                    // Add to outgoing radiance Lo.
                    // Note that we already multiplied the BRDF by the Fresnel
                    // (kS) so we won't multiply by kS again.
                    lo += (kd * albedo / PI + specular) * radiance * n_dot_l;
                }

                vec3 ambient = pow(ambient_light, vec3(2.2)) * albedo * ao;
                if (has_ibl) {
                    // Image based ambient lighting.
                    vec3 f = FresnelSchlickRoughness(max(dot(n, v), 0.0), f0,
                                                     roughness);
                    vec3 ks = f;
                    vec3 kd = 1.0 - ks;
                    ks *= 1.0 - metallic;
                    vec3 irradiance = texture(irradiance_map, n).rgb;
                    vec3 diffuse    = irradiance * albedo;

                    // Sample both the pre-filter map and the BRDF lut and
                    // combine them together as per the Split-Sum approximation
                    // to get the IBL specular part.
                    const float MAX_REFLECTION_LOD = 4.0;
                    vec3 prefilter_color =
                            textureLod(prefilter_map, r,
                                       roughness * MAX_REFLECTION_LOD).rgb;
                    vec2 brdf = texture(brdf_map, vec2(max(dot(n, v), 0.0),
                                                       roughness)).rg;
                    vec3 specular = prefilter_color * (f * brdf.x + brdf.y);
                    ambient = (kd * diffuse + specular) * ao;
                }

                // Compute shadow.
                // Compute the frag pos in light space.
                float shadow = has_shadow_map ? ShadowCalculation(frag_pos, n)
                                              : 0.0f;

                vec3 color = ambient + (1.0 - shadow) * lo;

                // HDR tonemapping (Reinhard).
                color = color / (color + vec3(1.0));

                // Gamma correct.
                color = pow(color, vec3(1.0 / 2.2));

                out_color = vec4(color, 1.0);
            }
        );

        shader_ = GL_CREATE_SHADER(vertex_shader, fragment_shader);
        shader_->Use();

        int l0 = shader_->GetLocation("material.use_vertex_color");
        int l1 = shader_->GetLocation("material.albedo");
        int l2 = shader_->GetLocation("material.has_albedo_map");
        int l3 = shader_->GetLocation("material.metallic");
        int l4 = shader_->GetLocation("material.has_metallic_map");
        int l5 = shader_->GetLocation("material.roughness");
        int l6 = shader_->GetLocation("material.has_roughness_map");
        int l7 = shader_->GetLocation("material.ao");
        int l8 = shader_->GetLocation("material.has_ao_map");
        int l9 = shader_->GetLocation("material.has_normal_map");
        m_locations_ = {l0, l1, l2, l3, l4, l5, l6, l7, l8, l9};

        shader_->SetUniform("material.albedo_map",    0);
        shader_->SetUniform("material.roughness_map", 1);
        shader_->SetUniform("material.metallic_map",  2);
        shader_->SetUniform("material.ao_map",        3);
        shader_->SetUniform("material.normal_map",    4);
        shader_->SetUniform("irradiance_map",         13);
        shader_->SetUniform("prefilter_map",          14);
        shader_->SetUniform("brdf_map",               15);
    }

    // Rendering shader.
    gl::Shader* shader_ = nullptr;

    // Locations of shader materials.
    Array<int> m_locations_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_RENDERER_LIGHT_PASS_H_
