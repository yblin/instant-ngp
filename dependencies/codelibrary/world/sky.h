//
// Copyright 2021-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_SKY_H_
#define CODELIBRARY_WORLD_SKY_H_

#include <cmath>

#include "codelibrary/opengl/shader_factory.h"
#include "codelibrary/world/cubemap.h"
#include "codelibrary/world/light/light.h"
#include "codelibrary/world/no_lighting_node.h"
#include "codelibrary/world/primitive/cube_sphere.h"
#include "codelibrary/world/sky_data_rgb.inl"

namespace cl {
namespace world {

/**
 * Sky dome for 3D world.
 *
 * Reference:
 *   Hosek L, Wilkie A. An analytic model for full spectral sky-dome
 *   radiance[J]. ACM Transactions on Graphics, 2012, 31(4):1-9.
 */
class Sky : public NoLightingNode {
    static const int SUN_RADIANCE_D = 16;
    static const RVector3D SUN_RADIANCE[SUN_RADIANCE_D * SUN_RADIANCE_D];

public:
    /**
     * Initialize a cubemap sky.
     *
     * Parameters:
     *   sun_position       - the relative position of sun.
     *   cubemap_resolution - the resolution of cubemap, note that the sky will
     *                        be render into cubemap.
     */
    Sky(const FVector3D& sun_position = FVector3D(1.0f, 1.0f, 1.0f),
        int cubemap_resolution = 1024)
        : NoLightingNode("Sky"),
          cubemap_resolution_(cubemap_resolution),
          sun_light_("Sun light"),
          ambient_light_("Sky ambient light"),
          cubemap_("Sky cubemap") {
        CHECK(cubemap_resolution_ > 0);

        sun_light_.set_type(Light::DIRECTIONAL);
        ambient_light_.set_type(Light::AMBIENT);
        this->AddNode(&sun_light_);
        this->AddNode(&ambient_light_);
        this->AddNode(&cubemap_);

        sun_position_ = Normalize(sun_position);
        sky_sphere_.SetRenderData(CubeSphere(4));

        this->Update();
    }

    /**
     * Initialize a sphere sky, it can be move with scene.
     *
     * Parameters:
     *   radius       - the radius of the sky.
     *   sun_position - the relative position of sun.
     */
    Sky(float radius,
        const FVector3D& sun_position = FVector3D(1.0f, 1.0f, 1.0f))
        : NoLightingNode("Sky"),
          sun_light_("Sun light"),
          ambient_light_("Sky ambient light") {
        sun_light_.set_type(Light::DIRECTIONAL);
        ambient_light_.set_type(Light::AMBIENT);
        this->AddNode(&sun_light_);
        this->AddNode(&ambient_light_);

        sun_position_ = Normalize(sun_position);
        sky_sphere_.SetRenderData(CubeSphere(4));
        this->SetScale(radius);

        this->Update();
        this->AddRenderObject(&sky_sphere_);
    }

    /**
     * Set relative position of sun.
     */
    void SetSunPosition(const FVector3D& sun_position) {
        sun_position_ = Normalize(sun_position);
        this->Update();
    }

    /**
     * Set sun position according to its polar angle and azimuth angle.
     */
    void SetSunPosition(float polar_angle, float azimuth_angle) {
        float theta = DegreeToRadian(polar_angle);
        float phi   = DegreeToRadian(azimuth_angle);
        sun_position_ = cl::FVector3D(std::sin(theta) * std::cos(phi),
                                      std::sin(theta) * std::sin(phi),
                                      std::cos(theta));
        this->Update();
    }

    /**
     * Set the albedo of atmosphere. The range is [0, 1].
     */
    void SetAlbedo(float albedo) {
        CHECK(albedo >= 0.0f && albedo <= 1.0f);

        albedo_ = albedo;
        this->Update();
    }

    /**
     * Set the turbidity of atmosphere. The range should be [2, 12].
     */
    void SetTurbidity(float turbidity) {
        CHECK(turbidity >= 2.0f && turbidity <= 12.0f);

        turbidity_ = turbidity;
        this->Update();
    }

    /**
     * Get color of the sky at the given direction.
     */
    RGB32Color GetColor(const FVector3D& v) const {
        FVector3D s = GetRadiance(v);
        s *= rgb_normalize_;

        return RGB32Color(s.x, s.y, s.z);
    }

    /**
     * Get radiance of the sky at the given direction.
     */
    FVector3D GetRadiance(const FVector3D& v) const {
        FVector3D v1 = Normalize(v);

        float cos_theta = Clamp(v1.z, 0.0f, 1.0f);
        float cos_gamma = Clamp(DotProduct(v1, sun_position_), 0.0f, 1.0f);
        float gamma = std::acos(cos_gamma);
        FVector3D s = HosekWilkie(cos_theta, gamma, cos_gamma);
        s *= coefficents_[9];

        return s;
    }

    /**
     * Return (unnormalized) RGB radiance of sun.
     */
    FVector3D GetSunRadiance() const {
        float cos_theta = std::cos(sun_theta_);

        if (cos_theta < 0.0f)
            return FVector3D(0.0f, 0.0f, 0.0f);

        double s = Clamp(cos_theta, 0.0f, 0.999999f);
        double t = Clamp((turbidity_ - 2.0) / 10.0, 0.0, 0.999999);
        s *= SUN_RADIANCE_D - 1;
        t *= SUN_RADIANCE_D - 1;

        int si0 = static_cast<int>(std::floor(s));
        int ti0 = static_cast<int>(std::floor(t));
        int si1 = (si0 + 1);
        int ti1 = (ti0 + 1);

        double sf = s - si0;
        double tf = t - ti0;

        ti0 *= SUN_RADIANCE_D;
        ti1 *= SUN_RADIANCE_D;

        RVector3D color = SUN_RADIANCE[si0 + ti0] * (1.0 - sf) * (1.0 - tf) +
                          SUN_RADIANCE[si1 + ti0] *        sf  * (1.0 - tf) +
                          SUN_RADIANCE[si0 + ti1] * (1.0 - sf) *        tf  +
                          SUN_RADIANCE[si1 + ti1] *        sf  *        tf;
        color *= 6.79998e-05;

        return FVector3D(static_cast<float>(color.x),
                         static_cast<float>(color.y),
                         static_cast<float>(color.z));
    }

    /**
     * Return the ambient color (use for ambient light).
     */
    FVector3D GetAmbientColor() const {
        FVector3D z(0.0f, 0.0f, 1.0f);
        FVector3D v = CrossProduct(z, sun_position_);
        v = CrossProduct(z, v);

        return GetRadiance(v) * rgb_normalize_;
    }

    /**
     * Convert the sky model to (SDR) cubemap.
     */
    void ToCubemap(int resolution, Cubemap* cubemap) {
        ToCubemap(resolution, false, cubemap);
    }

    /**
     * Convert the sky model to (HDR) cubemap.
     */
    void ToHDRCubemap(int resolution, Cubemap* cubemap) {
        ToCubemap(resolution, true, cubemap);
    }

    Light* sun_light()                    { return &sun_light_;   }
    const FVector3D& sun_position() const { return sun_position_; }
    float albedo()                  const { return albedo_;       }
    float turbidity()               const { return turbidity_;    }

private:
    virtual void SetShaderParameters(const gl::Camera &camera) override {
        shader_->Use();
        shader_->SetUniform("view", camera.viewing());
        shader_->SetUniform("projection", camera.projection());
        shader_->SetUniform("model", global_transform_);
        SetParameters(false);
    }

    /**
     * Set shader parameters.
     */
    void SetParameters(bool to_hdr = false) const {
        shader_->Use();
        shader_->SetUniform("A", coefficents_[0]);
        shader_->SetUniform("B", coefficents_[1]);
        shader_->SetUniform("C", coefficents_[2]);
        shader_->SetUniform("D", coefficents_[3]);
        shader_->SetUniform("E", coefficents_[4]);
        shader_->SetUniform("F", coefficents_[5]);
        shader_->SetUniform("G", coefficents_[6]);
        shader_->SetUniform("H", coefficents_[7]);
        shader_->SetUniform("I", coefficents_[8]);
        shader_->SetUniform("Z", coefficents_[9]);
        shader_->SetUniform("N", rgb_normalize_);
        shader_->SetUniform("to_hdr", to_hdr);
        shader_->SetUniform("sun_direction", sun_position_);
    }

    /**
     * It should be called when the parameters are changed.
     */
    void Update() {
        sun_theta_ = std::acos(Clamp(sun_position_.z, 0.0f, 1.0f));

        for (int i = 0; i < 3; ++i) {
            coefficents_[0][i] = Evaluate(sky::datasets_rgb[i] + 0, 9);
            coefficents_[1][i] = Evaluate(sky::datasets_rgb[i] + 1, 9);
            coefficents_[2][i] = Evaluate(sky::datasets_rgb[i] + 2, 9);
            coefficents_[3][i] = Evaluate(sky::datasets_rgb[i] + 3, 9);
            coefficents_[4][i] = Evaluate(sky::datasets_rgb[i] + 4, 9);
            coefficents_[5][i] = Evaluate(sky::datasets_rgb[i] + 5, 9);
            coefficents_[6][i] = Evaluate(sky::datasets_rgb[i] + 6, 9);

            // Swapped in the dataset.
            coefficents_[7][i] = Evaluate(sky::datasets_rgb[i] + 8, 9);
            coefficents_[8][i] = Evaluate(sky::datasets_rgb[i] + 7, 9);

            // Radiances.
            coefficents_[9][i] = Evaluate(sky::datasets_rgb_rad[i], 1);
        }

        // Compute the radiances of sun.
        FVector3D s = HosekWilkie(std::cos(sun_theta_), 0.0f, 1.0f);
        s *= coefficents_[9];

        FVector3D convert = FVector3D(0.2126f, 0.7152f, 0.0722f);
        const float illumination = DotProduct(s, convert);

        // We can change the sum amount here.
        float sun_amount = std::fmod(std::asin(sun_position_.z) / float(M_PI_2),
                                     4.0f);
        if (sun_amount > 2.0f)
            sun_amount = 0.0f;
        if (sun_amount > 1.0f)
            sun_amount = 2.0f - sun_amount;
        else if (sun_amount < -1.0f)
            sun_amount = -2.0f - sun_amount;
        sun_amount = 0.6f + 0.45f * sun_amount;
        rgb_normalize_ = sun_amount / illumination;

        if (cubemap_resolution_ > 0)
            ToCubemap(cubemap_resolution_, false, &cubemap_);
        UpdateLight();
    }

    /**
     * Setup the shader.
     */
    void InitializeShader() override {
        std::string vertex_shader = GLSL_SOURCE(
            layout (location = 0) in vec3 pos;

            uniform mat4 view;
            uniform mat4 projection;
            uniform mat4 model;

            out vec3 sky_texcoord;

            void main() {
                sky_texcoord = pos;
                gl_Position = projection * view * model * vec4(pos, 1);
            }
        );

        std::string fragment_shader = GLSL_SOURCE(
            in vec3 sky_texcoord;

            uniform vec3 sun_direction;
            uniform vec3 A;
            uniform vec3 B;
            uniform vec3 C;
            uniform vec3 D;
            uniform vec3 E;
            uniform vec3 F;
            uniform vec3 G;
            uniform vec3 H;
            uniform vec3 I;
            uniform vec3 Z;
            uniform float N;
            uniform bool to_hdr;

            out vec4 out_color;

            vec3 HosekWilkie(float cos_theta, float gamma, float cos_gamma) {
                vec3 chi = (1 + cos_gamma * cos_gamma) /
                           pow(1 + H * H - 2 * cos_gamma * H, vec3(1.5));
                return (1 + A * exp(B / (cos_theta + 0.01))) *
                       (C + D * exp(E * gamma) + F * (cos_gamma * cos_gamma) +
                        G * chi + I * sqrt(cos_theta));
            }

            vec3 SkyRGB(vec3 v, vec3 sun_dir) {
                vec3 v1 = normalize(v);

                float cos_theta = clamp(v1.z, 0, 1);
                float cos_gamma = clamp(dot(v1, sun_dir), 0, 1);
                float gamma = acos(cos_gamma);

                return Z * HosekWilkie(cos_theta, gamma, cos_gamma) * N;
            }

            void main() {
                vec3 color = SkyRGB(sky_texcoord, sun_direction);
                if (to_hdr) {
                    color *= 2.0;
                    color = pow(color, vec3(2.2));
                }
                out_color = vec4(color, 1);
            }
        );

        shader_ = GL_CREATE_SHADER(vertex_shader, fragment_shader);
    }

    /**
     * Hosek to get internal radiance.
     * (1 + A e ^ (B / cos(t))) (1 + C e ^ (D g) + E cos(g) ^ 2 + F mieM(g, G) +
     *  H cos(t)^1/2 + (I - 1))
     *
     * A: sky gradient, carries white -> blue gradient.
     * B: sky tightness.
     * C: sun, carries most of sun-centred blue term.
     * D: sun tightness, higher = tighter.
     * E: rosy hue around sun.
     *
     * Hosek-specific:
     * F: mie term, does most of the heavy lifting for sunset glow.
     * G: mie tuning.
     * H: zenith gradient.
     * I: constant term balanced with H.
     *
     * Notes:
     * A/B still carries some of the "blue" base of sky, but much comes from
     *     C/D.
     * C/E minimal effect in sunset situations, carry bulk of sun halo in
     *     sun-overhead.
     * F/G sunset glow, but also takes sun halo from yellowish to white
     *     overhead.
     */
    FVector3D HosekWilkie(float cos_theta, float gamma, float cos_gamma) const {
        FVector3D chi;
        for (int t = 0; t < 3; ++t) {
            const float a = coefficents_[0][t];
            const float b = coefficents_[1][t];
            const float c = coefficents_[2][t];
            const float d = coefficents_[3][t];
            const float e = coefficents_[4][t];
            const float f = coefficents_[5][t];
            const float g = coefficents_[6][t];
            const float h = coefficents_[7][t];
            const float i = coefficents_[8][t];

            const float exp_m = expf(e * gamma);
            // Rayleigh scattering.
            const float ray_m = cos_gamma * cos_gamma;
            const float mie_m = (1.0f + ray_m) /
                    powf((1.0f + h * h - 2.0f * h * cos_gamma), 1.5f);
            // Vertical zenith gradient.
            const float zenith = sqrtf(std::max(0.0f, cos_theta));

            chi[t] = (1.0f + a * expf(b / (cos_theta + 0.01f))) *
                     (c + d * exp_m + f * ray_m + g * mie_m + i * zenith);
        }
        return chi;
    }

    /**
     * Internal use.
     */
    double EvaluateSpline(const double* spline, int stride, double value) {
        double s1 = value;
        double s2 = s1 * s1;
        double s3 = s1 * s2;
        double s4 = s1 * s3;
        double s5 = s1 * s4;

        double is1 = (1.0 - value);
        double is2 = is1 * is1;
        double is3 = is1 * is2;
        double is4 = is1 * is3;
        double is5 = is1 * is4;

        return 1.0  * is5      * spline[0 * stride] +
               5.0  * is4 * s1 * spline[1 * stride] +
               10.0 * is3 * s2 * spline[2 * stride] +
               10.0 * is2 * s3 * spline[3 * stride] +
               5.0  * is1 * s4 * spline[4 * stride] +
               1.0  *       s5 * spline[5 * stride];
    }

    /**
     * Evaluate the coefficient.
     */
    float Evaluate(const double* dataset, int stride) {
        // Splines are functions of elevation^1/3.
        double elevation_k = std::pow(std::max(0.0, 1.0 - sun_theta_ / M_PI_2),
                                      1.0 / 3.0);

        // Table has values for turbidity 1..10.
        int turbidity0 = Clamp(static_cast<int>(turbidity_), 1, 10);
        int turbidity1 = std::min(turbidity0 + 1, 10);
        float turbidity_k = Clamp(turbidity_ - turbidity0, 0.0f, 1.0f);

        const double* dataset_a0 = dataset;
        const double* dataset_a1 = dataset + stride * 6 * 10;

        double a0t0 = EvaluateSpline(dataset_a0 + stride * 6 * (turbidity0 - 1),
                                     stride, elevation_k);
        double a1t0 = EvaluateSpline(dataset_a1 + stride * 6 * (turbidity0 - 1),
                                     stride, elevation_k);
        double a0t1 = EvaluateSpline(dataset_a0 + stride * 6 * (turbidity1 - 1),
                                     stride, elevation_k);
        double a1t1 = EvaluateSpline(dataset_a1 + stride * 6 * (turbidity1 - 1),
                                     stride, elevation_k);

        return static_cast<float>(a0t0 * (1.0 - albedo_) * (1.0 - turbidity_k) +
                                  a1t0 *        albedo_  * (1.0 - turbidity_k) +
                                  a0t1 * (1.0 - albedo_) * turbidity_k +
                                  a1t1 *        albedo_  * turbidity_k);
    }

    /**
     * Convert the sky model to SDR or HDR cubemap.
     */
    void ToCubemap(int resolution, bool to_hdr, Cubemap* cubemap) {
        CHECK(resolution > 0);
        CHECK(cubemap);

        if (!shader_) this->InitializeShader();
        cubemap->Reset(resolution, true);
        this->SetParameters(to_hdr);
        cubemap->SetTexture(shader_, true);
    }

    /**
     * Update sun light and ambient light.
     */
    void UpdateLight() {
        sun_light_.set_direction(-sun_position_);
        sun_light_.set_radiance(this->GetSunRadiance());
        ambient_light_.set_radiance(this->GetAmbientColor());
    }

    // Normalized sun position.
    FVector3D sun_position_;

    // Sun theta = acos(sun_position.z)
    float sun_theta_;

    // Coefficents for rendering.
    FVector3D coefficents_[10];

    // The coefficient to normalize the RGB color.
    float rgb_normalize_;

    // Albedo of atmosphere.
    float albedo_ = 0.1f;

    // The reduction in the transparency of air because of scattering of light
    // by dust and smoke particles and water droplets.
    // Increase the turbidity to make the atmosphere denser - hazier.
    float turbidity_ = 4.0f;

    // The resolution of the cubemap.
    int cubemap_resolution_ = 0;

    // Sky sphere.
    RenderObject sky_sphere_;

    // Sun light.
    Light sun_light_;

    // Ambient light.
    Light ambient_light_;

    // Sky cubemap.
    Cubemap cubemap_;
};

const RVector3D Sky::SUN_RADIANCE[256] = {
    {39.4028, 1.98004, 5.96046e-08}, {68821.4, 29221.3, 3969.28},
    {189745, 116333, 43283.4},       {284101, 199843, 103207},
    {351488, 265139, 161944},        {400584, 315075, 213163},
    {437555, 353806, 256435},        {466261, 384480, 292823},
    {489140, 409270, 323569},        {507776, 429675, 349757},
    {523235, 446739, 372260},        {536260, 461207, 391767},
    {547379, 473621, 408815},        {556978, 484385, 423827},
    {565348, 493805, 437137},        {572701, 502106, 449002},
    {34.9717, 0.0775114, 0},         {33531, 11971.9, 875.627},
    {127295, 71095, 22201.3},        {216301, 142827, 66113.9},
    {285954, 205687, 115900},        {339388, 256990, 163080},
    {380973, 298478, 205124},        {414008, 332299, 241816},
    {440780, 360220, 273675},        {462869, 383578, 301382},
    {481379, 403364, 325586},        {497102, 420314, 346848},
    {510615, 434983, 365635},        {522348, 447795, 382333},
    {532628, 459074, 397255},        {541698, 469067, 410647},
    {10.0422, 0, 0.318865},          {16312.8, 4886.47, 84.98},
    {85310.4, 43421.5, 11226.2},     {164586, 102046, 42200.5},
    {232559, 159531, 82822.4},       {287476, 209581, 124663},
    {331656, 251771, 163999},        {367569, 287173, 199628},
    {397168, 317025, 231420},        {421906, 342405, 259652},
    {442848, 364181, 284724},        {460784, 383030, 307045},
    {476303, 399483, 326987},        {489856, 413955, 344876},
    {501789, 426774, 360988},        {512360, 438191, 375548},
    {2.3477, 5.96046e-08, 0.129991}, {117.185, 30.0648, 0},
    {57123.3, 26502.1, 5565.4},      {125170, 72886.2, 26819.8},
    {189071, 123708, 59081.9},       {243452, 170892, 95209.2},
    {288680, 212350, 131047},        {326303, 248153, 164740},
    {357842, 278989, 195638},        {384544, 305634, 223657},
    {407381, 328788, 248954},        {427101, 349038, 271779},
    {444282, 366866, 292397},        {459372, 382660, 311064},
    {472723, 396734, 328012},        {484602, 409337, 343430},
    {0.383395, 0, 0.027703},         {58.0534, 12.8383, 0},
    {38221.6, 16163.6, 2681.55},     {95147.4, 52043, 16954.8},
    {153669, 95910.9, 42062},        {206127, 139327, 72640.8},
    {251236, 179082, 104653},        {289639, 214417, 135896},
    {322383, 245500, 165343},        {350467, 272796, 192613},
    {374734, 296820, 217644},        {395864, 318050, 240533},
    {414400, 336900, 261440},        {430773, 353719, 280544},
    {445330, 368800, 298027},        {458337, 382374, 314041},
    {0.0560895, 0, 0.00474608},      {44.0061, 8.32402, 0},
    {25559, 9849.99, 1237.01},       {72294.8, 37148.7, 10649},
    {124859, 74345.6, 29875.8},      {174489, 113576, 55359.1},
    {218617, 151011, 83520.3},       {257067, 185252, 112054},
    {290413, 216016, 139698},        {319390, 243473, 165842},
    {344686, 267948, 190241},        {366896, 289801, 212852},
    {386513, 309371, 233736},        {403942, 326957, 252998},
    {419513, 342823, 270764},        {433487, 357178, 287149},
    {0.00811136, 0, 0.000761211},    {38.0318, 6.09287, 0},
    {17083.4, 5996.83, 530.476},     {54909.7, 26508.7, 6634.5},
    {101423, 57618.7, 21163.3},      {147679, 92573, 42135.2},
    {190207, 127327, 66606.4},       {228134, 160042, 92352.6},
    {261593, 190061, 117993},        {291049, 217290, 142758},
    {317031, 241874, 166258},        {340033, 264051, 188331},
    {360490, 284081, 208945},        {378771, 302212, 228135},
    {395184, 318667, 245976},        {409974, 333634, 262543},
    {0.00118321, 0, 0.000119328},    {34.5228, 4.62524, 0},
    {11414.1, 3646.94, 196.889},     {41690.9, 18909.8, 4091.39},
    {82364.6, 44646.9, 14944.8},     {124966, 75444.4, 32024.3},
    {165467, 107347, 53075.4},       {202437, 138252, 76076.7},
    {235615, 167214, 99627},         {265208, 193912, 122858},
    {291580, 218327, 145272},        {315124, 240580, 166611},
    {336208, 260851, 186761},        {355158, 279331, 205696},
    {372256, 296206, 223440},        {387729, 311636, 240030},
    {0.000174701, 0, 1.84774e-05},   {31.4054, 3.4608, 0},
    {7624.24, 2215.02, 48.0059},     {31644.8, 13484.4, 2490.1},
    {66872.4, 34589.1, 10515},       {105728, 61477.4, 24300.5},
    {143926, 90494.6, 42256.1},      {179617, 119420, 62635.3},
    {212200, 147105, 84088.4},       {241645, 173041, 105704},
    {268159, 197064, 126911},        {292028, 219187, 147374},
    {313550, 239512, 166913},        {333008, 258175, 185447},
    {350650, 275321, 202953},        {366683, 291081, 219433},
    {2.61664e-05, 0, 2.86102e-06},   {27.3995, 2.42835, 5.96046e-08},
    {391.889, 104.066, 0},           {24013.1, 9611.97, 1489.37},
    {54282.4, 26792.1, 7366.53},     {89437, 50090, 18406.3},
    {125174, 76280.7, 33609.8},      {159354, 103145, 51538.2},
    {191098, 129407, 70945.4},       {220163, 154409, 90919.4},
    {246607, 177864, 110847},        {270613, 199690, 130337},
    {292410, 219912, 149156},        {312229, 238614, 167173},
    {330289, 255902, 184328},        {346771, 271876, 200589},
    {3.93391e-06, 0, 4.76837e-07},   {21.8815, 1.51091, 0},
    {106.645, 26.2423, 0},           {18217.8, 6848.77, 869.811},
    {44054, 20748.7, 5134.5},        {75644.5, 40807, 13913.2},
    {108852, 64293.6, 26704.2},      {141364, 89082.8, 42380.1},
    {172081, 113831, 59831.4},       {200579, 137777, 78179.7},
    {226776, 160529, 96794.7},       {250759, 181920, 115250},
    {272686, 201910, 133270},        {292739, 220530, 150685},
    {311103, 237847, 167398},        {327934, 253933, 183349},
    {6.55651e-07, 0, 1.19209e-07},   {15.4347, 0.791314, 0},
    {67.98, 15.4685, 0},             {13818.5, 4877.71, 490.832},
    {35746.5, 16065.3, 3556.94},     {63969.8, 33240.3, 10492.5},
    {94648, 54185.5, 21192.5},       {125394, 76932.4, 34825.1},
    {154946, 100125, 50435.6},       {182726, 122930, 67203.7},
    {208530, 144877, 84504.4},       {232352, 165726, 101891},
    {254283, 185376, 119059},        {274458, 203811, 135807},
    {293024, 221062, 152009},        {310113, 237169, 167579},
    {5.96046e-08, 0, 0},             {9.57723, 0.336247, 0},
    {52.9113, 11.1074, 0},           {10479.8, 3472.19, 262.637},
    {29000.9, 12436.5, 2445.87},     {54089.5, 27073.4, 7891.84},
    {82288.3, 45662.7, 16796.5},     {111218, 66434.7, 28595.3},
    {139508, 88064, 42494.5},        {166453, 109678, 57749.2},
    {191743, 130747, 73756.6},       {215288, 150968, 90064.3},
    {237114, 170191, 106348},        {257311, 188355, 122384},
    {275989, 205455, 138022},        {293255, 221507, 153152},
    {0, 0, 0},                       {5.37425, 0.109694, 0},
    {44.9811, 8.68891, 5.96046e-08}, {7946.76, 2470.32, 128.128},
    {23524.7, 9625.27, 1666.58},     {45729.5, 22047.9, 5917.85},
    {71535.2, 38477.1, 13293.2},     {98636.4, 57365.7, 23460.6},
    {125598, 77452, 35785},          {151620, 97851, 49607},
    {176299, 117990, 64359},         {199469, 137520, 79594.4},
    {221098, 156245, 94979.6},       {241228, 174066, 110274},
    {259937, 190947, 125309},        {277307, 206875, 139956},
    {0, 0, 0},                       {2.83079, 0.0199037, 0},
    {40.0718, 7.10214, 0},           {6025.35, 1756.45, 51.1916},
    {19080.1, 7447.79, 1122.67},     {38657, 17952.9, 4422.16},
    {62181.1, 32419.5, 10503.8},     {87471.2, 49531.4, 19230.6},
    {113069, 68115.1, 30117.9},      {138102, 87295.1, 42596.4},
    {162092, 106474, 56143.2},       {184805, 125266, 70327.1},
    {206156, 143438, 84812.9},       {226144, 160857, 99349.8},
    {244814, 177459, 113755},        {262220, 193206, 127887},
    {0, 0, 0},                       {1.43779, 0, 0.00738072},
    {36.6245, 5.93644, 0},           {4568.17, 1248.02, 9.13028},
    {15473.4, 5761.51, 745.266},     {32674.7, 14616.6, 3291.16},
    {54045.1, 27313.1, 8284.85},     {77563.8, 42764.4, 15747.9},
    {101783, 59900.8, 25332.8},      {125782, 77874.7, 36561.6},
    {149022, 96078.4, 48962},        {171213, 114101, 62125.3},
    {192218, 131678, 75721.7},       {211998, 148648, 89495.8},
    {230564, 164920, 103255},        {247950, 180437, 116847}
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_SKY_H_
