//
// Copyright 2022-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UI_UI_H_
#define CODELIBRARY_UI_UI_H_

#include <map>

#include "codelibrary/base/clamp.h"
#include "codelibrary/base/log.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/opengl/glad.h"
#include "codelibrary/opengl/shader.h"
#include "codelibrary/opengl/shader_factory.h"
#include "codelibrary/ui/glfw_callback.h"
#include "codelibrary/ui/icons_font_awesome.h"
#include "codelibrary/ui/window.h"
#include "codelibrary/util/io/file_util.h"

namespace cl {

/**
 * Graphical User interface for Codelibrary.
 *
 * The following code is adapted from ImGUI library.
 *
 * Note that the following code is thread unsafe.
 *
 * ----------------------------------------------------------------
 * Sample usage:
 *
 * cl::Window window("Codelibrary UI example");
 * cl::UI ui(&window);
 *
 * while (!window.IsClosed()) {
 *      window.PollEvents();
 *
 *      // Clear GL color bits.
 *      glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
 *      glClear(GL_COLOR_BUFFER_BIT);
 *
 *      ui.NewFrame();
 *      ui.Begin("Info");
 *      ImGui::Button("It is a button");
 *      ImGui::Separator();
 *      ImGui::Text("It is a text");
 *      ui.End();
 *      ui.Render();
 *      window.SwapBuffers();
 * }
 * ----------------------------------------------------------------
 */
class UI {
    /**
     * Used to automatically restore the previous ImGuiContext after some
     * operations on a specific ImGuiContext.
     *
     * Usage:
     *   Func() {
     *       ContextControl control(context);
     *       Do something.....
     *   }
     */
    class ContextControl {
    public:
        explicit ContextControl(ImGuiContext* context) {
            CHECK(!current_context_);

            current_context_ = context;
            previous_context_ = ImGui::GetCurrentContext();
            ImGui::SetCurrentContext(current_context_);
        }

        ~ContextControl() {
            ImGui::SetCurrentContext(previous_context_);
        }

    private:
        ImGuiContext* current_context_ = nullptr;
        ImGuiContext* previous_context_ = nullptr;
    };

public:
    /**
     * Used to set the position of next window.
     */
    enum WindowPos {
        LEFT_BOTTOM_WINDOW,
        LEFT_TOP_WINDOW,
        RIGHT_BOTTOM_WINDOW,
        RIGHT_TOP_WINDOW
    };

    UI() = delete;

    explicit UI(Window* window)
        : window_(window) {
        CHECK(window);
        CHECK(ui::WINDOW_UI_BINDER.find(window->handle()) ==
              ui::WINDOW_UI_BINDER.end());

        imgui_context_ = ImGui::CreateContext();
        ui::WINDOW_UI_BINDER[window->handle()] = imgui_context_;

        this->SetDefaultFont(window->width() / 1500 * 13.0);
    }

    UI(const UI&) = delete;

    virtual ~UI() {
        ui::WINDOW_UI_BINDER.erase(window_->handle());
        glDeleteVertexArrays(1, &vertex_array_);
        glDeleteBuffers(1, &vertex_buffer_);
        glDeleteBuffers(1, &element_buffer_);
        glDeleteTextures(1, &font_texture_);
        ImGui::DestroyContext(imgui_context_);
        imgui_context_ = nullptr;
    }

    UI& operator=(const UI&) = delete;

    /**
     * Reset the current font to default font.
     *
     * Parameter:
     *  pixel_size - pixel size of the default font, the default value is 13.
     */
    void SetDefaultFont(float pixel_size = 13.0) {
        CHECK(pixel_size > 0.0);

        ImFontConfig icons_config;
        icons_config.SizePixels = pixel_size;
        IO().Fonts->AddFontDefault(&icons_config);
    }

    /**
     * Append Font-Awesome icons font to ImGui font set.
     *
     * Parameter:
     *  name       - font-awesome file path.
     *  pixel_size - pixel size of the icon font.
     */
    void AddFontAwesomeIcons(const std::string& name, float pixel_size = 13.0) {
        CHECK(pixel_size > 0.0);
        CHECK(file_util::IsFileExist(name))
            << "File: '" << name << "' is not exist.";

        // icons_ranges should be static here. Otherwise, AddFontFromFileTTF()
        // doesn't do what we want.
        static const ImWchar icons_ranges[] = { ICON_MIN_FA, ICON_MAX_FA, 0 };

        ImFontConfig icons_config;
        icons_config.MergeMode = true;
        icons_config.PixelSnapH = true;
        if (IO().Fonts->Fonts.empty()) {
            this->SetDefaultFont(pixel_size);
        }
        IO().Fonts->AddFontFromFileTTF(name.c_str(), pixel_size, &icons_config,
                                       icons_ranges);
    }

    /**
     * Start a new ImGui frame.
     *
     * It sets the current ImGui context to this one.
     */
    void NewFrame() {
        if (!shader_) Initialize();

        ImGuiIO& io = IO();

        // Setup display size (every frame to accommodate for window resizing).
        int w = window_->width(), h = window_->height();
        int display_w = window_->framebuffer_width();
        int display_h = window_->framebuffer_height();
        io.DisplaySize = ImVec2(static_cast<float>(w), static_cast<float>(h));
        if (w > 0 && h > 0) {
            io.DisplayFramebufferScale =
                    ImVec2(static_cast<float>(display_w) / w,
                           static_cast<float>(display_h) / h);
        }

        // Setup time step.
        double current_time = glfwGetTime();
        io.DeltaTime = current_time_ > 0.0 ? static_cast<float>(current_time -
                                                                current_time_)
                                           : 1.0f / 60.0f;
        current_time_ = current_time;

        UpdateMouseData();
        UpdateMouseCursor();
        UpdateGamepads();

        // Call ImGui new frame.
        ImGui::NewFrame();
    }

    /**
     * Render function.
     */
    void Render() {
        ContextControl control(imgui_context_);

        // Set ImGui render data.
        ImGui::Render();

        this->RenderDrawData(ImGui::GetDrawData());
    }

    /**
     * ImGui::Begin().
     */
    bool Begin(const char* name, bool* p_open = nullptr,
               ImGuiWindowFlags flags = 0) {
        previous_imgui_context_ = ImGui::GetCurrentContext();
        ImGui::SetCurrentContext(imgui_context_);
        return ImGui::Begin(name, p_open, flags);
    }

    /**
     * Creating a simple static window without decoration.
     */
    bool BeginStaticWindow(const char* name, WindowPos pos) {
        this->SetNextWindowPos(pos);
        return Begin(name, nullptr, ImGuiWindowFlags_NoDecoration       |
                                    ImGuiWindowFlags_AlwaysAutoResize   |
                                    ImGuiWindowFlags_NoSavedSettings    |
                                    ImGuiWindowFlags_NoFocusOnAppearing |
                                    ImGuiWindowFlags_NoNav);
    }

    /**
     * ImGui::End().
     */
    void End() {
        ImGui::End();
        ImGui::SetCurrentContext(previous_imgui_context_);
    }

    /**
     * ImGui::SetNextWindowPos().
     *
     * Parameter:
     *  pos  - predefined position of next window.
     *  cond - should be one of the following values:
     *         ImGuiCond_None = 0,
     *         ImGuiCond_Always = 1,
     *         ImGuiCond_Once = 2,
     *         ImGuiCond_FirstUseEver = 4,
     *         ImGuiCond_Appearing = 8
     *  gap  - gap between GLFW window and ImGui window.
     */
    void SetNextWindowPos(WindowPos pos, int cond = 0, int gap = 10) {
        ContextControl control(imgui_context_);
        switch(pos) {
        case LEFT_BOTTOM_WINDOW:
            ImGui::SetNextWindowPos(ImVec2(gap, window_->height() - gap),
                                    cond,
                                    ImVec2(0.0f, 1.0f));
            break;
        case LEFT_TOP_WINDOW:
            ImGui::SetNextWindowPos(ImVec2(gap, gap), cond, ImVec2(0.0f, 0.0f));
            break;
        case RIGHT_BOTTOM_WINDOW:
            ImGui::SetNextWindowPos(ImVec2(window_->width() - gap,
                                           window_->height() - gap),
                                    cond,
                                    ImVec2(1.0f, 1.0f));
            break;
        case RIGHT_TOP_WINDOW:
            ImGui::SetNextWindowPos(ImVec2(window_->width() - gap, gap), cond,
                                    ImVec2(1.0f, 0.0f));
            break;
        }
    }

    /**
     * ImGui::SetNextWindowBgAlpha().
     */
    void SetNextWindowBgAlpha(float alpha) {
        ContextControl control(imgui_context_);
        ImGui::SetNextWindowBgAlpha(alpha);
    }

    /**
     * Return the height of parent window.
     */
    int height() const {
        return window_->height();
    }

    /**
     * Return the width of parent window.
     */
    int width() const {
        return window_->width();
    }

    /**
     * Return ImGui::IO of current context.
     */
    ImGuiIO& IO() {
        return imgui_context_->IO;
    }

    /**
     * Return ImGui::IO of current context.
     */
    const ImGuiIO& IO() const {
        return imgui_context_->IO;
    }

    /**
     * ImGui::IsMousePosValid(nullptr).
     */
    bool IsMousePosValid() const {
        ContextControl control(imgui_context_);
        return ImGui::IsMousePosValid(nullptr);
    }

    /**
     * Return the current mouse position.
     */
    FPoint2D mouse_position() const {
        return FPoint2D(IO().MousePos.x, IO().MousePos.y);
    }

    /**
     * Return true if the given mouse button is just pressed.
     */
    bool mouse_just_pressed(ImGuiMouseButton button) const {
        return IO().MouseDownDuration[button] == 0.0f;
    }

    /**
     * Return true if the given mouse button is just released.
     */
    bool mouse_just_released(ImGuiMouseButton button) const {
        return IO().MouseReleased[button] == 0.0f;
    }

    /**
     * Return true if the given mouse button is pressed.
     */
    bool mouse_pressed(ImGuiMouseButton button) const {
        return IO().MouseDown[button];
    }

    /**
     * Return mouse moving vector between two frames.
     */
    FVector2D mouse_delta() const {
        return FVector2D(IO().MouseDelta.x, IO().MouseDelta.y);
    }

    /**
     * Return mouse wheel.
     */
    float mouse_wheel() const {
        return IO().MouseWheel;
    }

    /**
     * Return the current framerate.
     */
    float framerate() const {
        return IO().Framerate;
    }

    /**
     * Return the current window box.
     */
    FBox2D GetWindowBox() const {
        ImGuiWindow* window = imgui_context_->CurrentWindow;
        CHECK(window);

        float y = window_->height() - window->Pos.y - window->Size.y;
        return FBox2D(window->Pos.x, window->Pos.x + window->Size.x,
                      y, y + window->Size.y);
    }

    /**
     * Check if the mouse is hovered on any window.
     */
    bool IsMouseHoveredOnAnyWindow() {
        ContextControl control(imgui_context_);
        return ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow);
    }

private:
    /**
     * Render draw data.
     */
    void RenderDrawData(ImDrawData* draw_data) {
        // Avoid rendering when minimized, scale coordinates for retina displays
        // (screen coordinates != framebuffer coordinates).
        int fb_width = static_cast<int>(draw_data->DisplaySize.x *
                                        draw_data->FramebufferScale.x);
        int fb_height = static_cast<int>(draw_data->DisplaySize.y *
                                         draw_data->FramebufferScale.y);
        if (fb_width <= 0 || fb_height <= 0)
            return;

        // Backup GL state.
        GLint last_scissor_box[4];
        glGetIntegerv(GL_SCISSOR_BOX, last_scissor_box);
        bool last_enable_scissor_test = glIsEnabled(GL_SCISSOR_TEST);

        this->SetupRenderState(draw_data, fb_width, fb_height);

        // Will project scissor/clipping rectangles into framebuffer space.
        // (0,0) unless using multi-viewports.
        ImVec2 clip_off = draw_data->DisplayPos;
        // (1,1) unless using retina display which are often (2,2).
        ImVec2 clip_scale = draw_data->FramebufferScale;

        // Render command lists
        for (int n = 0; n < draw_data->CmdListsCount; ++n) {
            const ImDrawList* cmd_list = draw_data->CmdLists[n];

            // Upload vertex/index buffers
            GLsizeiptr vertex_buffer_size =
                    static_cast<GLsizeiptr>(cmd_list->VtxBuffer.Size) *
                    sizeof(ImDrawVert);
            GLsizeiptr index_buffer_size =
                    static_cast<GLsizeiptr>(cmd_list->IdxBuffer.Size) *
                    sizeof(ImDrawIdx);

            if (vertex_buffer_size_ < vertex_buffer_size) {
                vertex_buffer_size_ = vertex_buffer_size;
                glBufferData(GL_ARRAY_BUFFER, vertex_buffer_size_, nullptr,
                             GL_STREAM_DRAW);
            }
            if (index_buffer_size_ < index_buffer_size) {
                index_buffer_size_ = index_buffer_size;
                glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_buffer_size_,
                             nullptr, GL_STREAM_DRAW);
            }
            glBufferSubData(GL_ARRAY_BUFFER, 0, vertex_buffer_size,
                            (const GLvoid*)cmd_list->VtxBuffer.Data);
            glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, index_buffer_size,
                            (const GLvoid*)cmd_list->IdxBuffer.Data);

            for (int cmd_i = 0; cmd_i < cmd_list->CmdBuffer.Size; ++cmd_i) {
                const ImDrawCmd* pcmd = &cmd_list->CmdBuffer[cmd_i];
                if (pcmd->UserCallback) {
                    // User callback, registered via ImDrawList::AddCallback()
                    // (ImDrawCallback_ResetRenderState is a special callback
                    // value used by the user to request the renderer to reset
                    // render state.)
                    if (pcmd->UserCallback == ImDrawCallback_ResetRenderState)
                        SetupRenderState(draw_data, fb_width, fb_height);
                    else
                        pcmd->UserCallback(cmd_list, pcmd);
                } else {
                    // Project scissor/clipping rectangles into framebuffer
                    // space.
                    ImVec2 clip_min((pcmd->ClipRect.x - clip_off.x) *
                                    clip_scale.x,
                                    (pcmd->ClipRect.y - clip_off.y) *
                                    clip_scale.y);
                    ImVec2 clip_max((pcmd->ClipRect.z - clip_off.x) *
                                    clip_scale.x,
                                    (pcmd->ClipRect.w - clip_off.y) *
                                    clip_scale.y);
                    if (clip_max.x <= clip_min.x || clip_max.y <= clip_min.y)
                        continue;

                    // Apply scissor/clipping rectangle (Y is inverted in
                    // OpenGL)
                    glScissor(static_cast<int>(clip_min.x),
                              static_cast<int>(fb_height - clip_max.y),
                              static_cast<GLsizei>(clip_max.x - clip_min.x),
                              static_cast<GLsizei>(clip_max.y - clip_min.y));

                    // Bind texture, Draw.
                    glActiveTexture(GL_TEXTURE0);
                    glBindTexture(GL_TEXTURE_2D,
                                  (GLuint)(intptr_t)pcmd->GetTexID());
                    glDrawElements(GL_TRIANGLES,
                                   (GLsizei)pcmd->ElemCount,
                                   sizeof(ImDrawIdx) == 2 ? GL_UNSIGNED_SHORT
                                                          : GL_UNSIGNED_INT,
                                   (void*)(intptr_t)(pcmd->IdxOffset *
                                                     sizeof(ImDrawIdx)));
                }
            }
        }

        // Restore modified GL state.
        glScissor(last_scissor_box[0], last_scissor_box[1],
                  static_cast<GLsizei>(last_scissor_box[2]),
                  static_cast<GLsizei>(last_scissor_box[3]));
        if (last_enable_scissor_test)
            glEnable(GL_SCISSOR_TEST);
        else
            glDisable(GL_SCISSOR_TEST);
    }

    /**
     * Initialize, this function only needs to be called once.
     */
    void Initialize() {
        ImGuiIO& io = imgui_context_->IO;
        CHECK(!io.BackendRendererUserData)
                << "Already initialized a renderer backend!";

        InitializeGL();

        CHECK(!io.BackendPlatformUserData)
                << "Already initialized a platform backend!";

        // We can honor GetMouseCursor() values (optional).
        io.BackendFlags |= ImGuiBackendFlags_HasMouseCursors;
        // We can honor io.WantSetMousePos requests (optional, rarely used).
        io.BackendFlags |= ImGuiBackendFlags_HasSetMousePos;
        io.SetClipboardTextFn = [](void* user_data, const char* text) {
            glfwSetClipboardString((GLFWwindow*)user_data, text);
        };
        io.GetClipboardTextFn = [](void* user_data) {
            return glfwGetClipboardString((GLFWwindow*)user_data);
        };
        io.ClipboardUserData = window_->handle();

        // Set platform dependent data in viewport.
    #if defined(_WIN32)
        ImGui::GetMainViewport()->PlatformHandleRaw =
                (void*)glfwGetWin32Window(window_->handle());
    #endif

        // Create mouse cursors.
        // (By design, on X11 cursors are user configurable and some cursors may
        // be missing. When a cursor doesn't exist,
        // GLFW will emit an error which will often be printed by the app, so we
        // temporarily disable error reporting.
        // Missing cursors will return NULL and our UpdateMouseCursor()
        // function will use the Arrow cursor instead.)
        mouse_cursors_[ImGuiMouseCursor_Arrow] =
                glfwCreateStandardCursor(GLFW_ARROW_CURSOR);
        mouse_cursors_[ImGuiMouseCursor_TextInput] =
                glfwCreateStandardCursor(GLFW_IBEAM_CURSOR);
        mouse_cursors_[ImGuiMouseCursor_ResizeNS] =
                glfwCreateStandardCursor(GLFW_VRESIZE_CURSOR);
        mouse_cursors_[ImGuiMouseCursor_ResizeEW] =
                glfwCreateStandardCursor(GLFW_HRESIZE_CURSOR);
        mouse_cursors_[ImGuiMouseCursor_Hand] =
                glfwCreateStandardCursor(GLFW_HAND_CURSOR);
        mouse_cursors_[ImGuiMouseCursor_ResizeAll] =
                glfwCreateStandardCursor(GLFW_ARROW_CURSOR);
        mouse_cursors_[ImGuiMouseCursor_ResizeNESW] =
                glfwCreateStandardCursor(GLFW_ARROW_CURSOR);
        mouse_cursors_[ImGuiMouseCursor_ResizeNWSE] =
                glfwCreateStandardCursor(GLFW_ARROW_CURSOR);
        mouse_cursors_[ImGuiMouseCursor_NotAllowed] =
                glfwCreateStandardCursor(GLFW_ARROW_CURSOR);

        // Install GLFW callback.
        glfwSetWindowFocusCallback(window_->handle(), ui::WindowFocusCallback);
        glfwSetCursorPosCallback(window_->handle(), ui::CursorPositionCallback);
        glfwSetMouseButtonCallback(window_->handle(), ui::MouseButtonCallback);
        glfwSetScrollCallback(window_->handle(), ui::ScrollCallback);
        glfwSetKeyCallback(window_->handle(), ui::KeyCallback);
        glfwSetCharCallback(window_->handle(), ui::CharCallback);
    }

    /**
     * Unlike SetupGL(), this function only needs to be called once.
     */
    void InitializeGL() {
        std::string vertex_shader = GLSL_SOURCE(
            layout (location = 0) in vec2 position;
            layout (location = 1) in vec2 uv;
            layout (location = 2) in vec4 color;

            uniform mat4 projection;
            out vec2 frag_uv;
            out vec4 frag_color;

            void main() {
                frag_uv = uv;
                frag_color = color;
                gl_Position = projection * vec4(position.xy, 0, 1);
            };
        );

        std::string fragment_shader = GLSL_SOURCE(
            in vec2 frag_uv;
            in vec4 frag_color;
            uniform sampler2D texure;
            layout (location = 0) out vec4 out_color;
            void main() {
                out_color = frag_color * texture(texure, frag_uv);
            }
        );

        shader_ = gl::ShaderFactory::GetInstance()->Create(vertex_shader,
                                                           fragment_shader);

        texture_location_ = shader_->GetLocation("texure");
        projection_location_ = shader_->GetLocation("projection");

        // Create buffers.
        glGenVertexArrays(1, &vertex_array_);
        glGenBuffers(1, &vertex_buffer_);
        glGenBuffers(1, &element_buffer_);

        CreateFontsTexture();
    }

    /**
     * Create fonts texture.
     */
    void CreateFontsTexture() {
        ImGuiIO& io = imgui_context_->IO;

        // Build texture atlas.
        unsigned char* pixels;
        int width, height;
        // Load as RGBA 32-bit (75% of the memory is wasted, but default font is
        // so small) because it is more likely to be compatible with user's
        // existing shaders. If your ImTextureId represent a higher-level
        // concept than just a GL texture id, consider calling
        // GetTexDataAsAlpha8() instead to save on GPU memory.
        io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);

        // Upload texture to graphics system.
        // (Bilinear sampling is required by default.
        // Set 'io.Fonts->Flags |= ImFontAtlasFlags_NoBakedLines' or
        // 'style.AntiAliasedLinesUseTex = false' to allow point/nearest
        // sampling).
        GLint last_texture;
        glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
        glGenTextures(1, &font_texture_);
        glBindTexture(GL_TEXTURE_2D, font_texture_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, pixels);

        // Store our identifier.
        io.Fonts->SetTexID((ImTextureID)(intptr_t)font_texture_);

        // Restore state.
        glBindTexture(GL_TEXTURE_2D, last_texture);
    }

    /**
     * Setup render state. It will be called every frame.
     */
    void SetupRenderState(ImDrawData* draw_data, int fb_width, int fb_height) {
        // Setup render state: alpha-blending enabled, no face culling,
        // no depth testing, scissor enabled, polygon fill.
        glEnable(GL_BLEND);
        glBlendEquation(GL_FUNC_ADD);
        glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE,
                            GL_ONE_MINUS_SRC_ALPHA);
        glDisable(GL_CULL_FACE);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_STENCIL_TEST);
        glEnable(GL_SCISSOR_TEST);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // Setup viewport, orthographic projection matrix.
        // Our visible imgui space lies from draw_data->DisplayPos (top left) to
        // draw_data->DisplayPos+data_data->DisplaySize (bottom right).
        // DisplayPos is (0,0) for single viewport apps.
        glViewport(0, 0, fb_width, fb_height);
        float l = draw_data->DisplayPos.x;
        float r = draw_data->DisplayPos.x + draw_data->DisplaySize.x;
        float t = draw_data->DisplayPos.y;
        float b = draw_data->DisplayPos.y + draw_data->DisplaySize.y;
        gl::Transform ortho_projection(
            2.0f / (r - l),    0.0f,               0.0f,   0.0f,
            0.0f,              2.0f / (t - b),     0.0f,   0.0f,
            0.0f,              0.0f,              -1.0f,   0.0f,
            (r + l) / (l - r), (t + b) / (b - t),  0.0f,   1.0f);

        shader_->Use();
        shader_->SetUniform(projection_location_, ortho_projection);
        shader_->SetUniform(texture_location_, 0);

        // Bind vertex/index buffers and setup attributes for ImDrawVert.
        glBindVertexArray(vertex_array_);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, element_buffer_);

        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(ImDrawVert),
                              (GLvoid*)IM_OFFSETOF(ImDrawVert, pos));
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(ImDrawVert),
                              (GLvoid*)IM_OFFSETOF(ImDrawVert, uv));
        glVertexAttribPointer(2, 4, GL_UNSIGNED_BYTE, GL_TRUE,
                              sizeof(ImDrawVert),
                              (GLvoid*)IM_OFFSETOF(ImDrawVert, col));
    }

    /**
     * Update mouse data. It will be called every frame.
     */
    void UpdateMouseData() {
        ImGuiIO& io = imgui_context_->IO;

        if (window_->GetAttribute(GLFW_FOCUSED) != 0) {
            // (Optional) Set OS mouse position from Dear ImGui if requested
            // (rarely used, only when ImGuiConfigFlags_NavEnableSetMousePos is
            // enabled by user).
            if (io.WantSetMousePos) {
                window_->SetMousePosition(io.MousePos.x, io.MousePos.y);
            }
        }
    }

    /**
     * Update mouse cursor flags.
     */
    void UpdateMouseCursor() {
        ImGuiIO& io = imgui_context_->IO;
        if ((io.ConfigFlags & ImGuiConfigFlags_NoMouseCursorChange) ||
            window_->GetInputMode(GLFW_CURSOR) == GLFW_CURSOR_DISABLED)
            return;

        ImGuiMouseCursor imgui_cursor = ImGui::GetMouseCursor();
        if (imgui_cursor == ImGuiMouseCursor_None || io.MouseDrawCursor) {
            // Hide OS mouse cursor if imgui is drawing it or if it wants no
            // cursor.
            window_->HideMouseCursor();
        } else {
            // Show OS mouse cursor.
            window_->SetMouseCursor(mouse_cursors_[imgui_cursor] ?
                                        mouse_cursors_[imgui_cursor] :
                                        mouse_cursors_[ImGuiMouseCursor_Arrow]);
            window_->ShowMouseCursor();
        }
    }

    /**
     * Update gamepad inputs.
     */
    void UpdateGamepads() {
        ImGuiIO& io = ImGui::GetIO();
        if ((io.ConfigFlags & ImGuiConfigFlags_NavEnableGamepad) == 0)
            return;

        io.BackendFlags &= ~ImGuiBackendFlags_HasGamepad;
        int axes_count = 0, buttons_count = 0;
        const float* axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &axes_count);
        const unsigned char* buttons =
                window_->GetJoystickButtons(&buttons_count);
        if (axes_count == 0 || buttons_count == 0)
            return;

        io.BackendFlags |= ImGuiBackendFlags_HasGamepad;
        MapGamepadButton(ImGuiKey_GamepadStart, buttons_count, 6, buttons);
        MapGamepadButton(ImGuiKey_GamepadFaceDown, buttons_count, 0, buttons);
        MapGamepadButton(ImGuiKey_GamepadFaceRight, buttons_count, 1, buttons);
        MapGamepadButton(ImGuiKey_GamepadFaceLeft, buttons_count, 2, buttons);
        MapGamepadButton(ImGuiKey_GamepadFaceUp, buttons_count, 3, buttons);
        MapGamepadButton(ImGuiKey_GamepadDpadLeft, buttons_count, 13, buttons);
        MapGamepadButton(ImGuiKey_GamepadDpadRight, buttons_count, 11, buttons);
        MapGamepadButton(ImGuiKey_GamepadDpadUp, buttons_count, 10, buttons);
        MapGamepadButton(ImGuiKey_GamepadDpadDown, buttons_count, 12, buttons);
        MapGamepadButton(ImGuiKey_GamepadL1, buttons_count, 4, buttons);
        MapGamepadButton(ImGuiKey_GamepadR1, buttons_count, 5, buttons);
        MapGamepadAnalog(ImGuiKey_GamepadL2, axes_count, 4, axes, -0.75f, 1.0f);
        MapGamepadAnalog(ImGuiKey_GamepadR2, axes_count, 5, axes, -0.75f, 1.0f);
        MapGamepadButton(ImGuiKey_GamepadL3, buttons_count, 8, buttons);
        MapGamepadButton(ImGuiKey_GamepadR3, buttons_count, 9, buttons);
        MapGamepadAnalog(ImGuiKey_GamepadLStickLeft, axes_count, 0, axes,
                         -0.25f, -1.0f);
        MapGamepadAnalog(ImGuiKey_GamepadLStickRight, axes_count, 0, axes,
                         0.25f, 1.0f);
        MapGamepadAnalog(ImGuiKey_GamepadLStickUp, axes_count, 1, axes,
                         -0.25f, -1.0f);
        MapGamepadAnalog(ImGuiKey_GamepadLStickDown, axes_count, 1, axes,
                         0.25f, 1.0f);
        MapGamepadAnalog(ImGuiKey_GamepadRStickLeft, axes_count, 2, axes,
                         -0.25f, -1.0f);
        MapGamepadAnalog(ImGuiKey_GamepadRStickRight, axes_count, 2, axes,
                         0.25f, 1.0f);
        MapGamepadAnalog(ImGuiKey_GamepadRStickUp, axes_count, 3, axes,
                         -0.25f, -1.0f);
        MapGamepadAnalog(ImGuiKey_GamepadRStickDown, axes_count, 3, axes,
                         0.25f, 1.0f);
    }

    /**
     * Used for UpdateGamepads().
     */
    void MapGamepadButton(ImGuiKey key, int buttons_count, int button_no,
                          const unsigned char* buttons) {
        ImGuiIO& io = ImGui::GetIO();
        io.AddKeyEvent(key, (buttons_count > button_no &&
                             buttons[button_no] == GLFW_PRESS));
    }

    /**
     * Used for UpdateGamepads().
     */
    void MapGamepadAnalog(ImGuiKey key, int axes_count, int axis_no,
                          const float* axes, float v0, float v1) {
        ImGuiIO& io = ImGui::GetIO();
        float v = (axes_count > axis_no) ? axes[axis_no] : v0;
        v = (v - v0) / (v1 - v0);
        io.AddKeyAnalogEvent(key, v > 0.10f, Clamp(v, 0.0f, 1.0f));
    }

    // Font texture.
    GLuint font_texture_ = 0;

    // Shader to draw UI.
    gl::Shader* shader_ = nullptr;

    // Location of uniform paramters.
    GLint texture_location_ = 0;
    GLint projection_location_ = 0;

    // Rendering data for OpenGL.
    GLuint vertex_array_ = 0;
    GLuint vertex_buffer_ = 0;
    GLuint element_buffer_ = 0;

    // Buffer size.
    GLsizeiptr vertex_buffer_size_ = 0;
    GLsizeiptr index_buffer_size_ = 0;

    // The parent window.
    Window* window_ = nullptr;

    // Mouse cursors.
    GLFWcursor* mouse_cursors_[ImGuiMouseCursor_COUNT];

    // Current time.
    double current_time_ = 0.0;

    // Context for ImGui.
    ImGuiContext* imgui_context_ = nullptr;

    // Previous ImGui contex. Used for Begin() and End().
    ImGuiContext* previous_imgui_context_ = nullptr;
};

} // namespace cl

#endif // CODELIBRARY_UI_UI_H_
