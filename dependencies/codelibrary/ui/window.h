//
// Copyright 2020-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UI_WINDOW_H_
#define CODELIBRARY_UI_WINDOW_H_

#include <cstring>

#include "codelibrary/base/log.h"
#include "codelibrary/opengl/glad.h"
#include "codelibrary/util/color/rgb32_color.h"

#if __has_include("GLFW/glfw3.h")
#include "GLFW/glfw3.h"
#else
#include "third_party/GLFW/glfw3.h"
#endif

namespace cl {

/**
 * GLFW window.
 *
 * Usage:
 *
 *   Window window;
 *   while (!window.IsClosed()) {
 *      window.PollEvents();
 *
 *      // Rendering. ..
 *
 *      window.SwapBuffers();
 *  }
 */
class Window {
    /**
     * Load and release GLFW.
     * It is a singleton class.
     */
    class GLFWLib {
    public:
        GLFWLib() {
            glfwSetErrorCallback(ErrorCallback);

            // This function initializes the GLFW library.
            // Before most GLFW functions can be used, GLFW must be initialized.
            CHECK(glfwInit());

            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
            glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        }

        ~GLFWLib() {
            glfwTerminate();
        }

        static void ErrorCallback(int error, const char* description) {
            LOG(INFO) << "GLFW error " << error << ": " << description;
        }
    };

    static GLFWLib GLFW_LIB;

public:
    using Handle = GLFWwindow;

    /**
     * Enumeration of the window styles.
     */
    enum Style {
        NONE        = 0,      // No border / title bar.
        TITLE_BAR   = 1 << 0, // Title bar + fixed border.
        RESIZABLE   = 1 << 1, // Title bar + resizable border + maximize button.
        MAXIMIZE    = 1 << 2, // Maximize the window.

        // Default window style.
        DEFAULT = TITLE_BAR | RESIZABLE,

        // Full screen mode.
        FULL_SCREEN = MAXIMIZE
    };

    Window() = delete;

    /**
     * Create a maximize window.
     */
    Window(const std::string& title, int n_samples = 4)
        : title_(title), n_samples_(n_samples) {
        CHECK(n_samples >= 0);

        GLFWmonitor* monitor = glfwGetPrimaryMonitor();
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);
        ConstructWindow(mode->width, mode->height, DEFAULT, n_samples);
        glfwMaximizeWindow(handle_);

        if (n_samples > 1) glEnable(GL_MULTISAMPLE);
    }

    /**
     * Create a window.
     *
     * It the style set to MAXIMIZE, the parameters of width and height are
     * ignored.
     *
     * Parameters:
     *   title     - window's title.
     *   width     - the width of window.
     *   height    - the height of window.
     *   style     - window's style.
     *   n_samples - enable or distable multisample anti-aliasing (MSAA).
     */
    Window(const std::string& title, int width, int height, int style = DEFAULT,
           int n_samples = 4)
        : title_(title), n_samples_(n_samples) {
        CHECK(width > 0 && height > 0);
        CHECK(style >= 0 && style <= 7);
        CHECK(n_samples >= 0);

        if (style & MAXIMIZE) {
            GLFWmonitor* monitor = glfwGetPrimaryMonitor();
            const GLFWvidmode* mode = glfwGetVideoMode(monitor);
            width = mode->width;
            height = mode->height;
        }

        ConstructWindow(width, height, style, n_samples);
        if (style & MAXIMIZE) glfwMaximizeWindow(handle_);

        if (n_samples > 1) glEnable(GL_MULTISAMPLE);
    }

    Window(const Window&) = delete;

    virtual ~Window() {
        if (handle_) {
            glfwDestroyWindow(handle_);
            handle_ = nullptr;
        }
    }

    void operator=(const Window&) = delete;

    /**
     * Check if the window is closed.
     */
    bool IsClosed() const {
        return glfwWindowShouldClose(handle_);
    }

    /**
     * Close the window.
     * Here we do not destroy the window. We only hide it.
     */
    void Close() const {
        Hide();
    }

    /**
     * Hide the window.
     */
    void Hide() const {
        glfwHideWindow(handle_);
    }

    /**
     * Show the window.
     */
    void Show() const {
        glfwShowWindow(handle_);
    }

    /**
     * Activating vertical synchronization will limit the number of frames
     * displayed to the refresh rate of the monitor. This can avoid some visual
     * artifacts, and limit the framerate to a good value (but not constant
     * across different computers).
     *
     * Vertical synchronization is enabled by default.
     */
    void SetVerticalSyncEnabled(bool is_enable) const {
        glfwSwapInterval(is_enable);
    }

    /**
     * Processes all pending events (inputs, window resize, etc).
     */
    void PollEvents() const {
        glfwPollEvents();
    }

    /**
     * This function swaps the front and back buffers of the window when
     * rendering with OpenGL.
     *
     * When an application draws in a single buffer the resulting image might
     * display flickering issues. This is because the resulting output image is
     * not drawn in an instant, but drawn pixel by pixel and usually from left
     * to right and top to bottom. Because this image is not displayed at an
     * instant to the user while still being rendered to, the result may contain
     * artifacts. To circumvent these issues, windowing applications apply a
     * double buffer for rendering. The front buffer contains the final output
     * image that is shown at the screen, while all the rendering commands draw
     * to the back buffer. As soon as all the rendering commands are finished we
     * swap the back buffer to the front buffer so the image can be displayed
     * without still being rendered to, removing all the aforementioned
     * artifacts.
     */
    void SwapBuffers() const {
        glfwSwapBuffers(handle_);
    }

    /**
     * Makes the context of the specified window current for the calling thread.
     */
    void MakeContextCurrent() const{
        glfwMakeContextCurrent(handle_);
    }

    /**
     * Get mouse position.
     */
    void GetMousePosition(double* mouse_x, double* mouse_y) const {
        CHECK(mouse_x && mouse_y);

        glfwGetCursorPos(handle_, mouse_x, mouse_y);
    }

    /**
     * Return the state of the given mouse button.
     *
     * 0 for released, 1 for pressed.
     */
    int GetMouseButtonState(int mouse_button) const {
        return glfwGetMouseButton(handle_, mouse_button);
    }

    /**
     * Check if a key is pressed.
     */
    bool KeyPressed(int key) const {
        return glfwGetKey(handle_, key) == GLFW_PRESS;
    }

    /**
     * Register mouse cursor.
     */
    void SetMouseCursor(GLFWcursor* cursor) const {
        glfwSetCursor(handle_, cursor);
    }

    /**
     * Set mouse position.
     */
    void SetMousePosition(double x, double y) {
        glfwSetCursorPos(handle_, x, y);
    }

    /**
     * Return an attribute value.
     */
    int GetAttribute(int attribute) {
        return glfwGetWindowAttrib(handle_, attribute);
    }

    /**
     * Return the value of an input option.
     */
    int GetInputMode(int mode) {
        return glfwGetInputMode(handle_, mode);
    }

    /**
     * Hide mouse cursor.
     */
    void HideMouseCursor() {
        glfwSetInputMode(handle_, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
    }

    /**
     * Show mouse cursor.
     */
    void ShowMouseCursor() {
        glfwSetInputMode(handle_, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }

    /**
     * Return the state of all buttons of the specified joystick.
     */
    const unsigned char* GetJoystickButtons(int* buttons_count) {
        return glfwGetJoystickButtons(GLFW_JOYSTICK_1, buttons_count);
    }

    /**
     * Return the values of all axes of the specified joystick.
     */
    const float* GetJoystickAxes(int* axes_count) {
        return glfwGetJoystickAxes(GLFW_JOYSTICK_1, axes_count);
    }

    /**
     * Set size call back.
     */
    void SetSizeCallback(GLFWwindowsizefun func) {
        glfwSetWindowSizeCallback(handle_, func);
    }

    int width() const {
        int w = 0, h = 0;
        glfwGetWindowSize(handle_, &w, &h);
        return w;
    }

    int height() const {
        int w = 0, h = 0;
        glfwGetWindowSize(handle_, &w, &h);
        return h;
    }

    int framebuffer_width() const {
        int w = 0, h = 0;
        glfwGetFramebufferSize(handle_, &w, &h);
        return w;
    }

    int framebuffer_height() const {
        int w = 0, h = 0;
        glfwGetFramebufferSize(handle_, &w, &h);
        return h;
    }

    const std::string& title() const {
        return title_;
    }

    Handle* handle() {
        return handle_;
    }

    int n_samples() const {
        return n_samples_;
    }

private:
    /**
     * Set our logo!
     */
    void SetIcon() {
        const char* logo[] = {
            "................",
            "................",
            ".000000..00.....",
            ".000000..00.....",
            ".00......00.....",
            ".00......00.....",
            ".00......00.....",
            ".00......00.....",
            ".00......00.....",
            ".00......00.....",
            ".00......00.....",
            ".00......00.....",
            ".000000..000000.",
            ".000000..000000.",
            "................",
            "................"
        };

        const unsigned char color[4] = {120, 160, 210, 255};

        unsigned char pixels[16 * 16 * 4];
        unsigned char* target = pixels;
        GLFWimage img = { 16, 16, pixels };

        for (int y = 0; y < img.width; ++y) {
            for (int x = 0;  x < img.height;  x++) {
                if (logo[y][x] == '0')
                    std::memcpy(target, color, 4);
                else
                    std::memset(target, 0, 4);
                target += 4;
            }
        }

        glfwSetWindowIcon(handle_, 1, &img);
    }

    /**
     * Create window with the given parameters.
     */
    void ConstructWindow(int width, int height, int style, int n_samples) {
        // Enable or disable title bar.
        glfwWindowHint(GLFW_DECORATED, (style & TITLE_BAR) != 0);

        // Set resizeable.
        glfwWindowHint(GLFW_RESIZABLE, (style & RESIZABLE) != 0);

        if (n_samples) glfwWindowHint(GLFW_SAMPLES, n_samples);
        handle_ = glfwCreateWindow(width, height, title_.c_str(), NULL, NULL);
        CHECK(handle_);

        Initialize();
    }

    /**
     * Window context initialization.
     */
    void Initialize() {
        glfwMakeContextCurrent(handle_);

        SetIcon();

        // Note that LoadGL() should call after glfwMakeContextCurrent().
        LoadGL();

        SetVerticalSyncEnabled(true);

        glfwSetWindowCloseCallback(handle_, glfwHideWindow);
        SetVerticalSyncEnabled(true);
    }

    std::string title_;
    int n_samples_;
    Handle* handle_ = nullptr;
};

Window::GLFWLib Window::GLFW_LIB = Window::GLFWLib();

} // namespace cl

#endif // CODELIBRARY_UI_WINDOW_H_
