#include "MainLoop.h"
#include "GraphicsModule.h" // Full definition included here
#include <stdexcept>
#include <chrono>
#include <iostream>
#include <thread>
#include "ImGuiModule.h"
#include "backends/imgui_impl_sdl3.h"

// Constructor and Destructor defined here where GraphicsModule is a complete type
MainLoop::MainLoop() = default;
MainLoop::~MainLoop() = default;

void MainLoop::Initialize(const std::string& title, uint32_t width, uint32_t height) {

    try {

        m_graphicsModule.Initialize("Vulkan RayTrace Exp");
    } catch (const std::exception& e) {
        std::cerr << "Graphics initialization failed: " << e.what() << std::endl;
        SDL_DestroyWindow(m_graphicsModule.getWindow());
        SDL_Quit();
        throw;
    }

    m_camera.SetViewport({0, 0, (int)width, (int)height});
    m_camera.LookAt({0.0f, 1.0f, 3.5f}, {0.0f, 0.5f, 0.0f});

}

void MainLoop::Run()
{
    using clock           = std::chrono::high_resolution_clock;
    using seconds_f       = std::chrono::duration<float>;
    constexpr seconds_f   kTargetFrame   { 1.0f / 60.0f };      // 16.666 ms

    auto lastFrameStart = clock::now();
    m_isRunning = true;

    while (m_isRunning)
    {
        /* ---------------- time & delta ---------------- */
        const auto thisFrameStart = clock::now();
        const float deltaTime     = seconds_f(thisFrameStart - lastFrameStart).count();
        lastFrameStart            = thisFrameStart;

        /* ------- input, simulation, rendering --------- */
        handleEvents();
        update(deltaTime);

         m_graphicsModule.RenderFrame(m_camera);

        /* --------------- frame throttling ------------- */
        const auto afterRender  = clock::now();
        const auto frameTime    = afterRender - thisFrameStart;

        if (frameTime < kTargetFrame)
            std::this_thread::sleep_for(kTargetFrame - frameTime);

        /* -------------- diagnostics output ------------ */
        const float fps  = 1.0f / std::max(seconds_f(kTargetFrame).count(),
                                          seconds_f(frameTime).count());
        const float mspp = 1000.0f / fps;
        std::printf("FPS: %.1f  (%.2f ms)\n", fps, mspp);
        std::fflush(stdout);
    }
}



void MainLoop::Shutdown() {

    m_graphicsModule.Shutdown();

    if (m_graphicsModule.getWindow()) {
        SDL_DestroyWindow(m_graphicsModule.getWindow());
    }
    SDL_Quit();
}

void MainLoop::handleEvents() {

    bool        relative = false;
    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {

        //ImGui_ImplSDL3_ProcessEvent(&ev);


        switch (ev.type) {
        case SDL_EVENT_QUIT:
            m_isRunning = false;
            break;
        case SDL_EVENT_WINDOW_RESIZED:
        case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED:
            m_graphicsModule.SignalResize();
            break;
        case SDL_EVENT_KEY_DOWN:
            if (ev.key.key == SDLK_ESCAPE) {
                m_isRunning = false;
            }
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
            if (ev.button.button == SDL_BUTTON_RIGHT) {   // still the same macro :contentReference[oaicite:2]{index=2}
                // query current state and flip it
                relative = !SDL_GetWindowRelativeMouseMode(m_graphicsModule.getWindow());     // SDL3 query :contentReference[oaicite:3]{index=3}
                SDL_SetWindowRelativeMouseMode(m_graphicsModule.getWindow(), relative);       // enable / disable :contentReference[oaicite:4]{index=4}

                // (optional) lock the cursor to the window too
                SDL_SetWindowMouseGrab(m_graphicsModule.getWindow(), relative);
            }
            break;
        case SDL_EVENT_MOUSE_MOTION:
            if (m_relativeMouseMode) {
                m_camera.Rotate(ev.motion.yrel * -0.003f, ev.motion.xrel * -0.003f);
            }
            break;
        }
    }
}

void MainLoop::update(float deltaTime) {
    const float cameraSpeed = 5.0f * deltaTime;
    const float rotationSpeed = 60.0f * deltaTime; // Degrees per second

    const bool* keyboardState = SDL_GetKeyboardState(nullptr);

    float moveForward = 0.0f;
    float moveSide = 0.0f;
    float moveVertical = 0.0f;
    float yawRotation = 0.0f;

    // Movement
    if (keyboardState[SDL_SCANCODE_W]) moveForward += cameraSpeed;
    if (keyboardState[SDL_SCANCODE_S]) moveForward -= cameraSpeed;
    if (keyboardState[SDL_SCANCODE_A]) moveSide -= cameraSpeed;
    if (keyboardState[SDL_SCANCODE_D]) moveSide += cameraSpeed;
    if (keyboardState[SDL_SCANCODE_UP]) moveVertical += cameraSpeed;
    if (keyboardState[SDL_SCANCODE_DOWN]) moveVertical -= cameraSpeed;

    // Rotation around Y-axis (yaw)
    if (keyboardState[SDL_SCANCODE_Q]) yawRotation -= rotationSpeed;
    if (keyboardState[SDL_SCANCODE_E]) yawRotation += rotationSpeed;

    m_camera.Move(moveSide, moveForward, moveVertical);
    if (yawRotation != 0.0f) {
        m_camera.Rotate(yawRotation, 0.0f); // Yaw rotation only
    }
}


void MainLoop::handleMouseMotion(const SDL_Event& e, float deltaTime) {
    const float mouseSensitivity = 0.1f;

    float deltaX = static_cast<float>(e.motion.xrel);
    float deltaY = static_cast<float>(e.motion.yrel);

    m_camera.Rotate(-deltaX * mouseSensitivity, -deltaY * mouseSensitivity);
}
