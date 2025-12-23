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
    m_camera.LookAt({0.0f, 9.0f, 0.1f}, {0.0f, 0.0f, 0.0f});

}

void MainLoop::Run()
{
    using clock           = std::chrono::high_resolution_clock;
    using seconds_f       = std::chrono::duration<float>;
    constexpr seconds_f   kTargetFrame   { 1.0f / 60.0f };      // 16.666 ms

    auto lastFrameStart = clock::now();
    m_isRunning = true;

    float lastTime = 0.0f;
    bool running = true;

    int step = 0;

    while (m_isRunning)
    {
        /* ---------------- time & delta ---------------- */
        const auto thisFrameStart = clock::now();
        const float deltaTime     = seconds_f(thisFrameStart - lastFrameStart).count();
        lastFrameStart            = thisFrameStart;

        m_totalTime += deltaTime;

        /* ------- input, simulation, rendering --------- */
        handleEvents(step);
        update(deltaTime,step);

        float currentTime = (float)SDL_GetTicks() / 1000.0f;

        // Calculate delta time (time since last frame)
        float dt = currentTime - lastTime;

        // Update lastTime for the next frame
        lastTime = currentTime;

         m_graphicsModule.RenderFrame(m_camera, m_totalTime, dt, step);

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

        step++;
    }
}



void MainLoop::Shutdown() {

    m_graphicsModule.Shutdown();

    if (m_graphicsModule.getWindow()) {
        SDL_DestroyWindow(m_graphicsModule.getWindow());
    }
    SDL_Quit();
}

void MainLoop::handleEvents(int& step) {

    bool        relative = false;
    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {

        ImGui_ImplSDL3_ProcessEvent(&ev);


        switch (ev.type) {
        case SDL_EVENT_QUIT:
            m_isRunning = false;
            break;
        case SDL_EVENT_WINDOW_RESIZED:
        case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED:
            m_graphicsModule.SignalResize();
            step = 0;
            break;
        case SDL_EVENT_KEY_DOWN:
            if (ev.key.key == SDLK_ESCAPE) {
                m_isRunning = false;
            }
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
            if (ev.button.button == SDL_BUTTON_RIGHT) {   // still the same macro :contentReference[oaicite:2]{index=2}
                // query current state and flip it
                relative = !SDL_GetWindowRelativeMouseMode(m_graphicsModule.getWindow());
                SDL_SetWindowRelativeMouseMode(m_graphicsModule.getWindow(), relative);
                SDL_SetWindowMouseGrab(m_graphicsModule.getWindow(), relative);
                m_relativeMouseMode = relative;
                step = 0;
            }
            break;

        case SDL_EVENT_MOUSE_MOTION:
            // Check if ImGui wants to capture mouse
            if (ImGui::GetIO().WantCaptureMouse) break;

            // Right click: fly mode
            if (m_relativeMouseMode) {
                const bool* ks = SDL_GetKeyboardState(nullptr);
                const bool slow = ks[SDL_SCANCODE_LSHIFT] || ks[SDL_SCANCODE_RSHIFT];
                const float sensDeg = (slow ? (0.1f / 5.0f) : 0.1f); // deg per pixel

                m_camera.RotateYawPitchDeg(-ev.motion.xrel * sensDeg,
                                           -ev.motion.yrel * sensDeg);
                step = 0;
            }
            // Left click: orbit mode
            else if (ev.motion.state & SDL_BUTTON_LMASK) {
                const float orbitSens = 0.5f; 
                m_camera.Orbit(ev.motion.xrel * orbitSens, ev.motion.yrel * orbitSens);
                step = 0;
            }
            break;
        }
    }
}

void MainLoop::update(float deltaTime, int& step) {
    // If ImGui wants keyboard input, skip camera movement
    if (ImGui::GetIO().WantCaptureKeyboard) return;

    const bool* ks = SDL_GetKeyboardState(nullptr);
    const bool slow = ks[SDL_SCANCODE_LSHIFT] || ks[SDL_SCANCODE_RSHIFT];
    const float slowFactor = slow ? (1.0f / 5.0f) : 1.0f;

    const float cameraSpeed   = 5.0f  * slowFactor * deltaTime;  // move units/sec
    const float rotationSpeed = 60.0f * slowFactor * deltaTime;  // deg/sec

    float moveForward = 0.0f, moveSide = 0.0f, moveVertical = 0.0f;
    float yawRotation = 0.0f;

    if (ks[SDL_SCANCODE_W]) moveForward += cameraSpeed;
    if (ks[SDL_SCANCODE_S]) moveForward -= cameraSpeed;
    if (ks[SDL_SCANCODE_A]) moveSide    -= cameraSpeed;
    if (ks[SDL_SCANCODE_D]) moveSide    += cameraSpeed;
    
    // MoveVertical using R/F and Up/Down
    if (ks[SDL_SCANCODE_UP] || ks[SDL_SCANCODE_R])   moveVertical += cameraSpeed;
    if (ks[SDL_SCANCODE_DOWN] || ks[SDL_SCANCODE_F]) moveVertical -= cameraSpeed;

    if (ks[SDL_SCANCODE_Q]) yawRotation -= rotationSpeed;
    if (ks[SDL_SCANCODE_E]) yawRotation += rotationSpeed;

    if (moveSide || moveForward || moveVertical) {
        m_camera.Move(moveSide, moveForward, moveVertical);
        step = 0;
    }
    if (yawRotation != 0.0f) {
        m_camera.RotateYawPitchDeg(yawRotation, 0.0f);
        step = 0;
    }
}



void MainLoop::handleMouseMotion(const SDL_Event& e, float deltaTime) {
    const float mouseSensitivity = 0.1f;

    float deltaX = static_cast<float>(e.motion.xrel);
    float deltaY = static_cast<float>(e.motion.yrel);

    m_camera.Rotate(-deltaX * mouseSensitivity, -deltaY * mouseSensitivity);
}
