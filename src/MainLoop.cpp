#include "MainLoop.h"
#include "GraphicsModule.h" // Full definition included here
#include <stdexcept>
#include <chrono>
#include <iostream>

// Constructor and Destructor defined here where GraphicsModule is a complete type
MainLoop::MainLoop() = default;
MainLoop::~MainLoop() = default;

void MainLoop::Initialize(const std::string& title, uint32_t width, uint32_t height) {

    if (!SDL_Init(SDL_INIT_VIDEO))
        throw std::runtime_error(std::string("Failed to initialize SDL3: ") + SDL_GetError());

    m_window = SDL_CreateWindow(title.c_str(), width, height, SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE);
    if (!m_window) {
        throw std::runtime_error("SDL window creation failed: " + std::string(SDL_GetError()));
    }

    try {
        m_graphicsModule = std::make_unique<GraphicsModule>();
        m_graphicsModule->Initialize(m_window, title);
    } catch (const std::exception& e) {
        std::cerr << "Graphics initialization failed: " << e.what() << std::endl;
        SDL_DestroyWindow(m_window);
        SDL_Quit();
        throw;
    }

    m_camera.SetViewport({0, 0, (int)width, (int)height});
    m_camera.LookAt({0.0f, 1.0f, 3.5f}, {0.0f, 0.5f, 0.0f});
}

void MainLoop::Run()
{
    using clock = std::chrono::high_resolution_clock;

    auto last = clock::now();
    m_isRunning = true;

    while (m_isRunning)
    {
        auto  now       = clock::now();
        float deltaTime = std::chrono::duration<float>(now - last).count();
        last = now;

        /* ---------- обработка ввода и рендер ---------- */
        handleEvents();
        update(deltaTime);
        if (m_graphicsModule)
            m_graphicsModule->RenderFrame(m_camera);
        /* --------------------------------------------- */

        /* ---------- вывод FPS/мс за кадр -------------- */
        if (deltaTime > 0.0f)               // защита /0
        {
            const float fps  = 1.0f / deltaTime;
            const float mspp = deltaTime * 1000.0f;   // milliseconds per picture
            std::printf("FPS: %.1f  (%.2f ms)\n", fps, mspp);
            std::fflush(stdout);             // если запускаете из IDE — чтобы сразу видно
        }
    }
}



void MainLoop::Shutdown() {
    if (m_graphicsModule) {
        m_graphicsModule->Shutdown();
    }
    if (m_window) {
        SDL_DestroyWindow(m_window);
        m_window = nullptr;
    }
    SDL_Quit();
}

void MainLoop::handleEvents() {

    bool        relative = false;
    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
        switch (ev.type) {
        case SDL_EVENT_QUIT:
            m_isRunning = false;
            break;
        case SDL_EVENT_WINDOW_RESIZED:
        case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED:
            if (m_graphicsModule) m_graphicsModule->SignalResize();
            break;
        case SDL_EVENT_KEY_DOWN:
            if (ev.key.key == SDLK_ESCAPE) {
                m_isRunning = false;
            }
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
            if (ev.button.button == SDL_BUTTON_RIGHT) {   // still the same macro :contentReference[oaicite:2]{index=2}
                // query current state and flip it
                relative = !SDL_GetWindowRelativeMouseMode(m_window);     // SDL3 query :contentReference[oaicite:3]{index=3}
                SDL_SetWindowRelativeMouseMode(m_window, relative);       // enable / disable :contentReference[oaicite:4]{index=4}

                // (optional) lock the cursor to the window too
                SDL_SetWindowMouseGrab(m_window, relative);
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
    // CORRECTED: SDL3's SDL_GetKeyboardState returns const Uint8*
    const bool* keyboardState = SDL_GetKeyboardState(nullptr);

    float moveForward = 0.0f;
    float moveSide = 0.0f;

    if (keyboardState[SDL_SCANCODE_W]) moveForward += cameraSpeed;
    if (keyboardState[SDL_SCANCODE_S]) moveForward -= cameraSpeed;
    if (keyboardState[SDL_SCANCODE_A]) moveSide -= cameraSpeed;
    if (keyboardState[SDL_SCANCODE_D]) moveSide += cameraSpeed;

    m_camera.Move(moveSide, moveForward);
}
