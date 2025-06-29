#pragma once

#include "GraphicsModule.h" // This now forward-declares GraphicsModule
#include "framework/camera.h"
#include <SDL3/SDL.h>
#include <string>
#include <memory>

// Forward declare to break circular dependency and help with unique_ptr
class GraphicsModule;

class MainLoop {
public:
    MainLoop();  // Constructor
    ~MainLoop(); // DESTRUCTOR DECLARED HERE

    void Initialize(const std::string& title = "Vulkan Ray Tracing", uint32_t width = 1280, uint32_t height = 720);
    void Run();
    void Shutdown();

    void handleMouseMotion(const SDL_Event &e, float deltaTime);
private:
    void handleEvents();
    void update(float deltaTime);

    SDL_Window* m_window = nullptr;
    bool m_isRunning = true;

    // Use unique_ptr to an incomplete type to fully break the circular dependency
    // and solve the sizeof error chain.
    std::unique_ptr<GraphicsModule> m_graphicsModule;
    Camera m_camera;

    bool m_relativeMouseMode = false;
};

