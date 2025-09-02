#include "GraphicsModule.h"
#include "VulkanCheck.h"
#include "volk.h"
#include <imgui.h>

#include "RayTracingModule.h" // Full definition of RayTracingModule now included
#include <SDL3/SDL_vulkan.h>
#include <stdexcept>
#include <vector>
#include <iostream>
#include <algorithm>
#include "GeomCreate.h"
#include <fstream>

#include "CgalMeshBuilder.h"



// Forward declaration for the debug callback
static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
    VkDebugUtilsMessageTypeFlagsEXT messageType,
    const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
    void* pUserData);

// --- Constructor and Destructor Definitions ---
// These MUST be defined in the .cpp file where RayTracingModule is a complete type.
GraphicsModule::GraphicsModule() = default;
GraphicsModule::~GraphicsModule() = default;

// --- Public API Implementations ---

void GraphicsModule::Initialize(const std::string appName) {

    initSDL();
    initVulkan(appName);
    initRayTracingModule();
}

void GraphicsModule::Shutdown() {
    // Если устройство уже было уничтожено, выходим
    if (!m_device) return;

    // 1. Убеждаемся, что GPU закончил все операции.
    vkDeviceWaitIdle(m_device);

    // 2. Уничтожаем все ресурсы, созданные нашими модулями.
    //    unique_ptr сделает это автоматически при выходе из области видимости,
    //    но явный вызов Cleanup более нагляден.
    if (m_rtxModule) {
        m_rtxModule->Cleanup();
        m_rtxModule.reset(); // Уничтожаем объект
    }


    if (m_meshRenderer) {
        m_meshRenderer->Cleanup(m_device);
        m_meshRenderer.reset();
    }

    // 3. Уничтожаем все объекты, связанные со Swapchain.
    cleanupSwapchain();

    // 4. Уничтожаем объекты синхронизации и командный пул.
    //    Они были созданы из VkDevice.
    for (size_t i = 0; i < m_inFlightFences.size(); i++) {
        vkDestroySemaphore(m_device, m_renderFinishedSemaphores[i], nullptr);
        vkDestroySemaphore(m_device, m_imageAvailableSemaphores[i], nullptr);
        vkDestroyFence(m_device, m_inFlightFences[i], nullptr);
    }

    if (m_commandPool) {
        vkDestroyCommandPool(m_device, m_commandPool, nullptr);
    }

    // 5. После того как все дочерние объекты уничтожены, уничтожаем само логическое устройство.
    vkDestroyDevice(m_device, nullptr);
    m_device = VK_NULL_HANDLE; // Обнуляем хендл

    // 6. Теперь уничтожаем объекты уровня Instance, которые зависели от него.
    if (m_debugMessenger) {
        auto func = (PFN_vkDestroyDebugUtilsMessengerEXT)vkGetInstanceProcAddr(m_instance, "vkDestroyDebugUtilsMessengerEXT");
        if (func) {
            func(m_instance, m_debugMessenger, nullptr);
        }
    }

    if (m_surface) {
        vkDestroySurfaceKHR(m_instance, m_surface, nullptr);
    }

    // 7. В самую последнюю очередь уничтожаем сам инстанс Vulkan.
    if (m_instance) {
        vkDestroyInstance(m_instance, nullptr);
        m_instance = VK_NULL_HANDLE;
    }
}


void GraphicsModule::initSDL() {

    if (!SDL_Init(SDL_INIT_VIDEO))
        throw std::runtime_error(std::string("Failed to initialize SDL3: ") + SDL_GetError());

    m_window = SDL_CreateWindow("Vulkan raytracer exp", 800, 600, SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE);
    std::cout << "SDL Window created at: " << m_window << std::endl;
    assert(m_window && "SDL_CreateWindow returned NULL");
    if (!m_window)
        throw std::runtime_error("Failed to create SDL3 window");
}

// In GraphicsModule.cpp

void GraphicsModule::RenderFrame(const Camera& cam, float currentTime, float dt, int step) {
    // 1. Wait for the GPU to finish the frame that is currently "in flight"
    vkWaitForFences(m_device, 1, &m_inFlightFences[m_currentFrame], VK_TRUE, UINT64_MAX);

    // 2. Acquire an image from the swapchain
    uint32_t imageIndex;
    VkResult result = vkAcquireNextImageKHR(m_device, m_swapchain, UINT64_MAX, m_imageAvailableSemaphores[m_currentFrame], VK_NULL_HANDLE, &imageIndex);

    // Handle a resized/out-of-date swapchain
    if (result == VK_ERROR_OUT_OF_DATE_KHR) {
        recreateSwapchain();
        return;
    } else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
        VK_CHECK(result, "Failed to acquire swap chain image");
    }

    // NOW that we are sure we are rendering this frame, reset the fence for this frame.
    // This fence will be used by vkQueueSubmit to signal when the GPU is done.
    vkResetFences(m_device, 1, &m_inFlightFences[m_currentFrame]);

    // --- LOGIC AND ANIMATION UPDATE ---

    // 4. Update scene state based on the new time
    m_rtxModule->UpdateCamera(cam);
   // m_rtxModule->AnimateInstances(currentTime, /*orbitAroundWorldZ=*/true); // Update instance transforms and rebuild TLAS
   // m_rtxModule->AnimateRubik(dt);

    // 5. Update uniform data for shaders
    float pulse = (sin(currentTime * 2.0f) * 0.5f + 0.5f);
    float currentIntensity = 1.0f + pulse * 0.15f;
    glm::vec3 color = glm::vec3(0.8f, 0.85f, 0.8f);
    m_rtxModule->UpdateUniforms(currentTime, color, currentIntensity, step);

    // --- RECORDING AND SUBMISSION ---

    // 6. Reset and record the command buffer with the new, updated scene state
    vkResetCommandBuffer(m_commandBuffers[m_currentFrame], 0);
    recordCommandBuffer(imageIndex, cam); // Called only ONCE

    // 7. Submit the command buffer to the GPU
    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

    VkSemaphore waitSemaphores[] = { m_imageAvailableSemaphores[m_currentFrame] };
    VkPipelineStageFlags waitStages[] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT }; // Wait until it's safe to write to the image
    submitInfo.waitSemaphoreCount = 1;
    submitInfo.pWaitSemaphores = waitSemaphores;
    submitInfo.pWaitDstStageMask = waitStages;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &m_commandBuffers[m_currentFrame];

    VkSemaphore signalSemaphores[] = { m_renderFinishedSemaphores[m_currentFrame] };
    submitInfo.signalSemaphoreCount = 1;
    submitInfo.pSignalSemaphores = signalSemaphores;

    VK_CHECK(vkQueueSubmit(m_graphicsQueue, 1, &submitInfo, m_inFlightFences[m_currentFrame]), "Failed to submit draw command buffer");

    // 8. Present the rendered image to the screen
    VkPresentInfoKHR presentInfo{};
    presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pWaitSemaphores = signalSemaphores; // Wait for rendering to be finished
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = &m_swapchain;
    presentInfo.pImageIndices = &imageIndex;

    result = vkQueuePresentKHR(m_graphicsQueue, &presentInfo);

    // Handle resizing at the end of the frame
    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || m_framebufferResized) {
        m_framebufferResized = false;
        recreateSwapchain();
    } else if (result != VK_SUCCESS) {
        VK_CHECK(result, "Failed to present swap chain image");
    }

    // 9. Advance to the next frame index
    m_currentFrame = (m_currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
}

void GraphicsModule::SignalResize() {
    m_framebufferResized = true;
}


// --- Private Method Implementations ---

void GraphicsModule::initVulkan(const std::string& appName) {
    // Guard: prevent double-initialization
    if (m_device != VK_NULL_HANDLE || m_swapchain != VK_NULL_HANDLE || m_instance != VK_NULL_HANDLE) {
        throw std::runtime_error("[GraphicsModule] initVulkan called twice or already initialized!");
    }

    // Optional: assign instance ID for debug
    static int s_instanceCounter = 0;
    instanceId = ++s_instanceCounter;
    std::cout << "[GraphicsModule] initVulkan() called for instance ID: " << instanceId << " @ " << this << std::endl;

    VK_CHECK(volkInitialize(), "Failed to initialize Volk");

    createInstance(appName);
    volkLoadInstance(m_instance);

    setupDebugMessenger();
    createSurface();
    std::cout << "[Vulkan] Surface created: " << m_surface << std::endl;

    pickPhysicalDevice();
    findQueueFamilies();
    createLogicalDevice();
    volkLoadDevice(m_device);

    createCommandPool();
    createSwapchain(VK_NULL_HANDLE);
    createImageViews();
    createRenderPass();

    createFramebuffers();

    //createGraphicsPipeline();
    initImgui();  // sets up descriptor pool, context, SDL bridge, etc.

    initRasterRenderers();

    std::cout << "Window @GraphicsModule: " << m_window << std::endl;

    createSyncObjects();
}

/* ---------------------- initImGui --------------------------- */
void GraphicsModule::initImgui()
{
    m_imguiModule.init(m_window,
                     m_instance,
                     m_physicalDevice,
                     m_device,
                     m_graphicsQueue,
                     m_graphicsQueueFamilyIndex,
                     m_swapchainFormat,
                     m_swapchainExtent,
                     m_swapchainImageViews,
                     m_renderPass);
}


void GraphicsModule::initRayTracingModule() {

    vulkanhelpers::VulkanContext context;
    context.device = m_device;
    context.physicalDevice = m_physicalDevice;
    context.commandPool = m_commandPool;
    context.transferQueue = m_graphicsQueue;
    vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &context.physicalDeviceMemoryProperties);

    m_rtxModule = std::make_unique<rtx::RayTracingModule>();
    rtx::RayTracingModule::CreateInfo ci{};
    ci.device = m_device;
    ci.physicalDevice = m_physicalDevice;
    ci.commandPool = m_commandPool;
    ci.graphicsQueue = m_graphicsQueue;
    ci.shaderDir = "shaders/";
    m_rtxModule->Initialize(context, ci);

    CreateScene();
}

void GraphicsModule::CreateScene() {
    // 1) Geometry
    std::vector<Vertex> sphereVertices; std::vector<uint32_t> sphereIndices;
    GeomCreate::createIcosphere(4, sphereVertices, sphereIndices);


    // 1) start with polygonal cube
    SurfaceMesh sm;
    //CgalMeshBuilder::buildCube(sm, /*size*/ 1.0);
    CgalMeshBuilder::buildHollowCuboid(sm, /*N*/ 3, /*M*/ 3, /*L*/ 3, /*cellSize*/ 0.5);
    //CgalMeshBuilder::buildPlaneXY(sm, /*N*/3,/*M*/3,  /*cellSize*/ 0.5);

    //CgalMeshBuilder::buildCubeWithGrid(sm, /*size*/1.0, /*nx*/1, /*ny*/1);

    const int N = 2;
    //CgalMeshBuilder::subdivideQuadFacesGrid(sm, N, N);

   // CgalMeshBuilder::catmullClarkRefineNoSmooth(sm);
//CgalMeshBuilder::catmullClarkRefine_NoInterp(sm, /*keep_borders=*/false);
//    CgalMeshBuilder::catmullClarkRefine_NoInterp(sm, /*keep_borders=*/false);

    // 2) choose faces (e.g., 60% for extrude, 15% for delete)
    auto all     = CgalMeshBuilder::selectFacesRandom(sm, 1.0, 1337);


    // 3A) delete some faces
    auto count_faces = [&](const SurfaceMesh& m){
        std::size_t c=0; for (auto f: m.faces()) if(!m.is_removed(f)) ++c; return c;
    };

    auto face_stats = [&](const SurfaceMesh& m){
        size_t F=0, tri=0, quad=0, poly=0;
        for (auto f: m.faces()) if(!m.is_removed(f)) {
                ++F;
                int k=0; for (auto h: CGAL::halfedges_around_face(m.halfedge(f), m)) ++k;
                if (k==3) ++tri; else if (k==4) ++quad; else ++poly;
            }
        std::cerr << "faces="<<F<<"  tris="<<tri<<"  quads="<<quad<<"  polys="<<poly<<"\n";
    };


    {
        auto toExtr  = CgalMeshBuilder::selectFacesRandom(sm, 1.0, 4242);
        auto res = CgalMeshBuilder::extrudeFaces_collectBoth(sm, toExtr, 0.15, 0.9);
        auto res2 = CgalMeshBuilder::extrudeFaces_collectBoth(sm, res, 0.0, 0.75);
        auto res3 = CgalMeshBuilder::extrudeFaces_collectBoth(sm, res2, -0.1, 0.25);
    }

   // auto res = CgalMeshBuilder::extrudeFaces_collectBoth(sm, toExtr, 0.15, 0.9);


  //  auto res2 = CgalMeshBuilder::extrudeFaces_collectBoth(sm, res.caps, 0.0, 0.7);
  //  auto res3 = CgalMeshBuilder::extrudeFaces_collectBoth(sm, res2.caps, -0.1, 0.25);

    //auto res = CgalMeshBuilder::extrudeRegion(sm, toExtr,1.0, 4242);
    //CgalMeshBuilder::cleanup_after_deletions(sm);

   // auto toExtr  = CgalMeshBuilder::selectFacesRandom(sm, 0.5, 4242);
  //  auto res = CgalMeshBuilder::extrudeFaces(sm, toExtr, 0.15, 0.9);

    std::cerr << "faces before del: " << count_faces(sm) << "\n";
    face_stats(sm);
    auto toDel   = CgalMeshBuilder::selectFacesRandom(sm, 0.25, 7777);
    //CgalMeshBuilder::deleteFaces(sm, toDel, true);
    std::cerr << "faces after  del: " << count_faces(sm) << "\n";
    face_stats(sm);
    std::cerr << "selected: " << toDel.size() << "\n";  // you’ll likely see 2
    //CgalMeshBuilder::cleanup_after_deletions(sm);

        sm.collect_garbage();
    {
       // auto toExtr  = CgalMeshBuilder::selectFacesRandom(sm, 1.0, 4242);
       // auto res =  CgalMeshBuilder::extrudeFaces_collectBoth(sm, toExtr, 0.1, 0.9);

        //auto toExtr  = CgalMeshBuilder::selectFaceByIndex(sm, 4);
        //auto res =  CgalMeshBuilder::extrudeFaces_collectBoth(sm, toExtr, 0.1, 0.9);

       // {
       //     auto toExtr  = CgalMeshBuilder::selectFaceByIndex(sm,0);
       //     auto res =  CgalMeshBuilder::extrudeFaces_collectBoth(sm, toExtr, 0.1, 0.9);

       // }


        //auto res2 = CgalMeshBuilder::extrudeFaces_collectBoth(sm, res, 0.0, 0.5);
        //auto res3 = CgalMeshBuilder::extrudeFaces_collectBoth(sm, res2, -0.1, 0.25);
    }

    sm.collect_garbage();


    //CgalMeshBuilder::cleanup_after_deletions(sm);

     //   CgalMeshBuilder::cleanup_after_deletions(sm);
   // CgalMeshBuilder::applyCatmullClark(sm, 1, /*keep_borders=*/true);
    // 4) triangulate as a separate step

    // Densify a bit so rims have more verts to shape


    // Make each hole rim round-ish (optional)
    //CgalMeshBuilder::circularizeBorderLoops(sm, 1.0);

    // **Fillet**: push K rings from each rim with smooth falloff
    //CgalMeshBuilder::filletBorderLoops(sm,
    //                                   /*rings=*/5,          // try 4–8
    //                                   /*height=*/0.06,      // try 0.03–0.12 relative to cube size 1
    //                                   /*outward=*/true,
    //                                   /*sharpness=*/1.2);

    // Optional extra CC for overall softness

    //  CgalMeshBuilder::cleanup_after_deletions(sm);

    CgalMeshBuilder::applyCatmullClark(sm, 4, /*keep_borders=*/true);

    // Triangulate → export


    std::vector<Vertex> heLines;
    CgalMeshBuilder::buildHalfedgeArrows(sm, heLines, /*inset*/0.017f, /*head*/0.08f, true);

    face_stats(sm);
    CgalMeshBuilder::triangulateAll(sm);
    face_stats(sm);

    std::vector<Vertex>   cubeVertices;
    std::vector<uint32_t> cubeIndices;
    CgalMeshBuilder::toVertexIndexFlat(sm, cubeVertices, cubeIndices);


    // 2) Instances (one list per mesh)
    std::vector<rtx::InstanceData> sphereInstances;
    std::vector<rtx::InstanceData> cubeInstances;

    const int   gridSize = 2;
    const float spacing  = 2.75f;

    // базовые масштабы
    const float baseSphereScale = 0.70f;
    const float baseCubeScale   = 1.25f;

    const float specialScale    = 0.10f;

    // -------- PASS 1: просто собираем трансформы с базовым масштабом ----------
    //for (int z = -gridSize; z <= gridSize; ++z) {
    //    for (int y = -gridSize; y <= gridSize; ++y) {
    //        for (int x = -gridSize; x <= gridSize; ++x) {
    //            glm::vec3 pos = { x * spacing, y * spacing, z * spacing };
    //            glm::mat4 M   = glm::translate(glm::mat4(1.f), pos);

    //            // Центр — большая сфера
    //            if (x == 0 && y == 0 && z == 0) {
    //                //sphereInstances.push_back({ glm::scale(M, glm::vec3(0.25f)) });
    //                //M = M * glm::rotate(glm::mat4(1.f), glm::radians(0.f * float(x + y + z)),
    //                //                    glm::vec3(0, 1, 0));
    //                //cubeInstances.push_back({ glm::scale(M, glm::vec3(0.75f *baseCubeScale)) });
    //                //M = M * glm::rotate(glm::mat4(1.f), glm::radians(0.f * float(x + y + z)),
    //                //                    glm::vec3(0, 1, 0));
    //                //cubeInstances.push_back({ glm::scale(M, glm::vec3(0.5f *baseCubeScale)) });
    //                //cubeInstances.push_back({ glm::scale(M, glm::vec3(baseCubeScale)) });
    //                continue;
    //            }

    //            if (x==0 && y == 0 && z == 1) {
    //                //   sphereInstances.push_back({ glm::scale(M, glm::vec3(0.75f)) });
    //                  continue;
    //            }

    //            //const bool placeSphere = ((x + y + z) & 1) == 0;
    //            //if (placeSphere) {
    //            //    sphereInstances.push_back({ glm::scale(M, glm::vec3(baseSphereScale)) });
    //            //    cubeInstances.push_back({ glm::scale(M, glm::vec3(0.5f * baseCubeScale)) });

    //            //}
    //            if (x!=0 || y != 0)
    //            {
    //                // немного повернём кубики для разнообразия
    //                M = M * glm::rotate(glm::mat4(1.f), glm::radians(0.f * float(x + y + z)),
    //                                    glm::vec3(0, 1, 0));
    //                cubeInstances.push_back({ glm::scale(M, glm::vec3(baseCubeScale)) });
    //                //auto M1 = M * glm::rotate(glm::mat4(1.f), glm::radians(15.f * float(x + y + z)),
    //                //                    glm::vec3(0, 1, 0));
    //                //cubeInstances.push_back({ glm::scale(M1, glm::vec3(0.5f * baseCubeScale)) });

    //                //sphereInstances.push_back({ glm::scale(M, glm::vec3(0.5f * baseSphereScale)) });
    //            }
    //        }
    //    }
    //}
    glm::vec3 pos = { 0.0f, 0.0f, 0.0f };
    glm::mat4 M   = glm::translate(glm::mat4(1.f), pos);
    M = M * glm::rotate(glm::mat4(1.f), glm::radians(0.0f), glm::vec3(0, 1, 0));
    cubeInstances.push_back({ glm::scale(M, glm::vec3(baseCubeScale)) });
   // sphereInstances.push_back({ glm::scale(M, glm::vec3(0.45f * baseSphereScale)) });
    // Сколько сфер получилось (они пойдут первыми и получат uniqueID = [0..numSpheres-1])
    const uint32_t numSpheres = static_cast<uint32_t>(sphereInstances.size());

    // a) Сферы: uniqueID == sphereIndex
    for (uint32_t si = 0; si < numSpheres; ++si) {
        if (si != 0 && (si % 27u) == 0u) {
            const float k = specialScale / baseSphereScale;
            //sphereInstances[si].transform =
            //    sphereInstances[si].transform * glm::scale(glm::mat4(1.f), glm::vec3(k));
        }
    }

    // b) Кубы: uniqueID == numSpheres + cubeIndex
    for (uint32_t ci = 0; ci < cubeInstances.size(); ++ci) {
        const uint32_t uid = numSpheres + ci;
        if ((uid % 27u) == 0u) {
            const float k = specialScale / baseCubeScale;
            //cubeInstances[ci].transform =
            //    cubeInstances[ci].transform * glm::scale(glm::mat4(1.f), glm::vec3(k));
        }
    }

    // 3) Upload: meshId 0 = sphere, meshId 1 = cube (ВАЖЕН порядок!)
    m_rtxModule->LoadFromMultipleMeshes({
        { sphereVertices, sphereIndices, sphereInstances }, // meshId 0
        { cubeVertices,   cubeIndices,   cubeInstances   }  // meshId 1
    });


    if (m_meshRenderer) {
        //m_meshRenderer->SetMesh(cubeVertices, cubeIndices);

        // Пример линий (для топологии/halfedges):
        // std::vector<glm::vec3> dbgLines = ...;
        // m_meshRenderer->SetLines(dbgLines, {1,0,0});


        m_meshRenderer->SetColoredLines(heLines);
    }
}



void GraphicsModule::recreateSwapchain() {
    int width = 0, height = 0;

    SDL_ShowWindow(m_window);
    SDL_RaiseWindow(m_window);

    SDL_GetWindowSizeInPixels(m_window, &width, &height);
    while (width == 0 || height == 0) {
        SDL_GetWindowSizeInPixels(m_window, &width, &height);
        SDL_WaitEvent(nullptr);
    }

    vkDeviceWaitIdle(m_device);

    VkSwapchainKHR oldSwapchain = m_swapchain;

    // This now cleans up framebuffers and the render pass as well
    cleanupSwapchain();

    vkQueueWaitIdle(m_graphicsQueue);
    // Recreate the full stack
    createSwapchain(oldSwapchain);      // Recreates the swapchain and its images
    createImageViews();     // Re-populates m_swapchainImageViews
    createRenderPass();     // **KEY:** Recreate the render pass
    createFramebuffers();   // **KEY:** Recreate framebuffers for the new image views

    if (m_meshRenderer)
        m_meshRenderer->OnResize(m_device, m_renderPass);

    // Notify other modules that depend on the swapchain size/images
    if (m_rtxModule) {
        m_rtxModule->OnResize(m_swapchainExtent);
    }



    // You'll likely need to notify ImGui as well so it can recreate its
    // own framebuffers and pipelines if they depend on the swapchain.
    // m_imguiModule.OnResize(...);
}

void GraphicsModule::cleanupSwapchain() {
    for (auto framebuffer : m_framebuffers) {
        vkDestroyFramebuffer(m_device, framebuffer, nullptr);
    }
    m_framebuffers.clear();


    if (m_graphicsPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(m_device, m_graphicsPipeline, nullptr);
        m_graphicsPipeline = VK_NULL_HANDLE;
    }

    if (m_pipelineLayout != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(m_device, m_pipelineLayout, nullptr);
        m_pipelineLayout = VK_NULL_HANDLE;
    }

    if (m_renderPass != VK_NULL_HANDLE) {
        vkDestroyRenderPass(m_device, m_renderPass, nullptr);
        m_renderPass = VK_NULL_HANDLE;
    }

    for (auto imageView : m_swapchainImageViews) {
        vkDestroyImageView(m_device, imageView, nullptr);
    }
    m_swapchainImageViews.clear();

    // 6. ---- УДАЛИТЕ ЭТОТ БЛОК ----
    // НЕ НУЖНО уничтожать swapchain здесь, если вы используете oldSwapchain.
    // Драйвер сделает это сам.
    /*
    if (m_swapchain != VK_NULL_HANDLE) {
        vkDestroySwapchainKHR(m_device, m_swapchain, nullptr);
        m_swapchain = VK_NULL_HANDLE;
    }
    */
}


void GraphicsModule::recordCommandBuffer(uint32_t imageIndex, const Camera& cam) {
    VkCommandBuffer cmd = m_commandBuffers[m_currentFrame];

    VkCommandBufferBeginInfo beginInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    VK_CHECK(vkBeginCommandBuffer(cmd, &beginInfo), "Failed to begin recording command buffer");


    if(!solverParams.drawPolyMesh)
    {
        // --- 1. Execute Ray Tracing ---
        // This will trace the scene into an internal storage image and then
        // copy the final result into the swapchain image.
        // The final barrier in RecordCommands leaves the swapchain image in VK_IMAGE_LAYOUT_PRESENT_SRC_KHR.
        m_rtxModule->RecordCommands(
            cmd,
            m_swapchainImageViews[imageIndex], // Target view
            m_swapchainImages[imageIndex],     // Target image
            m_swapchainExtent
            );

        // --- 2. Render ImGui on top of the Ray-Traced Image ---
        // The previous call left the swapchain image in PRESENT_SRC_KHR layout.
        // The render pass for ImGui requires it to be in COLOR_ATTACHMENT_OPTIMAL.
        // We must insert a barrier to handle this transition.

        VkImageMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        // No need to specify old/new queues if they are the same
        barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; // The last operation was a copy (transfer)
        barrier.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT; // Next operation is rendering
        //barrier.oldLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR; // IMPORTANT: The layout after the rtx copy
        barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        barrier.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        barrier.image = m_swapchainImages[imageIndex];
        barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        barrier.subresourceRange.baseMipLevel = 0;
        barrier.subresourceRange.levelCount = 1;
        barrier.subresourceRange.baseArrayLayer = 0;
        barrier.subresourceRange.layerCount = 1;

        vkCmdPipelineBarrier(
            cmd,
            VK_PIPELINE_STAGE_TRANSFER_BIT,           // Wait for the copy to finish
            VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, // Before color output stage
            0,
            0, nullptr,
            0, nullptr,
            1, &barrier
            );
      }
        // Now, begin the render pass for ImGui
        VkClearValue clearColor = { {{0.0f, 0.0f, 0.0f, 1.0f}} };
        VkRenderPassBeginInfo renderPassInfo{ VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };
        renderPassInfo.renderPass = m_renderPass;
        renderPassInfo.framebuffer = m_framebuffers[imageIndex];
        renderPassInfo.renderArea.offset = { 0, 0 };
        renderPassInfo.renderArea.extent = m_swapchainExtent;
        renderPassInfo.clearValueCount = 1;
        renderPassInfo.pClearValues = &clearColor; // This value is now ignored due to the loadOp change below

    vkCmdBeginRenderPass(cmd, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

    // --- Debug mesh (raster) ---
    if (m_meshRenderer && solverParams.drawPolyMesh) {
        PushConstants pc{};
        const glm::mat4 model = glm::mat4(1.0f);

        // Камера: используем твоё API
        const glm::mat4 view = cam.GetTransform();     // view matrix
        const glm::mat4 proj = cam.GetProjection();    // projection matrix

        // Структура PushConstants из HelpStructures.h: { mvp, model }
        pc.mvp   = proj * view * model;
        pc.model = model;

        vkCmdPushConstants(cmd, m_meshRenderer->pipelineLayout(),
                           VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PushConstants), &pc);

        // viewport/scissor внутри Draw()
        m_meshRenderer->Draw(cmd, m_swapchainExtent);
    }

    // Draw the UI
    m_imguiModule.renderMenu(cmd, solverParams);

    vkCmdEndRenderPass(cmd);

    // The render pass's finalLayout will automatically transition the image
    // back to VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, so we are done.

    VK_CHECK(vkEndCommandBuffer(cmd), "Failed to record command buffer");
}

/* ------------------------------------------------------------
 * GraphicsModule::createInstance
 * ---------------------------------------------------------- */
// GraphicsModule.cpp
void GraphicsModule::createInstance(const std::string& appName)
{

    /* ------------------------------------------------- *
     * 1.  Базовая «визитка» приложения                  *
     * ------------------------------------------------- */
    VkApplicationInfo appInfo{ VK_STRUCTURE_TYPE_APPLICATION_INFO };
    appInfo.pApplicationName   = appName.c_str();
    appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.pEngineName        = "No Engine";
    appInfo.engineVersion      = VK_MAKE_VERSION(1, 0, 0);
    appInfo.apiVersion         = VK_API_VERSION_1_3;


    /* ------------------------------------------------- *
     * 2.  Расширения, которые требуют SDL + Debug       *
     * ------------------------------------------------- */
    uint32_t sdlExtCount = 0;
    const char* const* sdlExt = SDL_Vulkan_GetInstanceExtensions(&sdlExtCount);
    if (!sdlExt)
        throw std::runtime_error("SDL_Vulkan_GetInstanceExtensions failed");

    std::vector<const char*> instExt{ sdlExt, sdlExt + sdlExtCount };

#ifndef NDEBUG
    instExt.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    //instExt.push_back(VK_EXT_VALIDATION_FEATURES_EXTENSION_NAME);   // цепочка pNext
#endif


    /* ------------------------------------------------- *
     * 3.  (Debug)  Messenger + validation-features chain *
     * ------------------------------------------------- */
#ifndef NDEBUG
    /* 3.1 Debug messenger */
    VkDebugUtilsMessengerCreateInfoEXT dbgInfo{
                                               VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT };
    dbgInfo.messageSeverity =
        VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
        VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    dbgInfo.messageType =
        VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT      |
        VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT   |
        VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
    dbgInfo.pfnUserCallback = debugCallback;

    /* 3.2 Validation-features (можно на лету конфигурировать) */
    std::vector<VkValidationFeatureEnableEXT> enableList = {
        VK_VALIDATION_FEATURE_ENABLE_BEST_PRACTICES_EXT
    };
#ifdef ENABLE_GPU_ASSISTED             // добавьте -DENABLE_GPU_ASSISTED в cmake/Makefile
    enableList.push_back(VK_VALIDATION_FEATURE_ENABLE_GPU_ASSISTED_EXT);
#endif

    VkValidationFeaturesEXT valFeatures{
                                        VK_STRUCTURE_TYPE_VALIDATION_FEATURES_EXT };
    valFeatures.enabledValidationFeatureCount =
        static_cast<uint32_t>(enableList.size());
    valFeatures.pEnabledValidationFeatures = enableList.data();

    /* сцепляем pNext-цепочку: dbgInfo → valFeatures → nullptr */
    dbgInfo.pNext = &valFeatures;
#endif  // !NDEBUG


    /* ------------------------------------------------- *
     * 4.  Сам VkInstance                                *
     * ------------------------------------------------- */
    const char* validationLayer = "VK_LAYER_KHRONOS_validation";

    VkInstanceCreateInfo ci{ VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO };
#ifndef NDEBUG
    ci.pNext = &dbgInfo;                        // отладочная цепочка
    ci.enabledLayerCount       = 1;
    ci.ppEnabledLayerNames     = &validationLayer;
#else
    ci.pNext = nullptr;
    ci.enabledLayerCount       = 0;
    ci.ppEnabledLayerNames     = nullptr;
#endif
    ci.pApplicationInfo        = &appInfo;
    ci.enabledExtensionCount   = static_cast<uint32_t>(instExt.size());
    ci.ppEnabledExtensionNames = instExt.data();

    VK_CHECK(vkCreateInstance(&ci, nullptr, &m_instance),
             "Failed to create Vulkan instance");
}



void GraphicsModule::setupDebugMessenger() {
    VkDebugUtilsMessengerCreateInfoEXT createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
    createInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    createInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
    createInfo.pfnUserCallback = debugCallback;

    auto func = (PFN_vkCreateDebugUtilsMessengerEXT)vkGetInstanceProcAddr(m_instance, "vkCreateDebugUtilsMessengerEXT");
    if (func != nullptr) {
        VK_CHECK(func(m_instance, &createInfo, nullptr, &m_debugMessenger), "Failed to set up debug messenger");
    }
}

void GraphicsModule::createSurface() {
    if (!SDL_Vulkan_CreateSurface(m_window, m_instance, nullptr, &m_surface)) {
        throw std::runtime_error("Failed to create window surface: " + std::string(SDL_GetError()));
    }
}


void GraphicsModule::pickPhysicalDevice() {
    uint32_t deviceCount = 0;
    vkEnumeratePhysicalDevices(m_instance, &deviceCount, nullptr);
    if (deviceCount == 0) {
        throw std::runtime_error("Failed to find GPUs with Vulkan support!");
    }
    std::vector<VkPhysicalDevice> devices(deviceCount);
    vkEnumeratePhysicalDevices(m_instance, &deviceCount, devices.data());

    for (const auto& device : devices) {
        m_physicalDevice = device;
        return;
    }
    throw std::runtime_error("Failed to find a suitable GPU!");
}


void GraphicsModule::findQueueFamilies() {
    uint32_t queueFamilyCount = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(m_physicalDevice, &queueFamilyCount, nullptr);
    std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
    vkGetPhysicalDeviceQueueFamilyProperties(m_physicalDevice, &queueFamilyCount, queueFamilies.data());

    for (uint32_t i = 0; i < queueFamilies.size(); ++i) {
        if (queueFamilies[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
            m_graphicsQueueFamilyIndex = i;
            return;
        }
    }
    throw std::runtime_error("Failed to find a queue family supporting graphics.");
}


void GraphicsModule::createLogicalDevice() {
    VkDeviceQueueCreateInfo queueCreateInfo{};
    queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    queueCreateInfo.queueFamilyIndex = m_graphicsQueueFamilyIndex;
    queueCreateInfo.queueCount = 1;
    float queuePriority = 1.0f;
    queueCreateInfo.pQueuePriorities = &queuePriority;


    // 1. Объявляем нужные структуры. Обратите внимание, bdaFeatures больше нет.
    VkPhysicalDeviceVulkan12Features features12{};
    VkPhysicalDeviceRayTracingPipelineFeaturesKHR rtpFeatures{};
    VkPhysicalDeviceAccelerationStructureFeaturesKHR asFeatures{};
    VkPhysicalDeviceFeatures2 deviceFeatures2{};

    // 2. Заполняем их, выстраивая цепочку pNext

    // Это теперь конец цепочки, так как bdaFeatures удалена
    rtpFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_FEATURES_KHR;
    rtpFeatures.rayTracingPipeline = VK_TRUE;
    rtpFeatures.pNext = nullptr; // <-- ИЗМЕНЕНО

    asFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR;
    asFeatures.accelerationStructure = VK_TRUE;
    asFeatures.pNext = &rtpFeatures;

    // Включаем ОБЕ нужные нам возможности в одной структуре Vulkan 1.2
    features12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
    features12.runtimeDescriptorArray = VK_TRUE;    // Эта возможность была нужна раньше
    features12.bufferDeviceAddress = VK_TRUE;       // <-- А эту мы сюда ПЕРЕНЕСЛИ
    features12.pNext = &asFeatures;

    deviceFeatures2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
    deviceFeatures2.pNext = &features12;

    std::vector<const char*> deviceExtensions = {
        VK_KHR_SWAPCHAIN_EXTENSION_NAME,
        VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME,
        VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME,
        VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME,
        //VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME,
    };

    VkDeviceCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    createInfo.pNext = &deviceFeatures2;
    createInfo.queueCreateInfoCount = 1;
    createInfo.pQueueCreateInfos = &queueCreateInfo;
    createInfo.enabledExtensionCount = static_cast<uint32_t>(deviceExtensions.size());
    createInfo.ppEnabledExtensionNames = deviceExtensions.data();

    VK_CHECK(vkCreateDevice(m_physicalDevice, &createInfo, nullptr, &m_device), "Failed to create logical device!");
    vkGetDeviceQueue(m_device, m_graphicsQueueFamilyIndex, 0, &m_graphicsQueue);
}


void GraphicsModule::createCommandPool() {
    VkCommandPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    poolInfo.queueFamilyIndex = m_graphicsQueueFamilyIndex;
    VK_CHECK(vkCreateCommandPool(m_device, &poolInfo, nullptr, &m_commandPool), "Failed to create command pool");

    //VkCommandPoolCreateInfo poolInfo{ VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
    //poolInfo.queueFamilyIndex = m_graphicsQueueFamilyIndex;
    //poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

    //if (vkCreateCommandPool(m_device, &poolInfo, nullptr, &m_commandPool) != VK_SUCCESS)
    //    throw std::runtime_error("Failed to create command pool");

    //m_commandBuffers.resize(m_swapchainImageViews.size());

    //VkCommandBufferAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    //allocInfo.commandPool = m_commandPool;
    //allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    //allocInfo.commandBufferCount = static_cast<uint32_t>(m_commandBuffers.size());

    //if (vkAllocateCommandBuffers(m_device, &allocInfo, m_commandBuffers.data()) != VK_SUCCESS)
    //    throw std::runtime_error("Failed to allocate command buffers");
}


void GraphicsModule::createSwapchain(VkSwapchainKHR oldSwapchain) {
    VkSurfaceCapabilitiesKHR capabilities;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(m_physicalDevice, m_surface, &capabilities);

    uint32_t formatCount;
    vkGetPhysicalDeviceSurfaceFormatsKHR(m_physicalDevice, m_surface, &formatCount, nullptr);
    std::vector<VkSurfaceFormatKHR> formats(formatCount);
    if(formatCount != 0) {
        vkGetPhysicalDeviceSurfaceFormatsKHR(m_physicalDevice, m_surface, &formatCount, formats.data());
    }

    VkSurfaceFormatKHR surfaceFormat = formats[0];
    for (const auto& availableFormat : formats) {
        if (availableFormat.format == VK_FORMAT_R8G8B8A8_SRGB && availableFormat.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
            surfaceFormat = availableFormat;
            break;
        }
    }
    m_swapchainFormat = surfaceFormat.format;

    if (capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max()) {
        m_swapchainExtent = capabilities.currentExtent;
    } else {
        int width, height;
        SDL_GetWindowSizeInPixels(m_window, &width, &height);
        m_swapchainExtent.width = std::clamp(static_cast<uint32_t>(width), capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
        m_swapchainExtent.height = std::clamp(static_cast<uint32_t>(height), capabilities.minImageExtent.height, capabilities.maxImageExtent.height);
    }

    uint32_t imageCount = capabilities.minImageCount + 1;
    if (capabilities.maxImageCount > 0 && imageCount > capabilities.maxImageCount) {
        imageCount = capabilities.maxImageCount;
    }

    VkSwapchainCreateInfoKHR createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    createInfo.surface = m_surface;
    createInfo.minImageCount = imageCount;
    createInfo.imageFormat = m_swapchainFormat;
    createInfo.imageColorSpace = surfaceFormat.colorSpace;
    createInfo.imageExtent = m_swapchainExtent;
    createInfo.imageArrayLayers = 1;
    createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    createInfo.preTransform = capabilities.currentTransform;
    createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    createInfo.presentMode = VK_PRESENT_MODE_IMMEDIATE_KHR; //VK_PRESENT_MODE_FIFO_KHR;
    createInfo.clipped = VK_TRUE;

    createInfo.oldSwapchain = oldSwapchain;

    // 1) present modes
    uint32_t presentModeCount = 0;
    vkGetPhysicalDeviceSurfacePresentModesKHR(m_physicalDevice, m_surface, &presentModeCount, nullptr);
    std::vector<VkPresentModeKHR> presentModes(presentModeCount);
    vkGetPhysicalDeviceSurfacePresentModesKHR(m_physicalDevice, m_surface, &presentModeCount, presentModes.data());

    // 2) выбрать режим (FIFO — обязателен по стандарту)
    auto choosePresentMode = [&]()->VkPresentModeKHR {
        VkPresentModeKHR mode = VK_PRESENT_MODE_FIFO_KHR;           // дефолт/всегда есть
        // если хочешь без vsync — попробуем IMMEDIATE, если поддерживается:
        if (std::find(presentModes.begin(), presentModes.end(), VK_PRESENT_MODE_IMMEDIATE_KHR) != presentModes.end())
            mode = VK_PRESENT_MODE_IMMEDIATE_KHR;
        // или prefer MAILBOX:
        // if (std::find(presentModes.begin(), presentModes.end(), VK_PRESENT_MODE_MAILBOX_KHR) != presentModes.end())
        //     mode = VK_PRESENT_MODE_MAILBOX_KHR;
        return mode;
    }();
    createInfo.presentMode = choosePresentMode;

    VK_CHECK(vkCreateSwapchainKHR(m_device, &createInfo, nullptr, &m_swapchain), "Failed to create swap chain");

    vkGetSwapchainImagesKHR(m_device, m_swapchain, &imageCount, nullptr);
    m_swapchainImages.resize(imageCount);
    vkGetSwapchainImagesKHR(m_device, m_swapchain, &imageCount, m_swapchainImages.data());

    m_swapchainImageViews.resize(m_swapchainImages.size());
    for (size_t i = 0; i < m_swapchainImages.size(); i++) {
        VkImageViewCreateInfo viewInfo{};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = m_swapchainImages[i];
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = m_swapchainFormat;
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.layerCount = 1;
        VK_CHECK(vkCreateImageView(m_device, &viewInfo, nullptr, &m_swapchainImageViews[i]), "Failed to create image views");
    }
}

void GraphicsModule::createImageViews() {
    m_swapchainImageViews.resize(m_swapchainImages.size());
    for (size_t i = 0; i < m_swapchainImages.size(); ++i) {
        VkImageViewCreateInfo viewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        viewInfo.image = m_swapchainImages[i];
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = m_swapchainFormat;
        viewInfo.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewInfo.subresourceRange.baseMipLevel = 0;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.baseArrayLayer = 0;
        viewInfo.subresourceRange.layerCount = 1;

        if (vkCreateImageView(m_device, &viewInfo, nullptr, &m_swapchainImageViews[i]) != VK_SUCCESS)
            throw std::runtime_error("Failed to create swapchain image views");
    }
}

void GraphicsModule::createRenderPass() {
    VkAttachmentDescription colorAttachment{};
    colorAttachment.format = m_swapchainFormat ;
    colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    //colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    //not clear
    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
    colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    colorAttachment.initialLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;//VK_IMAGE_LAYOUT_UNDEFINED;
    colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentReference colorAttachmentRef{};
    colorAttachmentRef.attachment = 0;
    colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorAttachmentRef;

    VkSubpassDependency dependency{};
    dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
    dependency.dstSubpass = 0;
    dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency.srcAccessMask = 0;
    dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    VkRenderPassCreateInfo renderPassInfo{ VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO };
    renderPassInfo.attachmentCount = 1;
    renderPassInfo.pAttachments = &colorAttachment;
    renderPassInfo.subpassCount = 1;
    renderPassInfo.pSubpasses = &subpass;
    renderPassInfo.dependencyCount = 1;
    renderPassInfo.pDependencies = &dependency;

    if (vkCreateRenderPass(m_device, &renderPassInfo, nullptr, &m_renderPass) != VK_SUCCESS)
        throw std::runtime_error("Failed to create render pass");
}

void GraphicsModule::createFramebuffers() {
    m_framebuffers.resize(m_swapchainImageViews.size());

    for (size_t i = 0; i < m_swapchainImageViews.size(); ++i) {
        VkImageView attachments[] = { m_swapchainImageViews[i] };

        VkFramebufferCreateInfo framebufferInfo{ VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
        framebufferInfo.renderPass = m_renderPass;
        framebufferInfo.attachmentCount = 1;
        framebufferInfo.pAttachments = attachments;
        framebufferInfo.width = m_swapchainExtent.width;
        framebufferInfo.height = m_swapchainExtent.height;
        framebufferInfo.layers = 1;

        if (vkCreateFramebuffer(m_device, &framebufferInfo, nullptr, &m_framebuffers[i]) != VK_SUCCESS)
            throw std::runtime_error("Failed to create framebuffer");
    }
}


void GraphicsModule::createGraphicsPipeline() {
    auto readFile = [](const std::string& path) -> std::vector<char> {
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file.is_open())
            throw std::runtime_error("Failed to open file: " + path);

        size_t size = (size_t)file.tellg();
        if (size == static_cast<size_t>(-1))
            throw std::runtime_error("Failed to get file size: " + path);

        std::vector<char> buffer(size);
        file.seekg(0);
        file.read(buffer.data(), size);
        return buffer;
    };

    std::string shaderPath = SHADER_PATH_GLSL;
    auto vertShaderCode = readFile(shaderPath + "sphere.vert.spv");
    auto fragShaderCode = readFile(shaderPath + "sphere.frag.spv");

    VkShaderModuleCreateInfo createInfo{ VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
    createInfo.codeSize = vertShaderCode.size();
    createInfo.pCode = reinterpret_cast<const uint32_t*>(vertShaderCode.data());
    VkShaderModule vertModule;
    vkCreateShaderModule(m_device, &createInfo, nullptr, &vertModule);

    createInfo.codeSize = fragShaderCode.size();
    createInfo.pCode = reinterpret_cast<const uint32_t*>(fragShaderCode.data());
    VkShaderModule fragModule;
    vkCreateShaderModule(m_device, &createInfo, nullptr, &fragModule);

    VkPipelineShaderStageCreateInfo vertStage{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
    vertStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertStage.module = vertModule;
    vertStage.pName = "main";

    VkPipelineShaderStageCreateInfo fragStage{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
    fragStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragStage.module = fragModule;
    fragStage.pName = "main";

    VkPipelineShaderStageCreateInfo shaderStages[] = { vertStage, fragStage };

    auto binding = GeomCreate::getBindingDescription2();
    auto attributes = GeomCreate::getAttributeDescriptions2();

    VkPipelineVertexInputStateCreateInfo vertexInput{ VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };
    vertexInput.vertexBindingDescriptionCount = 1;
    vertexInput.pVertexBindingDescriptions = &binding;
    vertexInput.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributes.size());
    vertexInput.pVertexAttributeDescriptions = attributes.data();

    VkPipelineInputAssemblyStateCreateInfo inputAssembly{ VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO };
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    inputAssembly.primitiveRestartEnable = VK_FALSE;

    VkViewport viewport{ 0.0f, 0.0f, (float)m_swapchainExtent.width, (float)m_swapchainExtent.height, 0.0f, 1.0f };
    VkRect2D scissor{ {0, 0}, m_swapchainExtent };
    VkPipelineViewportStateCreateInfo viewportState{ VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO };
    viewportState.viewportCount = 1;
    viewportState.pViewports = &viewport;
    viewportState.scissorCount = 1;
    viewportState.pScissors = &scissor;

    VkPipelineRasterizationStateCreateInfo rasterizer{ VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.lineWidth = 1.0f;
    rasterizer.cullMode = VK_CULL_MODE_BACK_BIT;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;

    VkPipelineMultisampleStateCreateInfo multisampling{ VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO };
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineColorBlendAttachmentState colorBlendAttachment{};
    colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                                          VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    colorBlendAttachment.blendEnable = VK_FALSE;

    VkPipelineColorBlendStateCreateInfo colorBlending{ VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
    colorBlending.attachmentCount = 1;
    colorBlending.pAttachments = &colorBlendAttachment;

    VkPushConstantRange pushConstant{};
    pushConstant.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    pushConstant.offset = 0;
    pushConstant.size = sizeof(PushConstants);

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushConstant;

    vkCreatePipelineLayout(m_device, &pipelineLayoutInfo, nullptr, &m_pipelineLayout);

    VkGraphicsPipelineCreateInfo pipelineInfo{ VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
    pipelineInfo.stageCount = 2;
    pipelineInfo.pStages = shaderStages;
    pipelineInfo.pVertexInputState = &vertexInput;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.layout = m_pipelineLayout;
    pipelineInfo.renderPass = m_renderPass;
    pipelineInfo.subpass = 0;

    vkCreateGraphicsPipelines(m_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &m_graphicsPipeline);

    vkDestroyShaderModule(m_device, fragModule, nullptr);
    vkDestroyShaderModule(m_device, vertModule, nullptr);
}




void GraphicsModule::createSyncObjects() {
    m_imageAvailableSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
    m_renderFinishedSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
    m_inFlightFences.resize(MAX_FRAMES_IN_FLIGHT);
    m_commandBuffers.resize(MAX_FRAMES_IN_FLIGHT);

    VkSemaphoreCreateInfo semaphoreInfo{};
    semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    VkCommandBufferAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.commandPool = m_commandPool;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = (uint32_t)m_commandBuffers.size();
    VK_CHECK(vkAllocateCommandBuffers(m_device, &allocInfo, m_commandBuffers.data()), "Failed to allocate command buffers");

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        VK_CHECK(vkCreateSemaphore(m_device, &semaphoreInfo, nullptr, &m_imageAvailableSemaphores[i]), "Sync object creation failed");
        VK_CHECK(vkCreateSemaphore(m_device, &semaphoreInfo, nullptr, &m_renderFinishedSemaphores[i]), "Sync object creation failed");
        VK_CHECK(vkCreateFence(m_device, &fenceInfo, nullptr, &m_inFlightFences[i]), "Sync object creation failed");
    }
}

static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
    VkDebugUtilsMessageTypeFlagsEXT messageType,
    const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
    void* pUserData) {
    std::cerr << "[Vulkan Validation] " << pCallbackData->pMessage << std::endl;
    return VK_FALSE;
}


void GraphicsModule::initRasterRenderers() {
    m_meshRenderer = std::make_unique<StandardMeshRenderer>();
    m_meshRenderer->Initialize(m_device, m_physicalDevice, m_renderPass, m_graphicsQueueFamilyIndex);
}


