#include "MainLoop.h"
#include "GraphicsModule.h"
#include "ImGuiModule.h"
#include "GeomCreate.h"

void MainLoop::run() {
    GraphicsModule graphics;
    ImGuiModule ui;

    graphics.init();

    ui.init(
        graphics.getWindow(),
        graphics.getVulkanInstance(),
        graphics.getPhysicalDevice(),
        graphics.getDevice(),
        graphics.getGraphicsQueue(),
        graphics.getGraphicsQueueFamilyIndex(),
        graphics.getRenderPass(),
        static_cast<uint32_t>(graphics.getSwapchainImageViews().size())
        );

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    GeomCreate::createLowPolySphere(vertices, indices);  // Initial default
    GeomCreate::createVertexBuffer(graphics.getDevice(), graphics.getPhysicalDevice(),
                                   vertices, graphics.getVertexBuffer(), graphics.getVertexMemory());
    GeomCreate::createIndexBuffer(graphics.getDevice(), graphics.getPhysicalDevice(),
                                  indices, graphics.getIndexBuffer(), graphics.getIndexMemory());
    graphics.setIndexCount(static_cast<uint32_t>(indices.size()));

    //ui.uploadFonts(graphics.getCommandBuffer(0), graphics.getGraphicsQueue());

    //graphics.rebuildAccelerationStructures(
    //    graphics.getVertexBuffer(),
    //    graphics.getIndexBuffer(),
    //    static_cast<uint32_t>(vertices.size()),
    //    static_cast<uint32_t>(indices.size())
    //    );

    bool useRayTracing = false;

    // === Main loop ===
    while (!graphics.shouldClose()) {
        graphics.pollEvents();
        graphics.handleResizeIfNeeded();
        graphics.beginFrame();

        if (ui.hasGeometryChanged()) {
            vertices.clear();
            indices.clear();
            switch (ui.getCurrentType()) {
            case SphereType::LowPoly:
                GeomCreate::createLowPolySphere(vertices, indices);
                break;
            case SphereType::UVSphere:
                GeomCreate::createUVSphere(ui.getLatDiv(), ui.getLonDiv(), vertices, indices);
                break;
            case SphereType::Icosphere:
                GeomCreate::createIcosphere(ui.getSubdiv(), vertices, indices);
                break;
            }

            graphics.destroySphereBuffers();
            GeomCreate::createVertexBuffer(graphics.getDevice(), graphics.getPhysicalDevice(),
                                           vertices, graphics.getVertexBuffer(), graphics.getVertexMemory());
            GeomCreate::createIndexBuffer(graphics.getDevice(), graphics.getPhysicalDevice(),
                                          indices, graphics.getIndexBuffer(), graphics.getIndexMemory());
            graphics.setIndexCount(static_cast<uint32_t>(indices.size()));

            // *** REFACTORED: Rebuild AS with new geometry data ***
            if (useRayTracing) {
                graphics.rebuildAccelerationStructures(
                    graphics.getVertexBuffer(),
                    graphics.getIndexBuffer(),
                    static_cast<uint32_t>(vertices.size()),
                    static_cast<uint32_t>(indices.size())
                    );
            }

            ui.resetGeometryChanged();
        }

        if (useRayTracing) {
            graphics.drawRayTracedFrame();
        } else {
            graphics.draw([&](VkCommandBuffer cmd) {
                graphics.drawSphere(cmd);
                ui.renderMenu(cmd);
            });
        }
    }
    ui.cleanup();
    graphics.cleanup();
}
