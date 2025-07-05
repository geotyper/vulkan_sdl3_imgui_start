#include "ImGuiModule.h"
#include "VulkanCheck.h"
#include "volk.h"
#include <imgui.h>
#include "backends/imgui_impl_sdl3.h"
#include "backends/imgui_impl_vulkan.h"
#include <SDL3/SDL.h>
#include "imgui.h"


#include <stdexcept>


/* ---------------------- ImGui helpers ----------------------- */
void ImGuiModule::createDescriptorPool()
{
    VkDescriptorPoolSize poolSizes[] = {
                                        { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000 },
                                        { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,         1000 },
                                        { VK_DESCRIPTOR_TYPE_SAMPLER,                1000 },
                                        };
    VkDescriptorPoolCreateInfo poolInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
    poolInfo.flags         = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    poolInfo.maxSets       = 1000 * (uint32_t)std::size(poolSizes);
    poolInfo.poolSizeCount = (uint32_t)std::size(poolSizes);
    poolInfo.pPoolSizes    = poolSizes;
    VkResult result = vkCreateDescriptorPool(m_device, &poolInfo, nullptr, &m_descriptorPool);
    if (result != VK_SUCCESS)
        throw std::runtime_error("ImGui descriptor pool failed: " + std::to_string(result));


}

void ImGuiModule::createRenderPass(VkFormat swapchainFormat, VkExtent2D swapchainExtent, const std::vector<VkImageView>& swapchainImageViews)
{
    VkAttachmentDescription color{};
    color.format         = swapchainFormat;
    color.samples        = VK_SAMPLE_COUNT_1_BIT;
    color.loadOp         = VK_ATTACHMENT_LOAD_OP_LOAD;        // сохраняем RT-картинку
    color.storeOp        = VK_ATTACHMENT_STORE_OP_STORE;
    color.initialLayout  = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    color.finalLayout    = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentReference ref{ 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments    = &ref;

    VkRenderPassCreateInfo rp{ VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO };
    rp.attachmentCount = 1;
    rp.pAttachments    = &color;
    rp.subpassCount    = 1;
    rp.pSubpasses      = &subpass;
    VK_CHECK(vkCreateRenderPass(m_device, &rp, nullptr, &m_renderPass),
             "ImGui render pass");

    /* framebuffers */
    m_framebuffers.resize(swapchainImageViews.size());
    for (size_t i = 0; i < swapchainImageViews.size(); ++i)
    {
        VkImageView att[]{ swapchainImageViews[i] };
        VkFramebufferCreateInfo fb{ VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
        fb.renderPass      = m_renderPass;
        fb.attachmentCount = 1;
        fb.pAttachments    = att;
        fb.width  = swapchainExtent.width;
        fb.height = swapchainExtent.height;
        fb.layers = 1;
        VK_CHECK(vkCreateFramebuffer(m_device, &fb, nullptr, &m_framebuffers[i]),
                 "ImGui framebuffer");
    }
}

void ImGuiModule::init(SDL_Window* window,
                       VkInstance instance,
                       VkPhysicalDevice physicalDevice,
                       VkDevice device,
                       VkQueue graphicsQueue,
                       uint32_t queueFamilyIndex,
                       VkFormat swapchainFormat,
                       VkExtent2D swapchainExtent,
                       const std::vector<VkImageView>& swapchainImageViews,
                       VkRenderPass renderPass)
{
    m_window = window;
    m_instance = instance;
    m_physicalDevice = physicalDevice;
    m_device = device;
    m_graphicsQueue = graphicsQueue;
    m_extent = swapchainExtent;
    m_renderPass = renderPass;

    if (!m_device) {
        throw std::runtime_error("ImGuiModule: device is null before creating descriptor pool");
    }

    volkLoadDevice(device);

    createDescriptorPool();
    //createRenderPass(swapchainFormat, swapchainExtent, swapchainImageViews);

    //createFramebuffers(m_extent, swapchainImageViews);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    ImGui_ImplVulkan_LoadFunctions(
        VK_API_VERSION_1_3,
        [](const char* name, void* user_data) -> PFN_vkVoidFunction {
            return vkGetInstanceProcAddr(reinterpret_cast<VkInstance>(user_data), name);
        },
        m_instance);

    ImGui_ImplSDL3_InitForVulkan(m_window);

    ImGui_ImplVulkan_InitInfo info{};
    info.Instance = m_instance;
    info.PhysicalDevice = m_physicalDevice;
    info.Device = m_device;
    info.Queue = m_graphicsQueue;
    info.QueueFamily = queueFamilyIndex;
    info.DescriptorPool = m_descriptorPool;
    info.RenderPass = m_renderPass;
    info.MinImageCount = 2;
    info.ImageCount = static_cast<uint32_t>(swapchainImageViews.size());
    info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
    info.CheckVkResultFn = [](VkResult err) {
        if (err) std::cerr << "[ImGui/Vulkan] Error: " << err << std::endl;
    };

    ImGui_ImplVulkan_Init(&info);
}

void ImGuiModule::createFramebuffers(VkExtent2D extent, const std::vector<VkImageView>& imageViews) {
    m_framebuffers.resize(imageViews.size());
    for (size_t i = 0; i < imageViews.size(); ++i) {
        VkImageView attachments[] = { imageViews[i] };

        VkFramebufferCreateInfo fbInfo{ VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
        fbInfo.renderPass = m_renderPass;
        fbInfo.attachmentCount = 1;
        fbInfo.pAttachments = attachments;
        fbInfo.width = extent.width;
        fbInfo.height = extent.height;
        fbInfo.layers = 1;

        VK_CHECK(vkCreateFramebuffer(m_device, &fbInfo, nullptr, &m_framebuffers[i]),
                 "Failed to create ImGui framebuffer");
    }
}

void ImGuiModule::renderMenu(VkCommandBuffer commandBuffer) {

    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("Solver Menu");
    ImGui::Text("Sphere Options");

    ImGui::End();

    ImGui::Render();
    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer);
}


void ImGuiModule::cleanup() {
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();

    if (descriptorPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(m_device, descriptorPool, nullptr);
        descriptorPool = VK_NULL_HANDLE;
    }
}


//void ImGuiModule::uploadFonts(VkCommandBuffer cmd, VkQueue graphicsQueue)
//{
//    // 1. Создай шрифт
//    ImGui_ImplVulkan_CreateFontsTexture(cmd);
//
//    // 2. Заверши и отправь буфер
//    VkResult err = vkEndCommandBuffer(cmd);
//    if (err != VK_SUCCESS)
//        throw std::runtime_error("Failed to end font command buffer");
//
//    VkSubmitInfo submitInfo{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
//    submitInfo.commandBufferCount = 1;
//    submitInfo.pCommandBuffers = &cmd;
//
//    err = vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
//    if (err != VK_SUCCESS)
//        throw std::runtime_error("Failed to submit font upload");
//
//    vkQueueWaitIdle(graphicsQueue);
//
//    // 3. Удаляем staging объекты, которые создал ImGui внутри
//    ImGui_ImplVulkan_DestroyFontUploadObjects();
//}

void ImGuiModule::uploadFonts(VkCommandBuffer cmd, VkQueue graphicsQueue)
{
    ImGuiIO& io = ImGui::GetIO();
    unsigned char* pixels;
    int width, height;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);

    size_t uploadSize = width * height * 4;

    // Create font image
    VkImageCreateInfo imageInfo{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
    imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
    imageInfo.extent.width = width;
    imageInfo.extent.height = height;
    imageInfo.extent.depth = 1;
    imageInfo.mipLevels = 1;
    imageInfo.arrayLayers = 1;
    imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageInfo.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    VkImage fontImage;
    VkResult err = vkCreateImage(m_device, &imageInfo, nullptr, &fontImage);
    if (err != VK_SUCCESS) throw std::runtime_error("Failed to create font image");

    VkMemoryRequirements memReq;
    vkGetImageMemoryRequirements(m_device, fontImage, &memReq);

    VkMemoryAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
    allocInfo.allocationSize = memReq.size;
    allocInfo.memoryTypeIndex = 0;

    // Find memory type
    VkPhysicalDeviceMemoryProperties memProps;
    vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &memProps);
    for (uint32_t i = 0; i < memProps.memoryTypeCount; i++) {
        if ((memReq.memoryTypeBits & (1 << i)) &&
            (memProps.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)) {
            allocInfo.memoryTypeIndex = i;
            break;
        }
    }

    VkDeviceMemory fontMemory;
    err = vkAllocateMemory(m_device, &allocInfo, nullptr, &fontMemory);
    if (err != VK_SUCCESS) throw std::runtime_error("Failed to allocate font image memory");

    vkBindImageMemory(m_device, fontImage, fontMemory, 0);

    // Create staging buffer
    VkBuffer stagingBuffer;
    VkDeviceMemory stagingMemory;

    VkBufferCreateInfo bufInfo{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
    bufInfo.size = uploadSize;
    bufInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;

    err = vkCreateBuffer(m_device, &bufInfo, nullptr, &stagingBuffer);
    if (err != VK_SUCCESS) throw std::runtime_error("Failed to create staging buffer");

    vkGetBufferMemoryRequirements(m_device, stagingBuffer, &memReq);

    allocInfo.allocationSize = memReq.size;
    for (uint32_t i = 0; i < memProps.memoryTypeCount; i++) {
        if ((memReq.memoryTypeBits & (1 << i)) &&
            (memProps.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) &&
            (memProps.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)) {
            allocInfo.memoryTypeIndex = i;
            break;
        }
    }

    err = vkAllocateMemory(m_device, &allocInfo, nullptr, &stagingMemory);
    if (err != VK_SUCCESS) throw std::runtime_error("Failed to allocate staging buffer memory");

    vkBindBufferMemory(m_device, stagingBuffer, stagingMemory, 0);

    void* mapped;
    vkMapMemory(m_device, stagingMemory, 0, uploadSize, 0, &mapped);
    memcpy(mapped, pixels, uploadSize);
    vkUnmapMemory(m_device, stagingMemory);

    // Record command buffer
    VkCommandBufferBeginInfo beginInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cmd, &beginInfo);

    VkImageMemoryBarrier imgBarrier1{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER };
    imgBarrier1.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imgBarrier1.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    imgBarrier1.srcAccessMask = 0;
    imgBarrier1.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    imgBarrier1.image = fontImage;
    imgBarrier1.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    imgBarrier1.subresourceRange.levelCount = 1;
    imgBarrier1.subresourceRange.layerCount = 1;

    vkCmdPipelineBarrier(cmd,
                         VK_PIPELINE_STAGE_HOST_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         0, 0, nullptr, 0, nullptr, 1, &imgBarrier1);

    VkBufferImageCopy copyRegion{};
    copyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    copyRegion.imageSubresource.layerCount = 1;
    copyRegion.imageExtent = { (uint32_t)width, (uint32_t)height, 1 };

    vkCmdCopyBufferToImage(cmd, stagingBuffer, fontImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copyRegion);

    VkImageMemoryBarrier imgBarrier2 = imgBarrier1;
    imgBarrier2.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    imgBarrier2.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    imgBarrier2.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    imgBarrier2.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

    vkCmdPipelineBarrier(cmd,
                         VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                         0, 0, nullptr, 0, nullptr, 1, &imgBarrier2);

    vkEndCommandBuffer(cmd);

    // Submit and wait
    VkSubmitInfo submitInfo{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &cmd;

    vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
    vkQueueWaitIdle(graphicsQueue);

    // Create image view
    VkImageViewCreateInfo viewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
    viewInfo.image = fontImage;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
    viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    viewInfo.subresourceRange.levelCount = 1;
    viewInfo.subresourceRange.layerCount = 1;

    VkImageView fontImageView;
    err = vkCreateImageView(m_device, &viewInfo, nullptr, &fontImageView);
    if (err != VK_SUCCESS) throw std::runtime_error("Failed to create font image view");

    // Create sampler
    VkSamplerCreateInfo samplerInfo{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
    samplerInfo.magFilter = VK_FILTER_LINEAR;
    samplerInfo.minFilter = VK_FILTER_LINEAR;
    samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    samplerInfo.addressModeU = samplerInfo.addressModeV = samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;

    VkSampler fontSampler;
    vkCreateSampler(m_device, &samplerInfo, nullptr, &fontSampler);

    // Store in ImGui font texture
    io.Fonts->SetTexID(ImGui_ImplVulkan_AddTexture(fontSampler, fontImageView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL));

    // Clean up staging
    vkDestroyBuffer(m_device, stagingBuffer, nullptr);
    vkFreeMemory(m_device, stagingMemory, nullptr);

    // NOTE: you can optionally store and destroy `fontImage`, `fontImageView`, `fontSampler` in your cleanup later
}


