#include <vulkan/vulkan.h>

#include <SDL3/SDL.h>
#include "imgui.h"
#include "imgui_impl_sdl3.h"
//#define IMGUI_IMPL_VULKAN_NO_PROTOTYPES
#include "imgui_impl_vulkan.h"

#include "ImGuiModule.h"
#include <stdexcept>


void ImGuiModule::init(SDL_Window* window,
                       VkInstance instance,
                       VkPhysicalDevice inPhysicalDevice,
                       VkDevice inDevice,
                       VkQueue graphicsQueue,
                       uint32_t queueFamilyIndex,
                       VkRenderPass renderPass,
                       uint32_t imageCount)
{
    device = inDevice;
    physicalDevice = inPhysicalDevice;

    // 1. Descriptor pool
    VkDescriptorPoolSize poolSizes[] = {
        { VK_DESCRIPTOR_TYPE_SAMPLER, 1000 },
        { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000 },
        { VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000 },
        { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000 },
        { VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000 },
        { VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000 },
        { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000 },
        { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000 },
        { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000 },
        { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000 },
        { VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000 }
    };

    VkDescriptorPoolCreateInfo poolInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
    poolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    poolInfo.maxSets = 1000 * static_cast<uint32_t>(std::size(poolSizes));
    poolInfo.poolSizeCount = static_cast<uint32_t>(std::size(poolSizes));
    poolInfo.pPoolSizes = poolSizes;

    if (vkCreateDescriptorPool(device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS)
        throw std::runtime_error("Failed to create ImGui descriptor pool");

    // 2. Context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    // 3. SDL3 binding
    ImGui_ImplSDL3_InitForVulkan(window);

    // 4. Vulkan binding
    ImGui_ImplVulkan_InitInfo initInfo{};
    initInfo.Instance = instance;
    initInfo.PhysicalDevice = physicalDevice;
    initInfo.Device = device;
    initInfo.Queue = graphicsQueue;
    initInfo.DescriptorPool = descriptorPool;
    initInfo.MinImageCount = imageCount;
    initInfo.ImageCount = imageCount;
    initInfo.QueueFamily = queueFamilyIndex;
    initInfo.RenderPass = renderPass; // 👈 добавили

    ImGui_ImplVulkan_Init(&initInfo);
}


void ImGuiModule::renderMenu(VkCommandBuffer commandBuffer) {
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("Drone Menu");
    ImGui::Text("Sphere Options");

    const char* types[] = { "LowPoly", "UV Sphere", "Icosphere" };
    int typeIndex = static_cast<int>(currentType);

    if (ImGui::Combo("Sphere Type", &typeIndex, types, IM_ARRAYSIZE(types))) {
        currentType = static_cast<SphereType>(typeIndex);
        geometryChanged = true;
    }

    if (currentType == SphereType::UVSphere) {
        if (ImGui::SliderInt("Lat Div", &latDiv, 3, 64)) geometryChanged = true;
        if (ImGui::SliderInt("Lon Div", &lonDiv, 3, 64)) geometryChanged = true;
    } else if (currentType == SphereType::Icosphere) {
        if (ImGui::SliderInt("Subdiv", &icoSubdiv, 0, 5)) geometryChanged = true;
    }

    ImGui::End();

    ImGui::Render();
    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer);
}


void ImGuiModule::cleanup() {
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();

    if (descriptorPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device, descriptorPool, nullptr);
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
    VkResult err = vkCreateImage(device, &imageInfo, nullptr, &fontImage);
    if (err != VK_SUCCESS) throw std::runtime_error("Failed to create font image");

    VkMemoryRequirements memReq;
    vkGetImageMemoryRequirements(device, fontImage, &memReq);

    VkMemoryAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
    allocInfo.allocationSize = memReq.size;
    allocInfo.memoryTypeIndex = 0;

    // Find memory type
    VkPhysicalDeviceMemoryProperties memProps;
    vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProps);
    for (uint32_t i = 0; i < memProps.memoryTypeCount; i++) {
        if ((memReq.memoryTypeBits & (1 << i)) &&
            (memProps.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)) {
            allocInfo.memoryTypeIndex = i;
            break;
        }
    }

    VkDeviceMemory fontMemory;
    err = vkAllocateMemory(device, &allocInfo, nullptr, &fontMemory);
    if (err != VK_SUCCESS) throw std::runtime_error("Failed to allocate font image memory");

    vkBindImageMemory(device, fontImage, fontMemory, 0);

    // Create staging buffer
    VkBuffer stagingBuffer;
    VkDeviceMemory stagingMemory;

    VkBufferCreateInfo bufInfo{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
    bufInfo.size = uploadSize;
    bufInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;

    err = vkCreateBuffer(device, &bufInfo, nullptr, &stagingBuffer);
    if (err != VK_SUCCESS) throw std::runtime_error("Failed to create staging buffer");

    vkGetBufferMemoryRequirements(device, stagingBuffer, &memReq);

    allocInfo.allocationSize = memReq.size;
    for (uint32_t i = 0; i < memProps.memoryTypeCount; i++) {
        if ((memReq.memoryTypeBits & (1 << i)) &&
            (memProps.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) &&
            (memProps.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)) {
            allocInfo.memoryTypeIndex = i;
            break;
        }
    }

    err = vkAllocateMemory(device, &allocInfo, nullptr, &stagingMemory);
    if (err != VK_SUCCESS) throw std::runtime_error("Failed to allocate staging buffer memory");

    vkBindBufferMemory(device, stagingBuffer, stagingMemory, 0);

    void* mapped;
    vkMapMemory(device, stagingMemory, 0, uploadSize, 0, &mapped);
    memcpy(mapped, pixels, uploadSize);
    vkUnmapMemory(device, stagingMemory);

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
    err = vkCreateImageView(device, &viewInfo, nullptr, &fontImageView);
    if (err != VK_SUCCESS) throw std::runtime_error("Failed to create font image view");

    // Create sampler
    VkSamplerCreateInfo samplerInfo{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
    samplerInfo.magFilter = VK_FILTER_LINEAR;
    samplerInfo.minFilter = VK_FILTER_LINEAR;
    samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    samplerInfo.addressModeU = samplerInfo.addressModeV = samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;

    VkSampler fontSampler;
    vkCreateSampler(device, &samplerInfo, nullptr, &fontSampler);

    // Store in ImGui font texture
    io.Fonts->SetTexID(ImGui_ImplVulkan_AddTexture(fontSampler, fontImageView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL));

    // Clean up staging
    vkDestroyBuffer(device, stagingBuffer, nullptr);
    vkFreeMemory(device, stagingMemory, nullptr);

    // NOTE: you can optionally store and destroy `fontImage`, `fontImageView`, `fontSampler` in your cleanup later
}


