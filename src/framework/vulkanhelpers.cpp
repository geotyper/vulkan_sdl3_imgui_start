#include "vulkanhelpers.h"
#include <string>
#include <vector>
#include <fstream>
#include <cstring> // for memcpy
#include <string> // Required for std::to_string
#include <sstream> // Required for std::stringstream

#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_PSD
#define STBI_NO_GIF
#define STBI_NO_PIC
#define STBI_NO_PNM
#include "stb_image.h"

namespace vulkanhelpers {


uint32_t GetMemoryType(const VkMemoryRequirements& memoryRequirements, VkMemoryPropertyFlags requiredProperties, const VkPhysicalDeviceMemoryProperties& deviceMemoryProperties) {
    for (uint32_t i = 0; i < deviceMemoryProperties.memoryTypeCount; i++) {
        // Check if the i-th memory type is supported for this resource
        if (memoryRequirements.memoryTypeBits & (1 << i)) {
            // Check if this memory type has all the properties we need
            if ((deviceMemoryProperties.memoryTypes[i].propertyFlags & requiredProperties) == requiredProperties) {
                return i; // Found a suitable type
            }
        }
    }

    // If we get here, no suitable type was found.
    // Let's create a detailed error message to help debug.
    std::stringstream errorMsg;
    errorMsg << "Failed to find suitable memory type!\n";
    errorMsg << "  > Required properties: " << requiredProperties << "\n";
    errorMsg << "  > Allowed by resource (memoryTypeBits): " << memoryRequirements.memoryTypeBits << "\n\n";
    errorMsg << "  > Available GPU memory types (" << deviceMemoryProperties.memoryTypeCount << "):\n";
    for (uint32_t i = 0; i < deviceMemoryProperties.memoryTypeCount; i++) {
        errorMsg << "    - Type " << i << ": flags = " << deviceMemoryProperties.memoryTypes[i].propertyFlags;
        if (memoryRequirements.memoryTypeBits & (1 << i)) {
            errorMsg << " (Compatible with resource)\n";
        } else {
            errorMsg << " (NOT compatible with resource)\n";
        }
    }

    throw std::runtime_error(errorMsg.str());
}

void ImageBarrier(VkCommandBuffer commandBuffer, VkImage image, const VkImageSubresourceRange& subresourceRange, VkAccessFlags srcAccessMask, VkAccessFlags dstAccessMask, VkImageLayout oldLayout, VkImageLayout newLayout) {
    VkImageMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    barrier.oldLayout = oldLayout;
    barrier.newLayout = newLayout;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = image;
    barrier.subresourceRange = subresourceRange;
    barrier.srcAccessMask = srcAccessMask;
    barrier.dstAccessMask = dstAccessMask;

    vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);
}

// --- Buffer Implementation ---
Buffer::Buffer() : mBuffer(VK_NULL_HANDLE), mMemory(VK_NULL_HANDLE), mSize(0) {}

VkResult Buffer::Create(const VulkanContext& ctx, VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags memoryProperties, const void* data) {
    mSize = size;

    bool isDeviceLocal = (memoryProperties & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) != 0;

    // --- Staging Buffer Logic ---
    if (data != nullptr && isDeviceLocal) {
        // Create a temporary, CPU-visible staging buffer
        vulkanhelpers::Buffer stagingBuffer;
        VK_CHECK(stagingBuffer.Create(ctx, size,
                                      VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                                      data),
                 "Staging buffer creation failed");

        // Create the final, GPU-only device buffer
        VK_CHECK(this->Create(ctx, size,
                              VK_BUFFER_USAGE_TRANSFER_DST_BIT | usage,
                              VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                              nullptr),
                 "Final device-local buffer creation failed");

        // Copy from staging to final buffer
        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandPool = ctx.commandPool;
        allocInfo.commandBufferCount = 1;

        VkCommandBuffer commandBuffer;
        vkAllocateCommandBuffers(ctx.device, &allocInfo, &commandBuffer);

        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        vkBeginCommandBuffer(commandBuffer, &beginInfo);

        VkBufferCopy copyRegion{};
        copyRegion.size = size;
        vkCmdCopyBuffer(commandBuffer, stagingBuffer.GetBuffer(), mBuffer, 1, &copyRegion);

        vkEndCommandBuffer(commandBuffer);

        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffer;

        vkQueueSubmit(ctx.transferQueue, 1, &submitInfo, VK_NULL_HANDLE);
        vkQueueWaitIdle(ctx.transferQueue);
        vkFreeCommandBuffers(ctx.device, ctx.commandPool, 1, &commandBuffer);

        stagingBuffer.Destroy(ctx);
        return VK_SUCCESS;
    }

    // --- Direct Creation Logic ---
    VkBufferCreateInfo bufferInfo{};
    bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufferInfo.size = size;
    bufferInfo.usage = usage;
    bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VK_CHECK(vkCreateBuffer(ctx.device, &bufferInfo, nullptr, &mBuffer), "Buffer creation failed");

    VkMemoryRequirements memRequirements;
    vkGetBufferMemoryRequirements(ctx.device, mBuffer, &memRequirements);

    // Determine if we need device address bit
    bool needsDeviceAddress = (usage & VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT) != 0;

    // Memory allocation
    VkMemoryAllocateFlagsInfo allocFlagsInfo{};
    allocFlagsInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO;
    allocFlagsInfo.flags = needsDeviceAddress ? VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT : 0;
    allocFlagsInfo.pNext = nullptr;

    VkMemoryAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocInfo.allocationSize = memRequirements.size;
    allocInfo.memoryTypeIndex = GetMemoryType(memRequirements, memoryProperties, ctx.physicalDeviceMemoryProperties);
    allocInfo.pNext = (allocFlagsInfo.flags != 0) ? &allocFlagsInfo : nullptr;

    VK_CHECK(vkAllocateMemory(ctx.device, &allocInfo, nullptr, &mMemory), "Buffer memory allocation failed");
    VK_CHECK(vkBindBufferMemory(ctx.device, mBuffer, mMemory, 0), "Buffer memory binding failed");

    // If data was provided for a host-visible buffer, map and copy it now
    if (data != nullptr && !isDeviceLocal) {
        void* mapped = Map(ctx, size);
        if (mapped) {
            memcpy(mapped, data, size);
            Unmap(ctx);
        }
    }

    return VK_SUCCESS;
}


void Buffer::Destroy(const VulkanContext& ctx) {
    if (mBuffer) vkDestroyBuffer(ctx.device, mBuffer, nullptr);
    if (mMemory) vkFreeMemory(ctx.device, mMemory, nullptr);
    mBuffer = VK_NULL_HANDLE;
    mMemory = VK_NULL_HANDLE;
}

void* Buffer::Map(const VulkanContext& ctx, VkDeviceSize size, VkDeviceSize offset) const {
    void* mapped = nullptr;
    if (size > mSize) { size = mSize; }
    if (vkMapMemory(ctx.device, mMemory, offset, size, 0, &mapped) != VK_SUCCESS) {
        return nullptr;
    }
    return mapped;
}

void Buffer::Unmap(const VulkanContext& ctx) const {
    vkUnmapMemory(ctx.device, mMemory);
}

bool Buffer::UploadData(const VulkanContext& ctx, const void* data, VkDeviceSize size, VkDeviceSize offset) const {
    void* mapped = Map(ctx,size, offset);
    if (!mapped) return false;
    memcpy(mapped, data, size);
    Unmap(ctx);
    return true;
}

VkBuffer Buffer::GetBuffer() const { return mBuffer; }
VkDeviceSize Buffer::GetSize() const { return mSize; }


Buffer Buffer::CreateDeviceLocal(const VulkanContext& ctx, VkBufferUsageFlags usage, VkDeviceSize size, const void* data) {
    Buffer stagingBuffer;
    VK_CHECK(stagingBuffer.Create(
                 ctx,
                 size,
                 VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
                 ), "Failed to create staging buffer");

    stagingBuffer.UploadData(ctx, data, size);

    Buffer deviceBuffer;
    VK_CHECK(deviceBuffer.Create(
                 ctx,
                 size,
                 VK_BUFFER_USAGE_TRANSFER_DST_BIT | usage,
                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
                 ), "Failed to create device-local buffer");

    VkCommandBufferAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandPool = ctx.commandPool;
    allocInfo.commandBufferCount = 1;

    VkCommandBuffer commandBuffer;
    vkAllocateCommandBuffers(ctx.device, &allocInfo, &commandBuffer);

    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginInfo);

    VkBufferCopy copyRegion{};
    copyRegion.size = size;
    vkCmdCopyBuffer(commandBuffer, stagingBuffer.GetBuffer(), deviceBuffer.GetBuffer(), 1, &copyRegion);

    vkEndCommandBuffer(commandBuffer);

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;

    vkQueueSubmit(ctx.transferQueue, 1, &submitInfo, VK_NULL_HANDLE);
    vkQueueWaitIdle(ctx.transferQueue);

    vkFreeCommandBuffers(ctx.device, ctx.commandPool, 1, &commandBuffer);

    stagingBuffer.Destroy(ctx);
    return deviceBuffer;
}

// --- Image Implementation ---
Image::Image() : mFormat(VK_FORMAT_UNDEFINED), mImage(VK_NULL_HANDLE), mMemory(VK_NULL_HANDLE), mImageView(VK_NULL_HANDLE), mSampler(VK_NULL_HANDLE) {}

void Image::Destroy(const VulkanContext& ctx) {
    if (mSampler) vkDestroySampler(ctx.device, mSampler, nullptr);
    if (mImageView) vkDestroyImageView(ctx.device, mImageView, nullptr);
    if (mImage) vkDestroyImage(ctx.device, mImage, nullptr);
    if (mMemory) vkFreeMemory(ctx.device, mMemory, nullptr);
    mSampler = VK_NULL_HANDLE;
    mImageView = VK_NULL_HANDLE;
    mImage = VK_NULL_HANDLE;
    mMemory = VK_NULL_HANDLE;
}

VkResult Image::Create(const VulkanContext& ctx, VkImageType imageType, VkFormat format, VkExtent3D extent, VkImageTiling tiling, VkImageUsageFlags usage, VkMemoryPropertyFlags memoryProperties) {
    mFormat = format;
    VkImageCreateInfo imageInfo{};
    imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imageInfo.imageType = imageType;
    imageInfo.extent = extent;
    imageInfo.mipLevels = 1;
    imageInfo.arrayLayers = 1;
    imageInfo.format = format;
    imageInfo.tiling = tiling;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imageInfo.usage = usage;
    imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VK_CHECK(vkCreateImage(ctx.device, &imageInfo, nullptr, &mImage), "Failed to create image");

    VkMemoryRequirements memRequirements;
    vkGetImageMemoryRequirements(ctx.device, mImage, &memRequirements);

    VkMemoryAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocInfo.allocationSize = memRequirements.size;
    allocInfo.memoryTypeIndex = GetMemoryType(memRequirements, memoryProperties, ctx.physicalDeviceMemoryProperties);

    VK_CHECK(vkAllocateMemory(ctx.device, &allocInfo, nullptr, &mMemory), "Failed to allocate image memory");
    VK_CHECK(vkBindImageMemory(ctx.device, mImage, mMemory, 0), "Failed to bind image memory");

    return VK_SUCCESS;
}

VkResult Image::CreateImageView(const VulkanContext& ctx, VkImageViewType viewType, VkFormat format, VkImageSubresourceRange subresourceRange) {
    VkImageViewCreateInfo viewInfo{};
    viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    viewInfo.image = mImage;
    viewInfo.viewType = viewType;
    viewInfo.format = format;
    viewInfo.subresourceRange = subresourceRange;
    return vkCreateImageView(ctx.device, &viewInfo, nullptr, &mImageView);
}

VkResult Image::CreateSampler(const VulkanContext& ctx, VkFilter magFilter, VkFilter minFilter, VkSamplerMipmapMode mipmapMode, VkSamplerAddressMode addressMode) {
    VkSamplerCreateInfo samplerInfo{};
    samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    samplerInfo.magFilter = magFilter;
    samplerInfo.minFilter = minFilter;
    samplerInfo.addressModeU = addressMode;
    samplerInfo.addressModeV = addressMode;
    samplerInfo.addressModeW = addressMode;
    samplerInfo.anisotropyEnable = VK_FALSE;
    samplerInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    samplerInfo.unnormalizedCoordinates = VK_FALSE;
    samplerInfo.compareEnable = VK_FALSE;
    samplerInfo.mipmapMode = mipmapMode;
    return vkCreateSampler(ctx.device, &samplerInfo, nullptr, &mSampler);
}

VkFormat Image::GetFormat() const { return mFormat; }
VkImage Image::GetImage() const { return mImage; }
VkImageView Image::GetImageView() const { return mImageView; }
VkSampler Image::GetSampler() const { return mSampler; }

// --- Shader Implementation ---
Shader::Shader() : mModule(VK_NULL_HANDLE) {}

bool Shader::LoadFromFile(const VulkanContext& ctx, const char* fileName) {
    std::ifstream file(fileName, std::ios::ate | std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    size_t fileSize = (size_t) file.tellg();
    std::vector<char> buffer(fileSize);
    file.seekg(0);
    file.read(buffer.data(), fileSize);
    file.close();

    VkShaderModuleCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    createInfo.codeSize = buffer.size();
    createInfo.pCode = reinterpret_cast<const uint32_t*>(buffer.data());

    return (vkCreateShaderModule(ctx.device, &createInfo, nullptr, &mModule) == VK_SUCCESS);
}

void Shader::Destroy(const VulkanContext& ctx) {
    if (mModule) {
        vkDestroyShaderModule(ctx.device, mModule, nullptr);
        mModule = VK_NULL_HANDLE;
    }
}

VkPipelineShaderStageCreateInfo Shader::GetShaderStage(VkShaderStageFlagBits stage) {
    VkPipelineShaderStageCreateInfo shaderStageInfo{};
    shaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStageInfo.stage = stage;
    shaderStageInfo.module = mModule;
    shaderStageInfo.pName = "main";
    return shaderStageInfo;
}

// --- Address Getters ---
VkDeviceOrHostAddressKHR GetBufferDeviceAddress(const VulkanContext& ctx, const Buffer& buffer) {
    VkBufferDeviceAddressInfo info{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO, nullptr, buffer.GetBuffer() };
    VkDeviceOrHostAddressKHR result{};

    assert(vkGetBufferDeviceAddressKHR && "vkGetBufferDeviceAddressKHR is NULL!");


    result.deviceAddress = vkGetBufferDeviceAddressKHR(ctx.device, &info);
    return result;
}

VkDeviceOrHostAddressConstKHR GetBufferDeviceAddressConst(const VulkanContext& ctx, const Buffer& buffer) {
    VkDeviceOrHostAddressKHR address = GetBufferDeviceAddress(ctx, buffer);
    VkDeviceOrHostAddressConstKHR result{};
    result.deviceAddress = address.deviceAddress;
    return result;
}

} // namespace vulkanhelpers
