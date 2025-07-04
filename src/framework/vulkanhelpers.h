#pragma once
//#define VK_NO_PROTOTYPES
#include <vulkan/vulkan.h>
#include <SDL3/SDL_vulkan.h>
//#include "volk.h"
#include <cassert>
#include <stdexcept> // For std::runtime_error

// Consistent error-checking macro for all files to use
#define VK_CHECK(x, msg)                                            \
do {                                                            \
        VkResult err = (x);                                         \
        if (err != VK_SUCCESS) {                                    \
            throw std::runtime_error(std::string(msg) + " failed: " + std::to_string(err)); \
    }                                                           \
} while (0)


namespace vulkanhelpers {

    struct VulkanContext {
        VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
        VkDevice device = VK_NULL_HANDLE;
        VkCommandPool commandPool = VK_NULL_HANDLE;
        VkQueue transferQueue = VK_NULL_HANDLE;
        VkPhysicalDeviceMemoryProperties physicalDeviceMemoryProperties;
    };

     uint32_t GetMemoryType(const VkMemoryRequirements& memoryRequirements, VkMemoryPropertyFlags memoryProperties, const VkPhysicalDeviceMemoryProperties& deviceMemoryProperties);

    // Теперь она принимает флаги стадий конвейера для точной синхронизации.
    void ImageBarrier(
        VkCommandBuffer         commandBuffer,
        VkImage                 image,
        VkImageLayout           oldLayout,
        VkImageLayout           newLayout,
        const VkImageSubresourceRange& subresourceRange,
        VkPipelineStageFlags    srcStageMask,
        VkPipelineStageFlags    dstStageMask,
        VkAccessFlags           srcAccessMask,
        VkAccessFlags           dstAccessMask
        );


    class Buffer {
    public:
        Buffer();
        ~Buffer() = default;

        VkResult Create(const VulkanContext& ctx, VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags memoryProperties, const void* data = nullptr);
        void Destroy(const VulkanContext& ctx);

        // --- CORRECTED FUNCTION SIGNATURES ---
        void* Map(const VulkanContext& ctx, VkDeviceSize size = UINT64_MAX, VkDeviceSize offset = 0) const;
        void Unmap(const VulkanContext& ctx) const;
        bool UploadData(const VulkanContext& ctx, const void* data, VkDeviceSize size, VkDeviceSize offset = 0) const;

        VkBuffer GetBuffer() const;
        VkDeviceSize GetSize() const;

        // --- DECLARATION FOR THE NEW STATIC HELPER ---
        static Buffer CreateDeviceLocal(const VulkanContext& ctx,VkBufferUsageFlags usage, VkDeviceSize size, const void* data);

    private:
        VkBuffer        mBuffer;
        VkDeviceMemory  mMemory;
        VkDeviceSize    mSize;
    };

    // Other class declarations... (Image, Shader, etc.)
    class Image {
    public:
        Image();
        ~Image() = default;
        VkResult    Create(const VulkanContext& ctx, VkImageType imageType, VkFormat format, VkExtent3D extent, VkImageTiling tiling, VkImageUsageFlags usage, VkMemoryPropertyFlags memoryProperties);
        void        Destroy(const VulkanContext& ctx);
        VkResult    CreateImageView(const VulkanContext& ctx, VkImageViewType viewType, VkFormat format, VkImageSubresourceRange subresourceRange);
        VkResult    CreateSampler(const VulkanContext& ctx, VkFilter magFilter, VkFilter minFilter, VkSamplerMipmapMode mipmapMode, VkSamplerAddressMode addressMode);
        VkFormat    GetFormat() const;
        VkImage     GetImage() const;
        VkImageView GetImageView() const;
        VkSampler   GetSampler() const;
    private:
        VkFormat        mFormat;
        VkImage         mImage;
        VkDeviceMemory  mMemory;
        VkImageView     mImageView;
        VkSampler       mSampler;
    };

    class Shader {
    public:
        Shader();
        ~Shader() = default;
        bool    LoadFromFile(const VulkanContext& ctx, const char* fileName);
        void    Destroy(const VulkanContext& ctx);
        VkPipelineShaderStageCreateInfo GetShaderStage(VkShaderStageFlagBits stage);
    private:
        VkShaderModule  mModule;
    };


    VkDeviceOrHostAddressKHR GetBufferDeviceAddress(const VulkanContext& ctx, const Buffer& buffer);
    VkDeviceOrHostAddressConstKHR GetBufferDeviceAddressConst(const VulkanContext& ctx, const Buffer& buffer);

} // namespace vulkanhelpers

