#pragma once
#include <stdexcept>
#include <string>
#define VK_CHECK(x, msg)                                            \
do {                                                            \
        VkResult err = (x);                                         \
        if (err != VK_SUCCESS) {                                    \
            throw std::runtime_error(std::string(msg) + " failed: " + std::to_string(err)); \
    }                                                           \
} while (0)
