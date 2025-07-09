cmake_minimum_required(VERSION 3.5.0)
project (
    "Vulkan RayTracer"
    VERSION 0.1.0
    DESCRIPTION "Vulkan RayTracing API"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

message("Compiler: " ${CMAKE_CXX_COMPILER_ID})

add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
if (NOT CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    add_compile_options(-Wno-nullability-completeness)
endif()

# === Output paths (optional) ===
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>")

#--------------------------------- Find VulkanSDK ---------------------------------#
find_package(Vulkan REQUIRED)
message("Vulkan_INCLUDE_DIR: " ${Vulkan_INCLUDE_DIR})

set(VULKAN $ENV{VULKAN_SDK}) #set to Vulkan SDK; needed to find SDL2 if installed with Vulkan SDK
message("VULKAN SDK: " ${VULKAN})

include(FetchContent) # enable Fetch Content

#--------------------------------- Fetch SDL3 ---------------------------------#
set(SDL_STATIC ON)
set(SDL_SHARED OFF)
FetchContent_Declare(sdl3
                     GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
                     GIT_TAG main
)
FetchContent_MakeAvailable(sdl3)
include_directories (${sdl3_SOURCE_DIR}/include)

#--------------------------------- Fetch ImGui ---------------------------------#
FetchContent_Declare(imgui
                     GIT_REPOSITORY https://github.com/ocornut/imgui.git
                     GIT_TAG master)

FetchContent_MakeAvailable(Imgui)
# ImGui does not provide CMakeLists.txt
add_library(imgui)
target_include_directories(imgui SYSTEM PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_include_directories(imgui SYSTEM PRIVATE ${Vulkan_INCLUDE_DIRS})
target_compile_definitions(imgui PRIVATE IMGUI_IMPL_VULKAN_NO_PROTOTYPES)
target_link_libraries(imgui SDL3::SDL3)
target_sources(imgui PRIVATE
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp
)
# available now as imgui


##--------------------------------- Fetch vk-bootstrap ---------------------------------#
#FetchContent_Declare(vkbootstrap
#                     GIT_REPOSITORY https://github.com/charles-lunarg/vk-bootstrap.git
#                     GIT_TAG main)
#FetchContent_MakeAvailable(vkbootstrap)
## available now as vk-bootstrap::vk-bootstrap

#--------------------------------- Fetch glm ---------------------------------#

FetchContent_Declare(glm
                     GIT_REPOSITORY https://github.com/g-truc/glm.git
                     GIT_TAG master)
FetchContent_MakeAvailable(glm)
include_directories(${glm_SOURCE_DIR}/src)



#--------------------------------- Main Target ---------------------------------#

set(TARGET DroneVisualizer)
set(HEADERS

    src/VulkanCheck.h
    src/shared_with_shaders.h
    src/framework/camera.h
    src/framework/common.h
    #src/framework/vulkanapp.h
    src/framework/vulkanhelpers.h

    #src/ArcBallCamera.h
    src/HelpStructures.h
    src/MainLoop.h
    src/ImGuiModule.h
    src/GraphicsModule.h
    #src/VulkanHelperMethods.h
    src/GeomCreate.h
    src/RayTracingModule.h
)

set(SRC

    src/framework/camera.cpp
    #src/framework/vulkanapp.cpp
    src/framework/vulkanhelpers.cpp

    #src/ArcBallCamera.cpp
    src/MainLoop.cpp
    src/ImGuiModule.cpp
    src/GraphicsModule.cpp
    #src/VulkanHelperMethods.cpp
    src/GeomCreate.cpp
    src/RayTracingModule.cpp
    src/main.cpp
)

# === Include dirs ===
include_directories(
#    external/imgui
#    external/imgui/backends
     external/stb
     external/tinyobjloader
     external/volk
     external/vulkan/include
#    #src
)

add_subdirectory(external/vulkan)   # Vulkan::Headers
add_subdirectory(external/volk)

# === Executable ===
add_executable(${TARGET} ${HEADERS} ${SRC}
)

target_include_directories(${TARGET} PRIVATE stb)
target_include_directories(${TARGET} PRIVATE external/volk)
target_include_directories(${TARGET} PRIVATE ${Vulkan_INCLUDE_DIR}/vma)
target_include_directories(${TARGET} SYSTEM PRIVATE ${Vulkan_INCLUDE_DIRS})
target_link_libraries(${TARGET} PRIVATE SDL3::SDL3 volk::volk imgui )

## === Shader Path Macro ===
#add_compile_definitions(SHADER_PATH="${CMAKE_CURRENT_BINARY_DIR}/shaders/")
#
## === Gather All GLSL Shaders (Raster and Ray Tracing) ===
#file(GLOB SHADER_SRC "shaders/*.vert" "shaders/*.frag" "shaders/*.rgen" "shaders/*.rchit" "shaders/*.rmiss")
#message("Start to compile shaders")
#foreach(SHADER ${SHADER_SRC})
#    get_filename_component(FILE_NAME ${SHADER} NAME)
#    set(SPIRV "${CMAKE_CURRENT_BINARY_DIR}/shaders/${FILE_NAME}.spv")
#    add_custom_command(
#        OUTPUT ${SPIRV}
#        COMMAND glslangValidator --target-env vulkan1.2 -V ${SHADER} -o ${SPIRV}
#        DEPENDS ${SHADER}
#        COMMENT "Compiling shader: ${FILE_NAME}"
#    )
#    list(APPEND SPIRV_SHADERS ${SPIRV})
#endforeach()
#
#
## === Create a Custom Target to Trigger Compilation ===
#add_custom_target(CompileShaders ALL DEPENDS ${SPIRV_SHADERS})

# ==========================================
#  paths / compile-time defines
# ==========================================
set(SHADER_BIN_DIR "${CMAKE_CURRENT_BINARY_DIR}/shaders")
file(MAKE_DIRECTORY "${SHADER_BIN_DIR}")

target_compile_definitions(DroneVisualizer PRIVATE
    SHADER_PATH_GLSL=\"${SHADER_BIN_DIR}/\"
    SHADER_PATH=\"${SHADER_BIN_DIR}/\"
)

# ==========================================
#  shader list
# ==========================================
file(GLOB SHADER_FILES
     "${CMAKE_SOURCE_DIR}/shaders/*.vert"
     "${CMAKE_SOURCE_DIR}/shaders/*.frag"
     "${CMAKE_SOURCE_DIR}/shaders/*.rgen"
     "${CMAKE_SOURCE_DIR}/shaders/*.rchit"
     "${CMAKE_SOURCE_DIR}/shaders/*.rahit"
     "${CMAKE_SOURCE_DIR}/shaders/*.rmiss")

find_program(GLSLANG_VALIDATOR_EXECUTABLE glslangValidator REQUIRED)

set(SPIRV_SHADERS "")

foreach(SHADER_FILE ${SHADER_FILES})
    get_filename_component(FILE_NAME ${SHADER_FILE} NAME)
    get_filename_component(FILE_EXT  ${SHADER_FILE} LAST_EXT)     # .vert /.rahit …

    # --- карта расширений → параметр -S ---
    string(SUBSTRING ${FILE_EXT} 1 -1 STAGE)   # убираем точку

    # На случай старого glslang поддерживаем только нужные стадии
    if(STAGE MATCHES "vert|frag|comp|geom|tesc|tese|rgen|rchit|rahit|rmiss")
        set(STAGE_ARG "-S" "${STAGE}")
    else()
        message(FATAL_ERROR "Unknown shader stage for file: ${SHADER_FILE}")
    endif()

    set(SPIRV_OUTPUT "${SHADER_BIN_DIR}/${FILE_NAME}.spv")

    add_custom_command(
        OUTPUT  ${SPIRV_OUTPUT}
        COMMAND ${GLSLANG_VALIDATOR_EXECUTABLE}
                --target-env vulkan1.3
                -V ${STAGE_ARG} ${SHADER_FILE} -o ${SPIRV_OUTPUT}
        DEPENDS ${SHADER_FILE}
        COMMENT "Compiling ${FILE_NAME} → ${STAGE}"
        VERBATIM)
    list(APPEND SPIRV_SHADERS ${SPIRV_OUTPUT})
endforeach()

add_custom_target(CompileShaders ALL DEPENDS ${SPIRV_SHADERS})
add_dependencies(DroneVisualizer CompileShaders)

# ==========================================
#  copy SPIR-V next to executable
# ==========================================
add_custom_command(TARGET DroneVisualizer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${SHADER_BIN_DIR}"
            "$<TARGET_FILE_DIR:DroneVisualizer>/shaders"
    COMMENT "Copying SPIR-V shaders to output folder")
