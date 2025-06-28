cmake_minimum_required(VERSION 3.16)
project(DroneVisualizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#set(CMAKE_POSITION_INDEPENDENT_CODE ON)
#add_compile_options(-fPIE)

# === Output paths (optional) ===
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>")

find_package(Vulkan REQUIRED)
find_package(glm REQUIRED)
find_package(SDL3 REQUIRED CONFIG)

# === ImGui sources ===
set(IMGUI_SRC
    external/imgui/imgui.cpp
    external/imgui/imgui_draw.cpp
    external/imgui/imgui_tables.cpp
    external/imgui/imgui_widgets.cpp
    external/imgui/imgui_demo.cpp
    external/imgui/backends/imgui_impl_sdl3.cpp
    external/imgui/backends/imgui_impl_vulkan.cpp
)

# === Your project headers & sources ===
set(HEADERS

    src/framework/camera.h
    src/framework/common.h
    #src/framework/vulkanapp.h
    src/framework/vulkanhelpers.h

    #src/ArcBallCamera.h
    src/HelpStructures.h
    src/MainLoop.h
    #src/ImGuiModule.h
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
    #src/ImGuiModule.cpp
    src/GraphicsModule.cpp
    #src/VulkanHelperMethods.cpp
    src/GeomCreate.cpp
    src/RayTracingModule.cpp
    src/main.cpp
)

# === Include dirs ===
include_directories(
    #external/imgui
    #external/imgui/backends
    external/stb
    external/tinyobjloader
    external/volk
    external/vulkan/include
    #src
)

add_subdirectory(external/vulkan)   # Vulkan::Headers
add_subdirectory(external/volk)

# === Executable ===
add_executable(DroneVisualizer ${HEADERS} ${SRC}
    #${IMGUI_SRC}
    #src/AccelerationStructures.h
    #src/AccelerationStructures.cpp

    )


target_link_options(DroneVisualizer PRIVATE -no-pie)

# === Link SDL3 and Vulkan ===
target_link_libraries(DroneVisualizer
    SDL3::SDL3
    Vulkan::Vulkan
    glm::glm
    volk::volk
)

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

# === Shader Path Definition ===
target_compile_definitions(DroneVisualizer PRIVATE SHADER_PATH="${CMAKE_CURRENT_BINARY_DIR}/shaders/")

# === Shader Compilation Logic ===
file(GLOB SHADER_FILES "shaders/*.rgen" "shaders/*.rchit" "shaders/*.rmiss")
set(SPIRV_SHADERS "")

find_program(GLSLANG_VALIDATOR_EXECUTABLE glslangValidator)
if(NOT GLSLANG_VALIDATOR_EXECUTABLE)
    message(FATAL_ERROR "glslangValidator not found. Please ensure it is in your system PATH.")
endif()

foreach(SHADER_FILE ${SHADER_FILES})
    get_filename_component(FILE_NAME ${SHADER_FILE} NAME)
    set(SPIRV_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/shaders_temp/${FILE_NAME}.spv")

    # --- THIS IS THE CORRECTED SHADER COMPILE COMMAND ---
    # Added --target-env vulkan1.2 to match our application's API version
    add_custom_command(
        OUTPUT ${SPIRV_OUTPUT}
        COMMAND ${GLSLANG_VALIDATOR_EXECUTABLE} --target-env vulkan1.2 -V ${SHADER_FILE} -o ${SPIRV_OUTPUT}
        DEPENDS ${SHADER_FILE}
        COMMENT "Compiling shader: ${FILE_NAME}"
    )
    list(APPEND SPIRV_SHADERS ${SPIRV_OUTPUT})
endforeach()

add_custom_target(CompileShaders ALL DEPENDS ${SPIRV_SHADERS})
add_dependencies(DroneVisualizer CompileShaders)

# === Copy Shaders to Executable Directory ===
add_custom_command(
    TARGET DroneVisualizer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_BINARY_DIR}/shaders_temp"
            "$<TARGET_FILE_DIR:DroneVisualizer>/shaders"
    COMMENT "Copying shaders to executable directory..."
)

