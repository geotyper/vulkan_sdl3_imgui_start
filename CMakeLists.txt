cmake_minimum_required(VERSION 3.16)
project(DroneVisualizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
    src/ArcBallCamera.h
    src/HelpStructures.h
    src/MainLoop.h
    src/ImGuiModule.h
    src/GraphicsModule.h
    src/VulkanHelperMethods.h
    src/GeomCreate.h
)

set(SRC

    src/ArcBallCamera.cpp
    src/MainLoop.cpp
    src/ImGuiModule.cpp
    src/GraphicsModule.cpp
    src/VulkanHelperMethods.cpp
    src/GeomCreate.cpp
    src/main.cpp
)

# === Include dirs ===
include_directories(
    external/imgui
    external/imgui/backends
    src
)

# === Executable ===
add_executable(DroneVisualizer ${HEADERS} ${SRC} ${IMGUI_SRC}
    src/AccelerationStructures.h
    src/AccelerationStructures.cpp)

# === Link SDL3 and Vulkan ===
target_link_libraries(DroneVisualizer
    SDL3::SDL3
    Vulkan::Vulkan
    glm::glm
)

# === Shader Path Macro ===
add_compile_definitions(SHADER_PATH="${CMAKE_CURRENT_BINARY_DIR}/shaders/")

# === Gather All GLSL Shaders (Raster and Ray Tracing) ===
file(GLOB SHADER_SRC "shaders/*.vert" "shaders/*.frag" "shaders/*.rgen" "shaders/*.rchit" "shaders/*.rmiss")
message("Start to compile shaders")
foreach(SHADER ${SHADER_SRC})
    get_filename_component(FILE_NAME ${SHADER} NAME)
    set(SPIRV "${CMAKE_CURRENT_BINARY_DIR}/shaders/${FILE_NAME}.spv")
    add_custom_command(
        OUTPUT ${SPIRV}
        COMMAND glslangValidator --target-env vulkan1.2 -V ${SHADER} -o ${SPIRV}
        DEPENDS ${SHADER}
        COMMENT "Compiling shader: ${FILE_NAME}"
    )
    list(APPEND SPIRV_SHADERS ${SPIRV})
endforeach()


# === Create a Custom Target to Trigger Compilation ===
add_custom_target(CompileShaders ALL DEPENDS ${SPIRV_SHADERS})

