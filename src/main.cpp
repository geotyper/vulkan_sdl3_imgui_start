#include "MainLoop.h"
#include <stdexcept>
#include <iostream>

int main(int argc, char* argv[]) {
    MainLoop app;

    try {
        app.Initialize("Vulkan Ray Tracer");
        app.Run();
    } catch (const std::exception& e) {
        std::cerr << "An error occurred: " << e.what() << std::endl;
        // Ensure shutdown is called even if initialization fails partway
        app.Shutdown();
        return EXIT_FAILURE;
    }

    app.Shutdown();
    return EXIT_SUCCESS;
}
