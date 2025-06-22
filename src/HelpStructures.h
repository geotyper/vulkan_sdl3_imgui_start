#ifndef HELPSTRUCTURES_H
#define HELPSTRUCTURES_H


#include <glm/glm.hpp>

struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    // glm::vec2 uv;
};

struct PushConstants {
    glm::mat4 mvp;
    glm::mat4 model;
};


#endif // HELPSTRUCTURES_H
