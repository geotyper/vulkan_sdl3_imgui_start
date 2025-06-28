#ifndef HELPSTRUCTURES_H
#define HELPSTRUCTURES_H


#include <glm/glm.hpp>

struct Vertex {
    glm::vec4 position;
    glm::vec4 normal;
    glm::vec4 color;
    glm::vec2 texCoord; // vec2 в конце структуры обычно безопасен,
    glm::vec2 _pad {};

};

struct PushConstants {
    glm::mat4 mvp;
    glm::mat4 model;
};


#endif // HELPSTRUCTURES_H
