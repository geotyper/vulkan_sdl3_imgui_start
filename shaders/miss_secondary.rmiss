#version 460 core
#extension GL_EXT_ray_tracing : enable
#extension GL_GOOGLE_include_directive : require

#include "../src/shared_with_shaders.h"

layout(location = SWS_LOC_PRIMARY_RAY) rayPayloadInEXT RadiancePayload secondaryPrd;

void main() {
    // Просто возвращаем зелёный цвет
    secondaryPrd.color = vec3(0.0, 1.0, 0.0);
}
