#version 460 core
#extension GL_EXT_ray_tracing : enable
#extension GL_GOOGLE_include_directive : require

#include "../src/shared_with_shaders.h"

layout(location = SWS_LOC3_REFLECTION_RAY) rayPayloadInEXT RadiancePayload prd;

void main() {
    prd.color = vec3(0.02); // темный отражающий фон
    prd.blocked = false;
}
