#version 460 core
#extension GL_EXT_ray_tracing : enable
#extension GL_GOOGLE_include_directive : require

#include "../src/shared_with_shaders.h"

layout(location = SWS_LOC2_SHADOW_RAY) rayPayloadInEXT RadiancePayload prd;

void main() {
    // В случае, если отражённый луч никуда не попал — возвращаем слабый фон
    prd.color = vec3(0.02); // тёмный "ambient" вместо полного чёрного
}

