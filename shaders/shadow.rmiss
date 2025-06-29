#version 460 core
#extension GL_EXT_ray_tracing : enable
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_nonuniform_qualifier : require

#include "../src/shared_with_shaders.h"

// Shadow payload (in-out)
layout(location = SWS_LOC_SHADOW_RAY) rayPayloadInEXT ShadowPayload   shadow;

void main() {
    // Ничего не препятствует
    shadow.blocked = false;
}

