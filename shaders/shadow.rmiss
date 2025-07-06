#version 460 core
#extension GL_EXT_ray_tracing : enable
#extension GL_GOOGLE_include_directive : require

#include "../src/shared_with_shaders.h"

layout(location = SWS_LOC_SHADOW_RAY) rayPayloadInEXT ShadowPayload shadow;

void main() {
    shadow.blocked = false;
}
