#version 460 core
#extension GL_EXT_ray_tracing          : enable
#extension GL_GOOGLE_include_directive : require

#include "../src/shared_with_shaders.h"

layout(location = SWS_LOC_PRIMARY_RAY) rayPayloadInEXT RadiancePayload prd;

layout(std140, set = SWS_SCENE_AS_SET, binding = SWS_UNIFORM_DATA_BINDING) uniform UniformData_ { UniformData sceneUB; };

void main() {
    vec3 d = normalize(gl_WorldRayDirectionEXT);
    vec3 skyTop = vec3(0.6, 0.75, 1.0);
    vec3 skyBot = vec3(1.0, 0.9, 0.85);
    float t = clamp(0.5 * (d.y + 1.0), 0.0, 1.0);
    vec3 skyColor = mix(skyBot, skyTop, t);
    
    // Apply intensity (packed in w)
    prd.throughput *= skyColor * sceneUB.lightColor.w;
    
    prd.done = true;
}
