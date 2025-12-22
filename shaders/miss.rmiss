#version 460 core
#extension GL_EXT_ray_tracing          : enable
#extension GL_GOOGLE_include_directive : require

#include "../src/shared_with_shaders.h"

layout(location = SWS_LOC_PRIMARY_RAY) rayPayloadInEXT RadiancePayload prd;

layout(std140, set = SWS_SCENE_AS_SET, binding = SWS_UNIFORM_DATA_BINDING) uniform UniformData_ { UniformData sceneUB; };
layout(set = SWS_SCENE_AS_SET, binding = SWS_CAMERA_BINDING) uniform CameraUBO { mat4 invView; mat4 invProj; } cam;

void main() {
    vec3 d = normalize(gl_WorldRayDirectionEXT);
    vec3 skyTop = pow(sceneUB.backColorTop.rgb, vec3(2.2));
    vec3 skyBot = pow(sceneUB.backColorBot.rgb, vec3(2.2));
    
    // Calculate view-aligned gradient for a "backdrop" feel
    // Extract camera's Up vector from the inverse view matrix (cam.invView)
    vec3 camUp = normalize(vec3(cam.invView[0][1], cam.invView[1][1], cam.invView[2][1]));
    
    // Projection of the ray direction onto the camera's local Y axis
    float viewT = dot(d, camUp);
    
    // Standard world-Y gradient as well
    float worldT = d.y;
    
    // Mix them to get a stable but view-responsive background
    float t = clamp(0.5 * (mix(worldT, viewT, 0.8) + 1.0), 0.0, 1.0);
    
    vec3 skyColor = mix(skyBot, skyTop, t);
    
    // Apply intensity (packed in w)
    prd.throughput *= skyColor * sceneUB.lightColor.w;
    
    prd.done = true;
}
