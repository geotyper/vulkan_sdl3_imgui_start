#version 460 core
#extension GL_EXT_ray_tracing : enable
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_nonuniform_qualifier : require

#include "../src/shared_with_shaders.h"


layout(set = SWS_SCENE_AS_SET, binding = SWS_SCENE_AS_BINDING)
  uniform accelerationStructureEXT topLevelAS;

// radiance payload (in-only)
layout(location = SWS_LOC_PRIMARY_RAY) rayPayloadInEXT RadiancePayload prd;
// shadow payload (in-out)
layout(location = SWS_LOC_SHADOW_RAY) rayPayloadInEXT ShadowPayload    shadow;

hitAttributeEXT vec2 bary;

// scene constants
const vec3 lampCenter = vec3(0.0);
const float lampRadius = 1.1;
const vec3 lampColor  = vec3(1.0, 0.9, 0.7) * 1.0;
const vec3 albedo     = vec3(0.8);

// ... pcg and sampleUnitSphere functions ...
uint pcg(inout uint s) {
    s = s * 747796405u + 2891336453u;
    return s;
}

vec3 sampleUnitSphere(inout uint s) {
    float u = float(pcg(s) & 0xFFFFu) / 65535.0;
    float v = float((pcg(s) >> 16) & 0xFFFFu) / 65535.0;
    float theta = 2.0 * 3.14159265359 * u;
    float phi   = acos(2.0*v - 1.0);
    float sinp  = sin(phi);
    return vec3(sinp * cos(theta), sinp * sin(theta), cos(phi));
}

void main()
{
    // 1) Handle the lamp itself (emissive) - no changes here
    //if (gl_InstanceCustomIndexEXT == 0u) {
    //    prd.color = lampColor;
    //    return;
    //}

    // 2) Calculate the hit position and the now-correct normal vector
    vec3 hitPos = gl_WorldRayOriginEXT +
                  gl_WorldRayDirectionEXT * gl_HitTEXT;

    mat4x3 objectToWorld = gl_ObjectToWorldEXT;
    mat4 instanceTransform = mat4(
        vec4(objectToWorld[0], 0.0), vec4(objectToWorld[1], 0.0),
        vec4(objectToWorld[2], 0.0), vec4(0.0, 0.0, 0.0, 1.0)
    );
    vec3 sphereWorldCenter = (instanceTransform * vec4(0.0, 0.0, 0.0, 1.0)).xyz;

    // The normal is correct, but we still need to flip it for the icosphere
    vec3 N = normalize(hitPos - sphereWorldCenter);
    N = -N;

    // 3) Soft Shadow and Lighting Loop
    uint seed = (gl_LaunchIDEXT.x * 1973u) ^ gl_PrimitiveID;
    vec3 accumulated_color = vec3(0.0); // Start with black
    const int SAMPLES = 8;

    for (int i = 0; i < SAMPLES; ++i)
    {
        // A) For each sample, pick a new random point on the lamp's surface
        vec3 p = lampCenter + lampRadius * sampleUnitSphere(seed);

        // B) Calculate the light vector and distance FOR THIS SPECIFIC SAMPLE
        vec3 L = normalize(p - hitPos);
        float tMax = length(p - hitPos) - 0.002;

        // C) Trace the shadow ray for this one sample
        shadow.blocked = true;
        traceRayEXT(
            topLevelAS,
            gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT,
            0x02, 1, 1,
            SWS_SHADOW_MISS_SHADERS_IDX,
            hitPos + N * 0.001, 0.001,
            L, tMax,
            SWS_LOC_SHADOW_RAY
        );

        // D) If the path was clear, calculate this sample's lighting and add it
        if (!shadow.blocked) {
            float NdotL = max(dot(N, L), 0.0);
            accumulated_color += albedo * lampColor * NdotL;
        }
    }

    // 4) Final Color: Average the accumulated direct light and add the ambient term
    prd.color = (accumulated_color / SAMPLES) + (albedo * 0.1);
}
