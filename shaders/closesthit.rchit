#version 460 core
#extension GL_EXT_ray_tracing : enable
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_nonuniform_qualifier : require

#include "../src/shared_with_shaders.h"

layout(set = SWS_SCENE_AS_SET, binding = SWS_SCENE_AS_BINDING)
  uniform accelerationStructureEXT topLevelAS;

// ... other layout bindings ...
layout(location = SWS_LOC_PRIMARY_RAY) rayPayloadInEXT RadiancePayload prd;
layout(location = SWS_LOC_SHADOW_RAY) rayPayloadInEXT ShadowPayload    shadow;


// ... scene constants ...
const vec3 lampCenter = vec3(0.0);
const float lampRadius = 1.1;
const vec3 lampColor  = vec3(1.0, 0.9, 0.7) * 20.0;
const vec3 albedo     = vec3(0.8);

// ... pcg and sampleUnitSphere functions ...

void main()
{
    // 1) lamp itself → just emit
    if (gl_InstanceCustomIndexEXT == 0u) {
        prd.color = lampColor;
        return;
    }

    // 2) compute hit position & normal
    vec3 hitPos = gl_WorldRayOriginEXT +
                  gl_WorldRayDirectionEXT * gl_HitTEXT;

    // --- FIX ---
    // The normal of a sphere points from its center to the hit position.
    // We must find the center of the specific sphere that was hit in world space.
    
    // Get the transformation matrix for the specific sphere instance that was hit.
        mat4x3 objectToWorld = gl_ObjectToWorldEXT;
    mat4 instanceTransform = mat4(
        vec4(objectToWorld[0], 0.0), // First column
        vec4(objectToWorld[1], 0.0), // Second column
        vec4(objectToWorld[2], 0.0), // Third column
        vec4(0.0, 0.0, 0.0, 1.0)     // Fourth column (identity)
    );

    // The center of our icosphere model is (0,0,0) in its local object space.
    // Transform this local center to find the sphere's actual world-space center.
    vec3 sphereWorldCenter = (instanceTransform * vec4(0.0, 0.0, 0.0, 1.0)).xyz;

    // Now, calculate the correct normal.
    vec3 N = normalize(hitPos - sphereWorldCenter);

    // --- Shadow Ray Logic (This part is now correct) ---
    vec3 toCenter = lampCenter - hitPos;
    float distC   = length(toCenter);
    vec3  L       = toCenter / distC;
    float tMax    = distC - 0.002;

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

    float visibility = shadow.blocked ? 0.0 : 1.0;

    // 4) lambert diffuse + ambient
    N = -N; // <-- Add this line to invert the normal

    // Now, put the original NdotL calculation back
    float NdotL = max(dot(N, L), 0.0);
    prd.color = albedo * lampColor * NdotL * visibility
              + albedo * 0.1;
}
