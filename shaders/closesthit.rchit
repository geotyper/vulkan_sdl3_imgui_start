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
layout(location = SWS_LOC_SHADOW_RAY) rayPayloadInEXT ShadowPayload   shadow;

hitAttributeEXT vec2 bary;

// scene constants
const vec3 lampCenter = vec3(0.0);
const float lampRadius = 0.4;
const vec3 lampColor  = vec3(1.0, 0.9, 0.7) * 20.0;
const vec3 albedo     = vec3(0.8);

uint pcg(inout uint s) {
    s = s * 747796405u + 2891336453u;
    return s;
}

vec3 sampleUnitSphere(inout uint s) {
    // generate two uniform [0,1) floats
    float u = float(pcg(s) & 0xFFFFu) / 65535.0;  // low 16 bits
    float v = float((pcg(s) >> 16) & 0xFFFFu) / 65535.0;
    // spherical coordinates
    float theta = 2.0 * 3.14159265359 * u;
    float phi   = acos(2.0*v - 1.0);
    float sinp  = sin(phi);
    return vec3(
      sinp * cos(theta),
      sinp * sin(theta),
      cos(phi)
    );
}

void main()
{
    // 1) lamp itself → just emit
    if (gl_InstanceCustomIndexEXT == 0u) {
        prd.color= lampColor;
        return;
    }

    // 2) compute hit position & normal (sphere primitive!)
    vec3 hitPos = gl_WorldRayOriginEXT +
                  gl_WorldRayDirectionEXT * gl_HitTEXT;
    vec3 N      = normalize(hitPos - lampCenter);

    vec3 toCenter = lampCenter - hitPos;
    float  distC  = length(toCenter);
    vec3   L      = toCenter / distC;
    float  tMax   = distC - lampRadius - 0.002;

    shadow.blocked = true;
    traceRayEXT(
      topLevelAS,
      gl_RayFlagsTerminateOnFirstHitEXT|gl_RayFlagsOpaqueEXT,
      0xFF,
      1,1,
      SWS_SHADOW_MISS_SHADERS_IDX,
      hitPos + N*0.001, 0.001,
      L, tMax,
      SWS_LOC_SHADOW_RAY
    );

    float visibility = shadow.blocked ? 0.0 : 1.0;

    // 4) lambert diffuse + ambient
    vec3 Lc     = normalize(lampCenter - hitPos);
    float NdotL = max(dot(N, Lc), 0.0);
    prd.color = albedo * lampColor * NdotL * visibility
              + albedo * 0.1;
}
