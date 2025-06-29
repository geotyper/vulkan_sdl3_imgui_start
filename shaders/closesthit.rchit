#version 460 core
#extension GL_EXT_ray_tracing : require

struct RadiancePayload { vec3 color; uint depth; };
struct ShadowPayload   { bool blocked; };

layout(location = 0) rayPayloadEXT RadiancePayload prd;
layout(location = 1) rayPayloadEXT ShadowPayload   shadow;

layout(set = 0, binding = 0) uniform accelerationStructureEXT topLevelAS;

const vec3 LIGHT_EMISSION = vec3(10.0);
const vec3 DIFFUSE_COLOR  = vec3(0.7);

void main()
{
    //------------------------------------------------------------------
    // 1. Позиция пересечения (EXT): gl_HitTEXT
    //------------------------------------------------------------------
    vec3 hitPos = gl_WorldRayOriginEXT +
                  gl_WorldRayDirectionEXT * gl_HitTEXT;

    // Нормаль — быстрая заглушка для сфер, если нет межвершинных нормалей
    vec3 N = normalize(hitPos);

    // 2. Лампочка (instanceCustomIndex == 0)
    if (gl_InstanceCustomIndexEXT == 0) {
        prd.color = LIGHT_EMISSION;
        return;
    }

    // 3. Теневой луч ---------------------------------------------------------
    vec3 lightPos = vec3(0.0);
    vec3 L        = normalize(lightPos - hitPos);
    float distL   = length(lightPos - hitPos);

    shadow.blocked = false;    // payload #1

    traceRayEXT(topLevelAS,
                gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT,
                0xFF,
                /*offset*/0, /*stride*/1, /*miss*/2,
                hitPos + N * 0.001, 0.001,
                L, distL - 0.002,
                /*payloadLocation*/1);   // ← вместо shadow

    float vis = shadow.blocked ? 0.0 : 1.0;
    prd.color = DIFFUSE_COLOR * max(dot(N, L), 0.0) * vis;
}

