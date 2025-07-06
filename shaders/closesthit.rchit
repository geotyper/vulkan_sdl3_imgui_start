#version 460 core
#extension GL_EXT_ray_tracing : enable
#extension GL_GOOGLE_include_directive : require

#include "../src/shared_with_shaders.h"

layout(set = SWS_SCENE_AS_SET, binding = SWS_SCENE_AS_BINDING)
uniform accelerationStructureEXT topLevelAS;

layout(location = SWS_LOC_PRIMARY_RAY) rayPayloadInEXT RadiancePayload prd;
layout(location = SWS_LOC2_SHADOW_RAY) rayPayloadEXT RadiancePayload reflectionPrd;

hitAttributeEXT vec2 attribs;

struct Vertex {
    vec4 position;
    vec4 normal;
    vec4 color;
};

layout(set = SWS_VERTICES_SET , binding = SWS_VERTICES_BINDING) readonly buffer Vertices {
    Vertex vertices[];
};

layout(set = SWS_INDICES_SET, binding = SWS_INDICES_BINDING) readonly buffer Indices {
    uint i[];
} indices;


// Получение индексов треугольника
uvec3 getTriangleIndices(uint primitiveIndex) {
    uint base = 3 * primitiveIndex;
    return uvec3(indices.i[base + 0], indices.i[base + 1], indices.i[base + 2]);
}

// Интерполяция нормали
vec3 interpolateNormal(uvec3 tri, vec2 baryUV) {
    float u = baryUV.x;
    float v = baryUV.y;
    float w = 1.0 - u - v;

    vec3 n0 = vertices[tri.x].normal.xyz;
    vec3 n1 = vertices[tri.y].normal.xyz;
    vec3 n2 = vertices[tri.z].normal.xyz;

    return normalize(n0 * w + n1 * u + n2 * v);
}

// Цвет по ID экземпляра
vec3 getColorFromInstanceID(uint instanceID) {
    const vec3 colors[7] = vec3[7](
        vec3(1.0, 0.0, 0.0), // Red
        vec3(0.0, 1.0, 0.0), // Green
        vec3(0.0, 0.0, 1.0), // Blue
        vec3(1.0, 1.0, 0.0), // Yellow
        vec3(1.0, 0.0, 1.0), // Magenta
        vec3(0.0, 1.0, 1.0), // Cyan
        vec3(1.0, 0.5, 0.0)  // Orange
    );
    return colors[instanceID % 7];
}

// Трассировка луча к свету
bool traceShadowRay(vec3 origin, vec3 direction) {
    uint flags = gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT;

    ShadowPayload shadow;
    shadow.blocked = false;

    traceRayEXT(
        topLevelAS,
        flags,
        0xFF,
        SWS_SHADOW_MISS_IDX,
        SWS_SHADOW_MISS_IDX,
        SWS_DEFAULT_HIT_IDX,
        origin,
        0.001,
        direction,
        1e20,
        SWS_LOC2_SHADOW_RAY
    );

    return !shadow.blocked;
}


void main() {
    uint primIndex = gl_PrimitiveID;
    uvec3 tri = getTriangleIndices(primIndex);
    vec3 objectNormal = interpolateNormal(tri, attribs);

    // Мировая нормаль
    mat3 objectToWorld = mat3(gl_ObjectToWorldEXT);
    vec3 worldNormal = normalize(transpose(inverse(objectToWorld)) * objectNormal);

    // Позиция попадания в объектном пространстве
    float u = attribs.x;
    float v = attribs.y;
    float w = 1.0 - u - v;

    vec3 p0 = vertices[tri.x].position.xyz;
    vec3 p1 = vertices[tri.y].position.xyz;
    vec3 p2 = vertices[tri.z].position.xyz;
    vec3 hitPosObject = p0 * w + p1 * u + p2 * v;

    // В мировое
    vec3 hitPosWorld = (gl_ObjectToWorldEXT * vec4(hitPosObject, 1.0)).xyz;

    uint instanceID = gl_InstanceCustomIndexEXT;
    vec3 baseColor = getColorFromInstanceID(instanceID);

    // Освещение
    vec3 lightPos = vec3(5.0, 10.0, 5.0);
    vec3 toLight = normalize(lightPos - hitPosWorld);
    bool visible = traceShadowRay(hitPosWorld + worldNormal * 0.01, toLight);

    if (visible) {
        float diffuse = max(dot(worldNormal, toLight), 0.0);
        prd.color = baseColor * diffuse;
    } else {
        prd.color = baseColor * 0.1;
    }

    // === Отражение (bounce) ===
    if (prd.depth < 2) {
        vec3 reflectDir = reflect(gl_WorldRayDirectionEXT, worldNormal);

        reflectionPrd.color = vec3(0.0);
        reflectionPrd.depth = prd.depth + 1;

        traceRayEXT(
            topLevelAS,
            gl_RayFlagsOpaqueEXT,
            0xFF,
            SWS_SECONDARY_MISS_IDX,
            SWS_SECONDARY_MISS_IDX,
            SWS_DEFAULT_HIT_IDX,
            hitPosWorld + reflectDir * 0.01,
            0.001,
            reflectDir,
            1e20,
            SWS_LOC2_SHADOW_RAY
        );

        prd.color += reflectionPrd.color * 0.2; // коэффициент отражения
    }
}

