#version 460 core
#extension GL_EXT_ray_tracing          : enable
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_nonuniform_qualifier : require

#include "../src/shared_with_shaders.h"

// ===== Payloads / attrs =====
layout(location = SWS_LOC_PRIMARY_RAY) rayPayloadInEXT RadiancePayload prd;
hitAttributeEXT vec2 attribs;

// ===== UBO =====
layout(set = SWS_SCENE_AS_SET, binding = SWS_UNIFORM_DATA_BINDING)
uniform UniformBlock { UniformData uni; } U;

// ===== Geometry buffers (array-of-SSBOs per mesh) =====
struct Vertex { vec4 position; vec4 normal; vec4 color; };

layout(set = SWS_SCENE_AS_SET, binding = SWS_VERTICES_BINDING) readonly buffer Vertices {
    Vertex v[];
} vertices[];

layout(set = SWS_SCENE_AS_SET, binding = SWS_INDICES_BINDING) readonly buffer Indices {
    uint i[];
} indices[];

// ===== Instance data (для meshId) =====
layout(set = SWS_SCENE_AS_SET, binding = SWS_INSTANCE_DATA_BINDING ) readonly buffer InstanceInfo {
    InstanceData ids[];
} instanceInfo[];

// ===== helpers =====
uvec3 getTriangleIndices(uint meshId, uint primitiveIndex) {
    uint base = 3u * primitiveIndex;
    return uvec3(indices[meshId].i[base + 0u],
                 indices[meshId].i[base + 1u],
                 indices[meshId].i[base + 2u]);
}

vec3 baryLerp3(vec3 a, vec3 b, vec3 c, float w, float u, float v) {
    return a * w + b * u + c * v;
}

// ===== DEBUG: нормали (0=off, 1=interp N, 2=geo N, 3=error heat) =====
#ifndef DEBUG_NORMALS
#define DEBUG_NORMALS 1
#endif

void main()
{
    // распакованный customIndex: низшие 8 бит = meshId, старшие = unique instance id
    uint packedID   = gl_InstanceCustomIndexEXT;
    uint meshId     = packedID & 0xFFu;
    uint uniqueID   = packedID >> 8;

    // три и барицентрики
    uvec3 tri = getTriangleIndices(meshId, gl_PrimitiveID);
    float bu = attribs.x;
    float bv = attribs.y;
    float bw = 1.0 - bu - bv;

    // --- читаем вершины ---
    Vertex v0 = vertices[meshId].v[tri.x];
    Vertex v1 = vertices[meshId].v[tri.y];
    Vertex v2 = vertices[meshId].v[tri.z];

    // --- мировые позиции вершин и точка пересечения ---
    vec3 p0w = gl_ObjectToWorldEXT * vec4(v0.position.xyz, 1.0);
    vec3 p1w = gl_ObjectToWorldEXT * vec4(v1.position.xyz, 1.0);
    vec3 p2w = gl_ObjectToWorldEXT * vec4(v2.position.xyz, 1.0);
    vec3  P  = baryLerp3(p0w, p1w, p2w, bw, bu, bv);

    // --- нормали в МИРЕ: transform(each vertex normal) -> bary -> normalize ---
    mat3 Nw_from_No = transpose(mat3(gl_WorldToObjectEXT)); // inverse-transpose 3x3
    vec3 n0w = normalize(Nw_from_No * v0.normal.xyz);
    vec3 n1w = normalize(Nw_from_No * v1.normal.xyz);
    vec3 n2w = normalize(Nw_from_No * v2.normal.xyz);
    vec3 N   = normalize(n0w * bw + n1w * bu + n2w * bv);

    // fallback геометрическая нормаль (если вершинные нормали нулевые)
    if (length(v0.normal.xyz) + length(v1.normal.xyz) + length(v2.normal.xyz) < 1e-5) {
        N = normalize(cross(p1w - p0w, p2w - p0w));
    }

    // ориентируем к лучу (аналог gl_FrontFacing для RT)
    vec3 I = -gl_WorldRayDirectionEXT;
    N = faceforward(N, I, N);

#if DEBUG_NORMALS == 1
    prd.color = N * 0.5 + 0.5;
    return;
#elif DEBUG_NORMALS == 2
    vec3 Ngeo = normalize(cross(p1w - p0w, p2w - p0w));
    Ngeo = faceforward(Ngeo, I, Ngeo);
    prd.color = Ngeo * 0.5 + 0.5;
    return;
#elif DEBUG_NORMALS == 3
    vec3 Ngeo = normalize(cross(p1w - p0w, p2w - p0w));
    Ngeo = faceforward(Ngeo, I, Ngeo);
    float d = clamp(dot(N, Ngeo), -1.0, 1.0);
    float t = acos(d) / 3.14159265; // [0..1]
    prd.color = mix(vec3(0,1,0), vec3(1,0,0), t);
    return;
#endif

    // --- базовый цвет (возьми из вершин или из палитры по instance id) ---
    vec3 base = baryLerp3(v0.color.rgb, v1.color.rgb, v2.color.rgb, bw, bu, bv);
    // если цвета «шумные», временно можно принудительно:
    // base = vec3(0.8);

    // --- освещение: одна точка + wrap diffuse для мягкого градиента ---
    vec3 Lpos = U.uni.lightPos;
    vec3 Lcol = U.uni.lightColor * U.uni.lightIntensity;

    vec3 Lvec = Lpos - P;
    float dist = max(length(Lvec), 1e-3);
    vec3  L    = Lvec / dist;

    // wrap-диффуз (k в [0..1], больше -> мягче край)
    float k = 0.2;
    float diff = clamp((dot(N, L) + k) / (1.0 + k), 0.0, 1.0);

    // лёгкая дистанционная аттенюация
    float atten = 1.0 / (1.0 + 0.09 * dist + 0.032 * dist * dist);

    // амбиент + диффуз
    vec3 ambient = 0.08 * base;
    vec3 color   = ambient + base * Lcol * (diff * atten);

    // небольшой спекуляр (только при «свете»)
    if (diff > 0.0) {
        vec3 V = normalize(I);
        vec3 H = normalize(L + V);
        float spec = pow(max(dot(N, H), 0.0), 32.0);
        color += 0.05 * spec;
    }

    prd.color = clamp(color, 0.0, 1.0);
}

