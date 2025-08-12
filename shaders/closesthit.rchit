#version 460 core
#extension GL_EXT_ray_tracing : enable
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_nonuniform_qualifier : require

#include "../src/shared_with_shaders.h"

layout(location = SWS_LOC_PRIMARY_RAY) rayPayloadInEXT RadiancePayload prd;
hitAttributeEXT vec2 attribs;

// --- ИЗМЕНЕНИЕ ЗДЕСЬ ---
// Мы будем получать данные из UBO, а не из отдельных биндингов
layout(set = SWS_SCENE_AS_SET, binding = SWS_UNIFORM_DATA_BINDING) uniform UniformBlock {
    UniformData uni;
} uniformBuffer;

struct Vertex {
    vec4 position;
    vec4 normal;
    vec4 color;
};

// Определение буферов остаётся прежним
layout(set = SWS_SCENE_AS_SET, binding = SWS_VERTICES_BINDING) readonly buffer Vertices {
 Vertex v[];
} vertices[];

layout(set = SWS_SCENE_AS_SET, binding = SWS_INDICES_BINDING) readonly buffer Indices {
    uint i[];
} indices[];

layout(set = SWS_SCENE_AS_SET, binding = SWS_INSTANCE_DATA_BINDING ) readonly buffer InstanceInfo {
    InstanceData ids[];
} instanceInfo[];


// Вспомогательные функции остаются без изменений
uvec3 getTriangleIndices(uint meshId, uint primitiveIndex) {
    uint base = 3 * primitiveIndex;
    return uvec3(
        indices[meshId].i[base + 0],
        indices[meshId].i[base + 1],
        indices[meshId].i[base + 2]
    );
}

vec3 interpolateNormal(uint meshId, uvec3 tri, vec2 baryUV) {
    float u = baryUV.x, v = baryUV.y, w = 1.0 - u - v;
    vec3 n0 = vertices[meshId].v[tri.x].normal.xyz;
    vec3 n1 = vertices[meshId].v[tri.y].normal.xyz;
    vec3 n2 = vertices[meshId].v[tri.z].normal.xyz;
    return normalize(n0 * w + n1 * u + n2 * v);
}

vec3 getHitPosition(uint meshId, uvec3 tri, vec2 baryUV) {
    float u = baryUV.x, v = baryUV.y, w = 1.0 - u - v;
    vec3 p0 = vertices[meshId].v[tri.x].position.xyz;
    vec3 p1 = vertices[meshId].v[tri.y].position.xyz;
    vec3 p2 = vertices[meshId].v[tri.z].position.xyz;
    return p0 * w + p1 * u + p2 * v;
}

const vec3 PALETTE[7] = vec3[7](
    vec3(0.800, 0.900, 1.000), // icy white-blue
    vec3(0.600, 0.800, 1.000), // soft sky blue
    vec3(0.400, 0.700, 1.000), // bright blue
    vec3(0.200, 0.500, 0.900), // medium blue
    vec3(0.100, 0.300, 0.700), // deep cold blue
    vec3(0.000, 0.200, 0.500), // dark navy
    vec3(0.500, 0.900, 1.000)  // electric cyan
);

vec3 colorFromInstanceID(uint instanceID) {
    return PALETTE[instanceID % 7];
}

mat3 rotZ(float a){
    float c = cos(a), s = sin(a);
    return mat3(  c,  s, 0,
                 -s,  c, 0,
                  0,  0, 1);
}


void main() {
    uint instanceID = gl_InstanceCustomIndexEXT;
    uint meshId = instanceID;

    uvec3 tri = getTriangleIndices(meshId, gl_PrimitiveID);
    vec3 normalObj = interpolateNormal(meshId, tri, attribs);
    vec3 posObj = getHitPosition(meshId, tri, attribs);

    vec3 posWorld = (gl_ObjectToWorldEXT * vec4(posObj, 1.0)).xyz;
    mat3 objToWorld = mat3(gl_ObjectToWorldEXT);
    vec3 normalWorld = normalize(transpose(inverse(objToWorld)) * normalObj);
    
    float angle = uniformBuffer.uni.uTime * 0.006;
    mat3 R = rotZ(angle);

    // first go to world, then rotate in world space
    vec3 posWorldNoRot = gl_ObjectToWorldEXT * vec4(posObj, 1.0);
    posWorld      = R * posWorldNoRot;

    // rotate the linear part in world space for normals
    mat3 objToWorld3 = R * mat3(gl_ObjectToWorldEXT);
    normalWorld = normalize(transpose(inverse(objToWorld3)) * normalObj);

    // rotated instance world position (its orbit)
    //vec3 objectPos = R * gl_ObjectToWorldEXT[3];

    // Процедурный цвет остаётся
    vec3 objectPos = gl_ObjectToWorldEXT[3].xyz;
    uint cubeIndex = uint(abs(objectPos.x * 13.37 + objectPos.y * 7.17 + objectPos.z * 3.14));
    vec3 baseColor = colorFromInstanceID(cubeIndex);

    // Если это источник света, делаем его ярким
    if (gl_InstanceCustomIndexEXT == 0) {
        prd.color = uniformBuffer.uni.lightColor * uniformBuffer.uni.lightIntensity;
    } else {
        // --- ИЗМЕНЕНИЕ ЗДЕСЬ ---
        // Используем стабильную позицию света из UBO
        vec3 lightPos = vec3(0,0,0);//uniformBuffer.uni.lightPos;

        // Расчёт освещения теперь корректен
        vec3 L = normalize(lightPos - posWorld);
        float diff = max(dot(normalWorld, L), 0.0);
        prd.color = baseColor * diff;
        
    }

    // --- ИЗМЕНЕНИЕ ЗДЕСЬ ---
    // Исправляем расчёт глубины для тумана
    float t = length(posWorld - gl_WorldRayOriginEXT);
    prd.depth = uint(t); // Убрали множитель * 500.0
}
