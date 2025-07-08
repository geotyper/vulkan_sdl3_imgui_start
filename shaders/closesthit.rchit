#version 460 core
#extension GL_EXT_ray_tracing : enable
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_nonuniform_qualifier : require

#include "../src/shared_with_shaders.h"

layout(location = SWS_LOC_PRIMARY_RAY) rayPayloadInEXT RadiancePayload prd;
hitAttributeEXT vec2 attribs;

struct Vertex {
    vec4 position;
    vec4 normal;
    vec4 color;
};

layout(set = SWS_SCENE_AS_SET, binding = SWS_VERTICES_BINDING) readonly buffer Vertices {
    Vertex v[];
} vertices[];

layout(set = SWS_SCENE_AS_SET, binding = SWS_INDICES_BINDING) readonly buffer Indices {
    uint i[];
} indices[];

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

vec3 getInstanceWorldPosition(uint instanceID) {
    // You can replace this with instanceBuffer.data[instanceID].transform if available
    // For now, assume meshId = instanceID and average 3 vertices
    vec3 sum = vec3(0.0);
    for (int j = 0; j < 3; ++j) {
        sum += vertices[instanceID].v[j].position.xyz;
    }
    return (gl_InstanceID == 0)
        ? (gl_ObjectToWorldEXT * vec4(sum / 3.0, 1.0)).xyz
        : sum / 3.0;
}


vec3 getHitPosition(uint meshId, uvec3 tri, vec2 baryUV) {
    float u = baryUV.x, v = baryUV.y, w = 1.0 - u - v;
    vec3 p0 = vertices[meshId].v[tri.x].position.xyz;
    vec3 p1 = vertices[meshId].v[tri.y].position.xyz;
    vec3 p2 = vertices[meshId].v[tri.z].position.xyz;
    return p0 * w + p1 * u + p2 * v;
}

const vec3 PALETTE[7] = vec3[7](
    vec3(0.945, 0.769, 0.059), // Sunflower (жёлтый)
    vec3(0.203, 0.286, 0.368), // Midnight Blue
    vec3(0.576, 0.439, 0.859), // Wisteria (фиолетовый)
    vec3(0.945, 0.392, 0.392), // Alizarin (красный)
    vec3(0.180, 0.800, 0.443), // Emerald (зелёный)
    vec3(0.203, 0.596, 0.858), // Peter River (голубой)
    vec3(1.000, 0.768, 0.000)  // Bright Orange
);

vec3 colorFromInstanceID(uint instanceID) {
    return PALETTE[instanceID % 7];
}

layout(set = SWS_SCENE_AS_SET, binding = SWS_INSTANCE_DATA_BINDING ) readonly buffer InstanceInfo {
    InstanceData ids[];
} instanceInfo[];

void main() {
    uint instanceID = gl_InstanceCustomIndexEXT;
    uint meshId = instanceID;

    uvec3 tri = getTriangleIndices(meshId, gl_PrimitiveID);
    vec3 normalObj = interpolateNormal(meshId, tri, attribs);
    vec3 posObj = getHitPosition(meshId, tri, attribs);

    vec3 posWorld = (gl_ObjectToWorldEXT * vec4(posObj, 1.0)).xyz;
    mat3 objToWorld = mat3(gl_ObjectToWorldEXT);
    vec3 normalWorld = normalize(transpose(inverse(objToWorld)) * normalObj);

    //vec3 baseColor = vertices[meshId].v[tri.x].color.rgb;
    //vec3 baseColor = colorFromInstanceID(instanceID);
    
    //uint cubeID = instanceInfo[gl_InstanceID].meshId;
    vec3 objectPos = gl_ObjectToWorldEXT[3].xyz;
    uint cubeIndex = uint(abs(objectPos.x * 13.37 + objectPos.y * 7.17 + objectPos.z * 3.14));
    vec3 baseColor = colorFromInstanceID(cubeIndex);

    vec3 lightColor = vec3(1.0, 0.9, 0.7);

    if (gl_InstanceCustomIndexEXT == 0) {
        prd.color = lightColor * 3.0;
    } else {
        vec3 L = normalize(vec3(1.0, 1.5, 0.5)); // свет сверху
        float diff = max(dot(normalWorld, L), 0.0);
        prd.color = baseColor * diff;
    }


    float t = length(posWorld - gl_WorldRayOriginEXT);
    prd.depth = uint(t * 500.0); // для тумана
}

