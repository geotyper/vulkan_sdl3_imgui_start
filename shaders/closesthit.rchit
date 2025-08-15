#version 460 core
#extension GL_EXT_ray_tracing : enable
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_nonuniform_qualifier : require

#include "../src/shared_with_shaders.h"

layout(location = SWS_LOC_PRIMARY_RAY) rayPayloadInEXT RadiancePayload prd;
layout(location = SWS_LOC2_SHADOW_RAY) rayPayloadEXT ShadowPayload shadow;
uniform accelerationStructureEXT topLevelAS;
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


// A simple pseudo-random number generator.
// We give it a "seed" that should be different for each pixel and frame.
float rand(inout uint seed) {
    seed = seed * 747796405u + 2891336453u;
    uint result = ((seed >> ((seed >> 28u) + 4u)) ^ seed) * 277803737u;
    result = (result >> 22u) ^ result;
    return float(result) / 4294967295.0f;
}

// Generates a random direction vector.
vec3 randomDirection(inout uint seed) {
    float x = rand(seed) * 2.0 - 1.0;
    float y = rand(seed) * 2.0 - 1.0;
    float z = rand(seed) * 2.0 - 1.0;
    return normalize(vec3(x, y, z));
}

// Gets a random point on the surface of a sphere.
vec3 getRandomPointOnSphere(vec3 center, float radius, inout uint seed) {
    return center + randomDirection(seed) * radius;
}


void main() {

    uint packedID = gl_InstanceCustomIndexEXT;

    // 2. Unpack the two values.
    //    - Use a bitwise AND to get the lower 8 bits for the meshId.
    //    - Use a bitwise RIGHT SHIFT to get the upper bits for the unique ID.
    uint meshId           = packedID & 0xFFu;
    uint instanceID = packedID >> 8;
    
    //uint instanceID = gl_InstanceCustomIndexEXT;
    //uint meshId = instanceID;

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
    //uint cubeIndex = uint(abs(objectPos.x * 13.37 + objectPos.y * 7.17 + objectPos.z * 3.14));
    //vec3 baseColor = colorFromInstanceID(cubeIndex);
    
    
    vec3 baseColor = colorFromInstanceID(instanceID);

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
    
     uint uniqueInstanceID = packedID >> 8;
      // --- Main Rendering for Cubelets ---
    baseColor = colorFromInstanceID(uniqueInstanceID);
    vec3 lightPos = vec3(0,0,0);
    
    

    // We hit a cubelet. Do the normal lighting calculation.
    //vec3 lightPos = vec3(0,0,0);
    float lightRadius = 0.025; // The radius of your light sphere

    // --- NEW MONTE CARLO LIGHTING ---

    // 1. Create a unique seed for this pixel and frame
    uint seed = gl_LaunchIDEXT.x * gl_LaunchSizeEXT.y + gl_LaunchIDEXT.y + uint(uniformBuffer.uni.uTime * 1000.0);

    // 2. Pick ONE random point on the light sphere's surface
    vec3 randomPointOnLight = getRandomPointOnSphere(lightPos, lightRadius, seed);

    // 3. Calculate direction and distance to that RANDOM point
    vec3 L = randomPointOnLight - posWorld;
    float distToLight = length(L);
    L = normalize(L);

    // 4. Trace ONE shadow ray to the random point
    shadow.blocked = false; // You need to add this payload to the file
    traceRayEXT(
        topLevelAS,
        gl_RayFlagsOpaqueEXT | gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsSkipClosestHitShaderEXT,
        0xFF,
        SWS_SHADOW_HIT_IDX, 1, SWS_SHADOW_MISS_IDX,
        posWorld + L * 0.001, // Ray origin
        0.001,                // tMin
        L,                    // Ray direction
        distToLight,          // tMax
        SWS_LOC2_SHADOW_RAY
    );

    // 5. Calculate final color
    float diffuseIntensity = 0.0;
    if (!shadow.blocked) {
        diffuseIntensity = max(dot(normalWorld, L), 0.0);
    }

    // You can use either the smooth or cartoon shading here!
    // For smooth soft shadows:
    // prd.color = baseColor * diffuseIntensity;

    // For cartoon soft shadows:
    float brightness = 0.0;
    if(diffuseIntensity > 0.0) { // If it's lit at all
        // Use only two tones: lit or unlit, for a stark look
        brightness = 1.0;
    } else {
        brightness = 0.3; // Shadow tone
    }
    
    float outlineFactor=0.5;
    vec3 celShadedColor = baseColor * brightness;
    // ... (Apply outline logic here as before)
    prd.color = celShadedColor * outlineFactor;

    // --- ИЗМЕНЕНИЕ ЗДЕСЬ ---
    // Исправляем расчёт глубины для тумана
    float t = length(posWorld - gl_WorldRayOriginEXT);
    prd.depth = uint(t); // Убрали множитель * 500.0
}
