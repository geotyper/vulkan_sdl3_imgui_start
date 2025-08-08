#version 460 core
#extension GL_EXT_ray_tracing : enable
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_nonuniform_qualifier : require

#include "../src/shared_with_shaders.h"

layout(set = SWS_SCENE_AS_SET, binding = SWS_SCENE_AS_BINDING) uniform accelerationStructureEXT topLevelAS;
layout(set = SWS_SCENE_AS_SET, binding = SWS_RESULT_IMAGE_BINDING, rgba8) uniform image2D resultImage;

layout(set = SWS_SCENE_AS_SET, binding = SWS_UNIFORM_DATA_BINDING) uniform UniformBlock {
    UniformData uni;
} uniformBuffer;

layout(location = SWS_LOC_PRIMARY_RAY) rayPayloadInEXT RadiancePayload prd;
layout(location = SWS_LOC2_SHADOW_RAY) rayPayloadEXT ShadowPayload shadow;
layout(location = SWS_LOC3_REFLECTION_RAY) rayPayloadEXT RadiancePayload reflectionPayload;

hitAttributeEXT vec2 attribs;

struct Vertex {
    vec3 position;
    vec4 normal;
    vec4 color;
};

layout(set = SWS_SCENE_AS_SET, binding = SWS_VERTICES_BINDING) readonly buffer Vertices {
    Vertex v[];
} vertices[];

layout(set = SWS_SCENE_AS_SET, binding = SWS_INDICES_BINDING) readonly buffer Indices {
    uint i[];
} indices[];

// Helpers
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

bool traceShadowRay(vec3 origin, vec3 dir) {
    shadow.blocked = false;
    traceRayEXT(topLevelAS,
        gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT,
        0xFF,
        SWS_SHADOW_MISS_IDX,
        SWS_SHADOW_MISS_IDX,
        SWS_DEFAULT_HIT_IDX,
        origin, 0.001, dir, 1e20,
        SWS_LOC2_SHADOW_RAY);
    return !shadow.blocked;
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

// Простая хеш-функция для получения псевдо-случайного числа от 0 до 1
float rand(inout uint seed) {
    seed = seed * 747796405u + 2891336453u;
    uint result = ((seed >> ((seed >> 28u) + 4u)) ^ seed) * 277803737u;
    result = (result >> 22u) ^ result;
    return float(result) / 4294967295.0f;
}

// Создает локальную систему координат вокруг нормали
void createCoordinateSystem(in vec3 N, out vec3 Nt, out vec3 Nb) {
    if (abs(N.x) > abs(N.y))
        Nt = vec3(N.z, 0, -N.x) / sqrt(N.x * N.x + N.z * N.z);
    else
        Nt = vec3(0, -N.z, N.y) / sqrt(N.y * N.y + N.z * N.z);
    Nb = cross(N, Nt);
}

// Генерирует случайный вектор в полусфере с косинус-взвешиванием
vec3 sampleCosineWeightedHemisphere(inout uint seed, vec3 normal) {
    float r1 = rand(seed);
    float r2 = rand(seed);

    float sinTheta = sqrt(1.0 - r1 * r1);
    float phi = 2.0 * 3.1415926535 * r2;
    float x = sinTheta * cos(phi);
    float z = sinTheta * sin(phi);
    vec3 sample_dir = vec3(x, r1, z); // y - up

    vec3 Nt, Nb;
    createCoordinateSystem(normal, Nt, Nb);
    return x * Nt + r1 * normal + z * Nb;
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

    //vec3 baseColor = vertices[meshId].v[tri.x].color.rgb;
    //vec3 baseColor = colorFromInstanceID(instanceID);
    
    //uint cubeID = instanceInfo[gl_InstanceID].meshId;
    vec3 objectPos = gl_ObjectToWorldEXT[3].xyz;
    uint cubeIndex = uint(abs(objectPos.x * 13.37 + objectPos.y * 7.17 + objectPos.z * 3.14));
    vec3 baseColor = colorFromInstanceID(cubeIndex);



    // === Light source from instance 0 ===
    vec3 lightPos = getInstanceWorldPosition(0);
    vec3 lightColor = uniformBuffer.uni.lightColor;
    float lightIntensity = uniformBuffer.uni.lightIntensity;

    vec3 toLight = lightPos - posWorld;
    float dist = length(toLight);
    vec3 lightDir = normalize(toLight);
    float NdotL = max(dot(normalWorld, lightDir), 0.0);
    float attenuation = 1.0 / (dist * dist + 1.0);

    bool isSelfLit = (instanceID == 0);
    bool lit = isSelfLit || traceShadowRay(posWorld + normalWorld * 0.01, lightDir);

    vec3 diffuse = vec3(0.0);
    if (lit) {
        diffuse = baseColor * lightColor * lightIntensity * NdotL * attenuation;
    }

    vec3 resultColor = diffuse;
    
    //if (instanceID == 0 && prd.depth > 1) {
    //    prd.color = baseColor * 0.1; // Возвращаем очень тусклый цвет, чтобы не создавать артефактов
    //    return;
   // }
    
    // В функции main() после основного diffuse-освещения:
     if (prd.depth < 3) {
        
        // 2. ИСПРАВЛЕНИЕ: Правильное сэмплирование полусферы
        // Создаем "зерно" для случайных чисел. Оно должно быть уникальным для пикселя и кадра.
        uint seed = gl_LaunchIDEXT.x * 1973 + gl_LaunchIDEXT.y * 9277 + uint(uniformBuffer.uni.uTime * 1000.0) + prd.depth * 5471;
        
        vec3 randomDir = sampleCosineWeightedHemisphere(seed, normalWorld);

        vec3 bounceOrigin = posWorld + normalWorld * 0.001;
        
        reflectionPayload.color = vec3(0);
        reflectionPayload.depth = prd.depth + 1;
        traceRayEXT(
            topLevelAS,
            gl_RayFlagsOpaqueEXT, 0xFF,
            SWS_SECONDARY_MISS_IDX, 0, SWS_DEFAULT_HIT_IDX,
            bounceOrigin, 0.001, randomDir, 1e20,
            SWS_LOC3_REFLECTION_RAY
        );
        
        //resultColor += reflectionPayload.color * baseColor; // Убрал bounceStrength для простоты
        vec3 bounced = reflectionPayload.color * baseColor;

        // === Clamping bounced light if too bright ===
        float maxBrightness = 2.0; // регулируй по вкусу
        float brightness = max(bounced.r, max(bounced.g, bounced.b));
        if (brightness > maxBrightness) {
            bounced *= maxBrightness / brightness;
        }

        resultColor += bounced;
    }


    // === Reflection ===
    if (prd.depth < 3) {
        vec3 reflectDir = reflect(gl_WorldRayDirectionEXT, normalWorld);
        reflectDir = normalize(reflectDir);
        vec3 reflectOrigin = posWorld + reflectDir * 0.01;

        reflectionPayload.color = vec3(0);
        reflectionPayload.depth = prd.depth + 1;

        traceRayEXT(
            topLevelAS,
            gl_RayFlagsOpaqueEXT,
            0xFF,
            SWS_SECONDARY_MISS_IDX,
            SWS_SECONDARY_MISS_IDX,
            SWS_DEFAULT_HIT_IDX,
            reflectOrigin, 0.001, reflectDir, 1e20,
            SWS_LOC3_REFLECTION_RAY
        );

        float fresnel = 0.2;
        resultColor += reflectionPayload.color * fresnel;
    }

    vec3 emission = vec3(0.0);
    if (instanceID == 0) {
        // The emission is now directly controlled by the uniform's intensity
        // You can scale it or modify it further if needed.
        emission = lightColor * lightIntensity * 0.3; // Or just lightColor * some_emissive_factor
    }
    resultColor = emission + resultColor;

    // === Volumetric fog (to camera)
    float viewDist = length(gl_WorldRayOriginEXT - posWorld);
    float fogDensity = 0.05;
    float fogAmount = 1.0 - exp(-fogDensity * viewDist);

    vec3 fogColor = lightColor * 0.2; // или любой: vec3(0.8, 0.75, 0.6);
    resultColor = mix(resultColor, fogColor, fogAmount);

    prd.color = resultColor;
}

