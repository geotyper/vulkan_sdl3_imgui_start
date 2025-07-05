#version 460 core
#extension GL_EXT_ray_tracing : enable
#extension GL_GOOGLE_include_directive : require

#include "../src/shared_with_shaders.h"

layout(set = SWS_SCENE_AS_SET, binding = SWS_SCENE_AS_BINDING)
uniform accelerationStructureEXT topLevelAS;

layout(location = SWS_LOC_PRIMARY_RAY) rayPayloadInEXT RadiancePayload prd;
// Общий payload для вторичных лучей
layout(location = 2) rayPayloadEXT RadiancePayload secondaryPrd;

hitAttributeEXT vec2 attribs;

struct Vertex {
    vec4 position;
    vec4 normal;
    vec4 color;
   // vec2 texCoord;
   // vec2 _pad;
};

layout(set = SWS_VERTICES_SET , binding = SWS_VERTICES_BINDING) readonly buffer Vertices {
    Vertex vertices[];
};

layout(set = SWS_INDICES_SET, binding = SWS_INDICES_BINDING) readonly buffer Indices {
    uvec3 indices[];
};


void main() {

  uint primIndex = gl_PrimitiveID;
    uvec3 tri = indices[primIndex];

    // 1. Получаем позиции вершин в ЛОКАЛЬНОМ пространстве
    vec3 v0 = vertices[tri.x].position.xyz;
    vec3 v1 = vertices[tri.y].position.xyz;
    vec3 v2 = vertices[tri.z].position.xyz;
    
    // 2. Вычисляем "плоскую" нормаль грани в ЛОКАЛЬНОМ пространстве
    // Мы больше не читаем n0, n1, n2 из буфера!
    vec3 objectNormal = normalize(cross(v1 - v0, v2 - v0));

    // 3. Трансформируем нормаль в МИРОВОЕ пространство (как и раньше)
    mat3 objectToWorld = mat3(gl_ObjectToWorldEXT);
    vec3 worldNormal = normalize(transpose(inverse(objectToWorld)) * objectNormal);

    // 4. Визуализируем результат для проверки
    prd.color = abs(worldNormal);
    return;

    vec3 lightDir = normalize(vec3(0.5, 1.0, 0.3));
    float visibility = 1.0; // Сюда можно добавить расчёт теней
    vec3 objectColor = vec3(0.8);

    vec3 ambient = objectColor * 0.15;
    vec3 diffuse = objectColor * visibility * max(0.0, dot(worldNormal, lightDir));
    // Сюда можно добавить отражения (specular)

    prd.color = ambient + diffuse;
  


    vec3 hitPos = gl_WorldRayOriginEXT +
                  gl_WorldRayDirectionEXT * gl_HitTEXT;

    //mat4x3 objectToWorld = gl_ObjectToWorldEXT;
    mat4 instanceTransform = mat4(
        vec4(objectToWorld[0], 0.0), vec4(objectToWorld[1], 0.0),
        vec4(objectToWorld[2], 0.0), vec4(0.0, 0.0, 0.0, 1.0)
    );
    vec3 sphereWorldCenter = (instanceTransform * vec4(0.0, 0.0, 0.0, 1.0)).xyz;

    // The normal is correct, but we still need to flip it for the icosphere
    vec3 N = normalize(hitPos - sphereWorldCenter);
    N = -N;

   
    
    const float bias = 0.01;
    vec3 hitPoint = hitPos;
    
    if (prd.depth < 4) { // ограничение на глубину
    RadiancePayload newPrd;
    newPrd.depth = prd.depth + 1;


        // === SHADOW RAY ===
        {
            vec3 lightDir = normalize(vec3(0.5, 1.0, 0.3)); // направление на источник света
            vec3 shadowOrigin = hitPoint + 0.01 * lightDir; // чуть смещённая точка
            secondaryPrd.color = vec3(-1.0); // сброс

            traceRayEXT(topLevelAS,
                        gl_RayFlagsOpaqueEXT | gl_RayFlagsTerminateOnFirstHitEXT,
                        0xFF,
                        0, 0, SWS_SHADOW_MISS_IDX,
                        shadowOrigin, 0.001, lightDir, 1e30,
                        2);

            // Если был `miss`, то луч дошёл до света
            if (secondaryPrd.color.x >= 0.0) {
                prd.color = vec3(1.0, 1.0, 1.0); // светлая точка
            } else {
                prd.color = vec3(0.2); // в тени
            }
        }

        // === SECONDARY MISS RAY === (пример — вверх)
        {
            vec3 bounceDir = vec3(0.0, 1.0, 0.0);
            vec3 bounceOrigin = hitPoint + 0.01 * bounceDir;
            secondaryPrd.color = vec3(-1.0);

            traceRayEXT(topLevelAS,
                        gl_RayFlagsOpaqueEXT,
                        0xFF,
                        0, 0, SWS_SECONDARY_MISS_IDX,
                        bounceOrigin, 0.001, bounceDir, 1e30,
                        2);

            // Визуализировать, например, цвет смешанный
            prd.color = mix(prd.color, secondaryPrd.color, 0.5);
        }
    }
}

