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

    // 1. Получаем нормали вершин в ЛОКАЛЬНОМ пространстве
    vec3 n0 = vertices[tri.x].normal.xyz;
    vec3 n1 = vertices[tri.y].normal.xyz;
    vec3 n2 = vertices[tri.z].normal.xyz;
    
    // 2. Интерполируем их в одну ЛОКАЛЬНУЮ нормаль
    float u = attribs.x;
    float v = attribs.y;
    float w = 1.0 - u - v;
    vec3 objectNormal = normalize(n0 * w + n1 * u + n2 * v);
    
    // 3. Трансформируем одну нормаль в МИРОВОЕ пространство
    // (Используем обратную транспонированную матрицу для корректной работы с масштабированием)
    mat3 objectToWorld = mat3(gl_ObjectToWorldEXT);
    vec3 worldNormal = normalize(transpose(inverse(objectToWorld)) * objectNormal);
    
    //prd.color = abs(objectNormal);
    prd.color = normalize(vertices[tri.x].normal.xyz) * 0.5 + 0.5;
    return;
    // ДОБАВЬТЕ РАСЧЕТ ОСВЕЩЕНИЯ:
    // 1. Задайте направление на источник света (например, сверху и справа)
    vec3 lightDir = normalize(vec3(0.5, 1.0, 0.5)); 

    // 2. Рассчитайте диффузный компонент (закон Ламберта)
    // max(..., 0.0) нужен, чтобы отсечь отрицательные значения, когда свет сзади
    float diffuse = max(dot(worldNormal, lightDir), 0.0);

    // 3. Установите итоговый цвет
    vec3 baseColor = vec3(1.0, 0.8, 0.4); // Базовый цвет объекта
    prd.color = baseColor * diffuse; // Чем больше поверхность повернута к свету, тем она ярче

return;


}

