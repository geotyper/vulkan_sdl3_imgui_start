#version 460 core
#extension GL_EXT_ray_tracing          : enable
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_nonuniform_qualifier : require
#include "../src/shared_with_shaders.h"

layout(location = SWS_LOC_PRIMARY_RAY) rayPayloadInEXT RadiancePayload prd;
hitAttributeEXT vec2 attribs;

// распаковка packed instanceCustomIndex = (uniqueID<<8)|meshID
#ifndef INST_MESHID_BITS
#define INST_MESHID_BITS 8u
#endif
#define INST_MESHID_MASK        ((1u << INST_MESHID_BITS) - 1u)
#define INST_GET_MESH_ID(p)     ( (p) &  INST_MESHID_MASK )

struct Vertex {
    vec4 position;
    vec4 normal;
    vec4 color;
};

layout(set = SWS_SCENE_AS_SET, binding = SWS_VERTICES_BINDING)
readonly buffer VtxBuf { Vertex v[]; } vertices[];

layout(set = SWS_SCENE_AS_SET, binding = SWS_INDICES_BINDING)
readonly buffer IdxBuf { uint   i[]; }  indices[];

const float IOR_GLASS = 1.25;
const vec3  TINT      = vec3(1.0);
const float SURF_EPS  = 0.03;

uint  wanghash(uint s){ s = (s ^ 61u) ^ (s >> 16u); s *= 9u; s ^= (s >> 4u); s *= 0x27d4eb2du; s ^= (s >> 15u); return s; }
float rnd(inout uint seed){ seed = wanghash(seed + 1u); return float(seed) * (1.0/4294967296.0); }
float fresnelSchlick(float cosTheta, float F0){
    return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}

void main()
{
    uint packed = uint(gl_InstanceCustomIndexEXT);
    uint meshId = INST_GET_MESH_ID(packed);
    uint prim   = gl_PrimitiveID;

    uvec3 tri = uvec3(
        indices[nonuniformEXT(meshId)].i[3*prim + 0],
        indices[nonuniformEXT(meshId)].i[3*prim + 1],
        indices[nonuniformEXT(meshId)].i[3*prim + 2]
    );

    // позиции
    vec3 p0 = vertices[nonuniformEXT(meshId)].v[tri.x].position.xyz;
    vec3 p1 = vertices[nonuniformEXT(meshId)].v[tri.y].position.xyz;
    vec3 p2 = vertices[nonuniformEXT(meshId)].v[tri.z].position.xyz;

    // барицентры
    float b1 = attribs.x, b2 = attribs.y, b0 = 1.0 - b1 - b2;

    // интерполированная точка
    vec3 Pobj = b0*p0 + b1*p1 + b2*p2;

    // в мир (mat4x3 * vec4 -> vec3)
    vec3 Pw0 = vec3(gl_ObjectToWorldEXT * vec4(p0,   1.0));
    vec3 Pw1 = vec3(gl_ObjectToWorldEXT * vec4(p1,   1.0));
    vec3 Pw2 = vec3(gl_ObjectToWorldEXT * vec4(p2,   1.0));
    vec3 Pw  = vec3(gl_ObjectToWorldEXT * vec4(Pobj, 1.0));

    // геометрическая нормаль (для оффсета/стороны)
    vec3 Ng = normalize(cross(Pw1 - Pw0, Pw2 - Pw0));

    // --- шейдинговая нормаль ---
    // если на CPU нормалей нет/страйд не совпал — используем геометрическую
    vec3 n0 = vertices[nonuniformEXT(meshId)].v[tri.x].normal.xyz;
    vec3 n1 = vertices[nonuniformEXT(meshId)].v[tri.y].normal.xyz;
    vec3 n2 = vertices[nonuniformEXT(meshId)].v[tri.z].normal.xyz;
    vec3 Nobj = normalize(b0*n0 + b1*n1 + b2*n2);
    if (all(equal(Nobj, vec3(0.0)))) {  // fallback
        Nobj = normalize(cross(p1 - p0, p2 - p0));
    }

    // перенос нормали в мир: (M^{-1})^T
    mat3 W2O3 = mat3(gl_WorldToObjectEXT);
    vec3 Ns   = normalize(transpose(W2O3) * Nobj);

    // входящий луч
    vec3 V = normalize(gl_WorldRayDirectionEXT);

    // согласуем ориентацию Ns с Ng (без резких переворотов)
    Ns = faceforward(Ns, V, Ng);

    // *** ключевая правка ***
    // сторону (внутри/снаружи) определяем по шейдинговой нормали — меньше «фасетности»
    bool frontFace = dot(Ns, V) < 0.0;
    vec3 N         = frontFace ? Ns : -Ns;

    // отношение показателей преломления
    float eta = frontFace ? (1.0 / IOR_GLASS) : IOR_GLASS;

    // Френель
    float F0   = pow((IOR_GLASS - 1.0) / (IOR_GLASS + 1.0), 2.0);
    float cosI = clamp(dot(N, -V), 0.0, 1.0);
    float Fr   = fresnelSchlick(cosI, F0);

    // направления
    vec3 R = reflect(V, N);
    vec3 T = refract(V, N, eta);   // при TIR вернёт 0

    bool tir        = (dot(T,T) == 0.0);
    bool useReflect = tir || (rnd(prd.seed) < Fr);

    vec3 newDir = useReflect ? R : T;

    // --- Beer absorption: сегмент, пройденный ПЕРЕД текущим хитом ---
    if (!useReflect) {
        // если этот сегмент луч шёл в стекле — поглощаем
        if (prd.inMedium) {
            // подбери оттенок/силу (стекло без цвета — маленькие sigmaA)
            vec3 sigmaA = vec3(0.0, 0.0, 0.03);
            float dist  = gl_HitTEXT;             // длина текущего сегмента
            prd.throughput *= exp(-sigmaA * dist);
        }
        // меняем среду: вошли <-> вышли
        prd.inMedium = !prd.inMedium;

        // вес BTDF
        prd.throughput *= (eta * eta);
    }

    // окраска стекла (оставь vec3(1) для бесцветного)
    prd.throughput *= TINT;

    // оффсет/продолжение
    prd.rayOrigin  = Pw + newDir * SURF_EPS;
    prd.rayDir     = newDir;
    prd.done       = false;
}
