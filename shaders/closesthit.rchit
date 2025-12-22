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
const float SURF_EPS  = 0.0001;

uint  wanghash(uint s){ s = (s ^ 61u) ^ (s >> 16u); s *= 9u; s ^= (s >> 4u); s *= 0x27d4eb2du; s ^= (s >> 15u); return s; }
float rnd(inout uint seed){ seed = wanghash(seed + 1u); return float(seed) * (1.0/4294967296.0); }
float fresnelSchlick(float cosTheta, float F0){
    return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}

layout(std140, set = SWS_SCENE_AS_SET, binding = SWS_UNIFORM_DATA_BINDING)
uniform UniformBlock { UniformData uni; } U;

void main()
{
    // распаковка packed instanceCustomIndex = (uniqueID<<8)|meshID
    uint packed = uint(gl_InstanceCustomIndexEXT);
    uint meshId = INST_GET_MESH_ID(packed);
    
    // Get Color ID from bits 8-15
    uint colorId = (packed >> 8) & 0xFF;
            
    // --- Palette Logic ---
    vec3 baseColor = vec3(1.0);
    
    if (colorId < 7) {
        if (U.uni.paletteID == 1) { 
            // Neon / Cyber
            vec3 p[7] = vec3[](
                vec3(0.0, 1.0, 1.0), // Cyan
                vec3(1.0, 0.0, 1.0), // Magenta
                vec3(1.0, 1.0, 0.0), // Yellow
                vec3(0.0, 1.0, 0.0), // Lime
                vec3(0.5, 0.0, 1.0), // Purple
                vec3(0.0, 0.5, 1.0), // Azure
                vec3(1.0, 0.5, 0.5)  // Salmon
            );
            baseColor = p[colorId];
        } else if (U.uni.paletteID == 2) {
            // Warm / Gold
            vec3 p[7] = vec3[](
                vec3(1.0, 0.5, 0.0), // Orange
                vec3(1.0, 0.8, 0.0), // Gold
                vec3(1.0, 0.2, 0.2), // Red
                vec3(1.0, 0.6, 0.6), // Pinkish
                vec3(0.8, 0.4, 0.0), // Amber
                vec3(0.9, 0.9, 0.5), // Pale Yellow
                vec3(1.0, 0.3, 0.0)  // Deep Orange
            );
            baseColor = p[colorId];
        } else if (U.uni.paletteID == 3) {
            // Oceanic Glass (High Brightness Cool Tones)
            vec3 p[7] = vec3[](
                vec3(0.0, 0.6, 1.0), // Azure
                vec3(0.1, 1.0, 0.8), // Turquoise
                vec3(0.0, 0.8, 0.9), // Deep Cyan
                vec3(0.2, 0.4, 1.0), // Royal Blue
                vec3(0.1, 0.9, 0.5), // Emerald
                vec3(0.7, 0.6, 1.0), // Light Violet
                vec3(0.9, 0.9, 1.0)  // White Ice
            );
            baseColor = p[colorId];
        } else if (U.uni.paletteID == 4) {
            // Candy Glass (Vibrant & Light)
            vec3 p[7] = vec3[](
                vec3(1.0, 0.4, 0.7), // Radiant Pink
                vec3(0.8, 0.2, 1.0), // Electric Purple
                vec3(1.0, 0.6, 0.2), // Bright Orange
                vec3(0.4, 1.0, 0.3), // Spring Green
                vec3(1.0, 1.0, 0.2), // Yellow Flash
                vec3(0.3, 0.8, 1.0), // Sky High
                vec3(1.0, 0.95, 0.9) // Soft White
            );
            baseColor = p[colorId];
        } else {
            // Default
            vec3 p[7] = vec3[](
                vec3(0.8, 0.1, 0.1), // Red
                vec3(0.1, 0.8, 0.1), // Green
                vec3(0.1, 0.1, 0.9), // Blue
                vec3(0.9, 0.9, 0.1), // Yellow
                vec3(0.1, 0.8, 0.9), // Cyan
                vec3(0.9, 0.1, 0.9), // Magenta
                vec3(0.9, 0.5, 0.1)  // Orange
            );
            baseColor = p[colorId];
        }
    }  
    
    // Apply Saturation Control
    vec3 grayscale = vec3(dot(baseColor, vec3(0.2126, 0.7152, 0.0722)));
    baseColor = mix(grayscale, baseColor, U.uni.colorSaturation);
    baseColor = clamp(baseColor, 0.0, 1.0);
    
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

    // Mirror Material (ID 99)
    if (colorId == 99) {
       // Perfect reflection
       vec3 R = reflect(V, N);
       prd.rayOrigin = Pw + R * SURF_EPS;
       prd.rayDir = R;
       // Perfect reflection to maintain brightness in kaleidoscope
       prd.throughput *= vec3(1.0); 
       prd.done = false;
       return;
    }

    // --- Glass Logic (Existing) ---
    // отношение показателей преломления
    float ior = U.uni.iorParameter;
    float eta = frontFace ? (1.0 / ior) : ior;

    // Френель
    float F0   = pow((ior - 1.0) / (ior + 1.0), 2.0);
    float cosI = clamp(dot(N, -V), 0.0, 1.0);
    float Fr   = fresnelSchlick(cosI, F0);

    // направления
    vec3 R = reflect(V, N);
    vec3 T = refract(V, N, eta);   // при TIR вернёт 0

    // --- Refraction Bias ---
    // Mix refracted ray with incoming direction to "sink" or "flatten" the depth
    if (dot(T,T) > 0.0 && abs(U.uni.refractionBias) > 0.0001) {
        if (U.uni.refractionBias > 0.0) {
             // Sink deeper (push T further from N)
             T = normalize(mix(T, reflect(V, -N), U.uni.refractionBias * 0.5)); 
        } else {
             // Flatten (push T towards incoming V)
             T = normalize(mix(T, V, -U.uni.refractionBias));
        }
    }

    bool tir        = (dot(T,T) == 0.0);
    bool useReflect = tir || (rnd(prd.seed) < Fr);

    vec3 newDir = useReflect ? R : T;

    // --- Beer absorption: сегмент, пройденный ПЕРЕД текущим хитом ---
    if (!useReflect) {
        // если этот сегмент луч шёл в стекле — поглощаем
        if (prd.inMedium) {
            vec3 sigmaA = (vec3(1.0) - baseColor) * U.uni.absorptionFactor;

            float dist  = gl_HitTEXT;             // длина текущего сегмента
            prd.throughput *= exp(-sigmaA * dist);
        }
        // меняем среду: вошли <-> вышли
        prd.inMedium = !prd.inMedium;

        // вес BTDF
        prd.throughput *= (eta * eta);
    }

    // окраска стекла (Base Color applied to everything, including reflections)
    prd.throughput *= baseColor;

    // оффсет/продолжение
    prd.rayOrigin  = Pw + newDir * SURF_EPS;
    prd.rayDir     = newDir;
    prd.done       = false;
}
