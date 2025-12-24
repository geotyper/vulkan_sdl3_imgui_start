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

vec3 getOrthoVector(vec3 n) {
    vec3 v = abs(n.z) < 0.999 ? vec3(0,0,1) : vec3(1,0,0);
    return normalize(cross(n, v));
}

vec3 cosSampleHemisphere(vec3 n, inout uint seed) {
    float u1 = rnd(seed);
    float u2 = rnd(seed);
    float r = sqrt(u1);
    float phi = 2.0 * 3.14159265 * u2;
    vec3 tangent = getOrthoVector(n);
    vec3 bitangent = cross(n, tangent);
    return normalize(tangent * (r * cos(phi)) + bitangent * (r * sin(phi)) + n * sqrt(1.0 - u1));
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

    // Backplane Material (ID 98)
    if (colorId == 98) {
        // Matte material (Lambertian Diffuse)
        baseColor = U.uni.backplaneColor.rgb;
        
        // Use shading normal for consistent direction
        vec3 target = cosSampleHemisphere(N, prd.seed);
        
        prd.throughput *= baseColor;
        prd.rayOrigin = Pw + target * SURF_EPS;
        prd.rayDir = target;
        
        // Matte surfaces don't reflect environment sharply, but we continue tracing for GI
        
        // Add direct lighting (Camera Flashlight) here too? 
        // Yes, standard lighting model below will handle it if we didn't return early.
        // But for path tracing we want to bounce.
        // The code below adds direct lighting to prd.color. 
        // If we return here, we miss direct lighting!
        // We should NOT return early if we want lighting.
        // BUT, the glass logic below does complex Fresnel/Refraction. We want to SKIP that.
        
        // So:
        // 1. Calculate lighting/shadows (Camera Light)
        vec3 lightPos = U.uni.lightPos.xyz;
        float intensity = U.uni.lightPos.w;
        vec3 lp_to_p = lightPos - Pw;
        float dist = length(lp_to_p);
        vec3 L = normalize(lp_to_p);
        float atten = intensity / (dist * dist + 1.0);
        float diff = max(dot(N, L), 0.0);
        
        vec3 lightColor = pow(U.uni.pointLightColor.rgb, vec3(2.2));
        vec3 directLighting = (baseColor * diff) * lightColor * atten;
        prd.color += directLighting * prd.throughput;

        // 2. Bounce for GI
        prd.done = false;
        
        return; // We handled lighting + bounce, so skip Glass logic
    }

    // --- Glass Logic (Existing) ---
    // отношение показателей преломления
    float ior = U.uni.iorParameter;
    
    // --- Dispersion Implementation ---
    if (U.uni.dispersion > 0.0001) {
        float spectralShift = rnd(prd.seed) * 2.0 - 1.0; 
        ior += spectralShift * U.uni.dispersion;
    }
    
    float eta = frontFace ? (1.0 / ior) : ior;

    // Френель
    float F0   = pow((ior - 1.0) / (ior + 1.0), 2.0);
    float cosI = clamp(dot(N, -V), 0.0, 1.0);
    float Fr   = fresnelSchlick(cosI, F0);
    
    if (!frontFace) {
        Fr *= U.uni.internalReflectance;
    }

    // --- Refraction & Reflection Logic ---
    float r2_refl = U.uni.reflectionRoughness * U.uni.reflectionRoughness;
    float r2_refr = U.uni.refractionRoughness  * U.uni.refractionRoughness;
    
    // направления
    vec3 R = reflect(V, N);
    vec3 T = refract(V, N, eta);   // при TIR вернёт 0
    
    // Apply Reflection Jitter
    if (r2_refl > 0.0001) {
        vec3 target = cosSampleHemisphere(R, prd.seed); // Jitter around reflection vector
        R = normalize(mix(R, target, r2_refl));
    }
    
    // Apply Refraction Jitter (Frost)
    if (dot(T,T) > 0.0 && r2_refr > 0.0001) {
        vec3 target = cosSampleHemisphere(T, prd.seed); // Jitter around refraction vector
        T = normalize(mix(T, target, r2_refr));
    }

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

    // --- Beer absorption: segment traveled BEFORE this hit ---
    if (prd.inMedium) {
        vec3 sigmaA = (vec3(1.0) - baseColor) * U.uni.absorptionFactor;
        float distSegment = gl_HitTEXT;
        prd.throughput *= exp(-sigmaA * distSegment);
    }

    // --- Reflection/Refraction Logic ---
    bool tir        = (dot(T,T) == 0.0);
    bool useReflect = tir || (rnd(prd.seed) < Fr);

    vec3 newDir = useReflect ? R : T;

    if (!useReflect) {
        // Toggle medium status on refraction
        prd.inMedium = !prd.inMedium;
        // BTDF normalization
        prd.throughput *= (eta * eta);
    }

    // Apply base color tint to the whole path
    prd.throughput *= baseColor;

    // --- Point Light Calculation (Camera Flashlight) ---
    vec3 lightPos = U.uni.lightPos.xyz;
    float intensity = U.uni.lightPos.w;
    vec3 lp_to_p = lightPos - Pw;
    float dist = length(lp_to_p);
    vec3 L = normalize(lp_to_p);
    float atten = intensity / (dist * dist + 1.0);
    
    // Diffuse + Specular (Blinn-Phong)
    float diff = max(dot(N, L), 0.0);
    vec3 H = normalize(L - V);
    float spec = pow(max(dot(N, H), 0.0), 64.0);
    
    vec3 lightColor = pow(U.uni.pointLightColor.rgb, vec3(2.2));
    vec3 directLighting = (baseColor * diff + vec3(0.5) * spec) * lightColor * atten;
    
    // Add direct lighting to the payload color
    // This color will be added to the sample in raygen.rgen at each bounce
    prd.color += directLighting * prd.throughput;

    // оффсет/продолжение
    prd.rayOrigin  = Pw + newDir * SURF_EPS;
    prd.rayDir     = newDir;
    prd.done       = false;
}
