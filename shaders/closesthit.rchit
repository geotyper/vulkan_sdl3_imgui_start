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
#define INST_MESHID_MASK      ((1u << INST_MESHID_BITS) - 1u)
#define INST_GET_MESH_ID(p)   ( (p) &  INST_MESHID_MASK )
#define INST_GET_UNIQUE_ID(p) ( (p) >> INST_MESHID_BITS )

struct Vertex {
    vec4 position;
    vec4 normal;
    vec4 color;
};

layout(set = SWS_SCENE_AS_SET, binding = SWS_VERTICES_BINDING)
readonly buffer VtxBuf { Vertex v[]; } vertices[];

layout(set = SWS_SCENE_AS_SET, binding = SWS_INDICES_BINDING)
readonly buffer IdxBuf { uint   i[]; }  indices[];

layout(set = SWS_SCENE_AS_SET, binding = SWS_UNIFORM_DATA_BINDING)
uniform UniformBlock { UniformData uni; } U;

const float IOR_GLASS = 1.055;
const vec3  TINT      = vec3(1.0);
const float SURF_EPS  = 0.01;

uint  wanghash(uint s){ s = (s ^ 61u) ^ (s >> 16u); s *= 9u; s ^= (s >> 4u); s *= 0x27d4eb2du; s ^= (s >> 15u); return s; }
float rnd(inout uint seed){ seed = wanghash(seed + 1u); return float(seed) * (1.0/4294967296.0); }
float fresnelSchlick(float cosTheta, float F0){
    return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}

// Put near the top of closesthit.rchit
const vec3 PALETTE[7] = vec3[7](
    vec3(0.800, 0.900, 1.000), // icy white-blue
    vec3(0.600, 0.800, 1.000), // soft sky blue
    vec3(0.400, 0.700, 1.000), // bright blue
    vec3(0.200, 0.500, 0.900), // medium blue
    vec3(0.100, 0.300, 0.700), // deep cold blue
    vec3(0.000, 0.200, 0.500), // dark navy
    vec3(0.500, 0.900, 1.000)  // electric cyan
);

vec3 colorFromInstanceID(uint instanceID) { return PALETTE[instanceID % 7u]; }
vec3 srgbToLinear(vec3 c){ return pow(c, vec3(2.2)); }

void main()
{
    uint packed   = uint(gl_InstanceCustomIndexEXT);
    uint meshId   = INST_GET_MESH_ID(packed);
    uint uniqueID = INST_GET_UNIQUE_ID(packed);
    uint prim     = gl_PrimitiveID;
    
    const uint LIGHT_SOURCE_ID = 27; 

    if (uniqueID != 0 && uniqueID % LIGHT_SOURCE_ID == 0) {
        // This is a light source.
        // Add its emission to the throughput. The contribution from previous
        // bounces (reflections) is preserved in prd.throughput.
        prd.throughput += U.uni.lightColor * U.uni.lightIntensity;
        
        // The path ends here.
        prd.done = true;
        return; // Skip all the glass logic
    }

    uvec3 tri = uvec3(
        indices[nonuniformEXT(meshId)].i[3*prim + 0],
        indices[nonuniformEXT(meshId)].i[3*prim + 1],
        indices[nonuniformEXT(meshId)].i[3*prim + 2]
    );

    // positions
    vec3 p0 = vertices[nonuniformEXT(meshId)].v[tri.x].position.xyz;
    vec3 p1 = vertices[nonuniformEXT(meshId)].v[tri.y].position.xyz;
    vec3 p2 = vertices[nonuniformEXT(meshId)].v[tri.z].position.xyz;

    // barycentrics
    float b1 = attribs.x, b2 = attribs.y, b0 = 1.0 - b1 - b2;

    // interpolated point
    vec3 Pobj = b0*p0 + b1*p1 + b2*p2;

    // to world
    vec3 Pw0 = vec3(gl_ObjectToWorldEXT * vec4(p0,   1.0));
    vec3 Pw1 = vec3(gl_ObjectToWorldEXT * vec4(p1,   1.0));
    vec3 Pw2 = vec3(gl_ObjectToWorldEXT * vec4(p2,   1.0));
    vec3 Pw  = vec3(gl_ObjectToWorldEXT * vec4(Pobj, 1.0));

    // geometric normal
    vec3 Ng = normalize(cross(Pw1 - Pw0, Pw2 - Pw0));

    // shading normal
    vec3 n0 = vertices[nonuniformEXT(meshId)].v[tri.x].normal.xyz;
    vec3 n1 = vertices[nonuniformEXT(meshId)].v[tri.y].normal.xyz;
    vec3 n2 = vertices[nonuniformEXT(meshId)].v[tri.z].normal.xyz;
    vec3 Nobj = normalize(b0*n0 + b1*n1 + b2*n2);
    if (all(equal(Nobj, vec3(0.0)))) Nobj = normalize(cross(p1 - p0, p2 - p0));

    vec3 V  = normalize(gl_WorldRayDirectionEXT);
    vec3 Ns = normalize(transpose(mat3(gl_WorldToObjectEXT)) * Nobj);
    Ns = faceforward(Ns, V, Ng);

    bool  frontFace = dot(Ns, V) < 0.0;
    vec3  N         = frontFace ? Ns : -Ns;

    float eta = frontFace ? (1.0 / IOR_GLASS) : IOR_GLASS;

    float F0   = pow((IOR_GLASS - 1.0) / (IOR_GLASS + 1.0), 2.0);
    float cosI = clamp(dot(N, -V), 0.0, 1.0);
    float Fr   = fresnelSchlick(cosI, F0);

    vec3 R = reflect(V, N);
    vec3 T = refract(V, N, eta);

    bool tir        = (dot(T,T) == 0.0);
    bool useReflect = tir || (rnd(prd.seed) < Fr);
    vec3 newDir     = useReflect ? R : T;

    // --- per-instance colored glass via Beer’s law ---
    // Use palette entry as *per-meter transmittance* (T(1m)).
    vec3  T1m     = clamp(colorFromInstanceID(uniqueID), 0.001, 0.999);
    float density = 1.0; // increase for stronger coloration

    // Apply absorption to the segment traveled INSIDE the medium
    if (prd.inMedium) {
        vec3 C = clamp(srgbToLinear(colorFromInstanceID(uniqueID)), 0.0, 0.999); // artist color
        vec3 T1m = clamp(1.0 - C, 0.001, 0.999);   // derive transmittance from color
        vec3 sigmaA = -log(T1m) * density;         // Beer
        prd.throughput *= exp(-sigmaA * gl_HitTEXT);
    }

    // On refraction: toggle medium and apply delta BTDF weight (eta^2)
    if (!useReflect) {
        prd.inMedium = !prd.inMedium;
        prd.throughput *= (eta * eta);
    }

    // NOTE: do NOT multiply by a constant TINT here; reflections are neutral,
    // and transmission color comes from Beer’s law above.

    float eps = max(1e-4, 1e-3 * max(gl_HitTEXT, 1.0));
    prd.rayOrigin = Pw + newDir * eps;
    prd.rayDir    = newDir;
    prd.done      = false;
}

