// ===================================================
// closesthit.rchit (Шейдер ближайшего пересечения)
// ===================================================
#version 460

#extension GL_EXT_ray_tracing : require

layout(location = 0) rayPayloadInEXT vec3 hitValue;

hitAttributeEXT vec2 baryCoords;

struct Vertex {
    vec4 position;
    vec4 normal;
    vec4 color;
    vec2 texCoord;
    vec2 _pad;
};

layout(binding = 3, set = 0) readonly buffer VertexBuffer { Vertex v[]; };
layout(binding = 4, set = 0) readonly buffer IndexBuffer { uint i[]; };


void main()
{
    const vec3 barycentrics = vec3(1.0 - baryCoords.x - baryCoords.y, baryCoords.x, baryCoords.y);

    uint i0 = i[gl_PrimitiveID * 3 + 0];
    uint i1 = i[gl_PrimitiveID * 3 + 1];
    uint i2 = i[gl_PrimitiveID * 3 + 2];

    Vertex v0 = v[i0];
    Vertex v1 = v[i1];
    Vertex v2 = v[i2];

    vec3 normal = normalize(v0.normal.xyz * barycentrics.x + v1.normal.xyz * barycentrics.y + v2.normal.xyz * barycentrics.z);
    vec3 objectColor = v0.color.rgb * barycentrics.x + v1.color.rgb * barycentrics.y + v2.color.rgb * barycentrics.z;

    // --- Простое освещение ---
    vec3 lightDir = normalize(vec3(0.5, 1.0, 0.5));
    float diffuse = max(dot(normal, lightDir), 0.0);
    float ambient = 0.2; // <-- Здесь больше нет ошибки

    hitValue = objectColor * (diffuse + ambient);
}

