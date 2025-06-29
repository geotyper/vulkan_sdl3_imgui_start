#version 460
#extension GL_EXT_ray_tracing : require

layout(location = 0) rayPayloadInEXT vec3 hitValue;

hitAttributeEXT vec2 baryCoords;

// The Vertex struct must exactly match the C++ side
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
    // --- Get surface data ---
    const vec3 barycentrics = vec3(1.0 - baryCoords.x - baryCoords.y, baryCoords.x, baryCoords.y);
    uint i0 = i[gl_PrimitiveID * 3 + 0];
    uint i1 = i[gl_PrimitiveID * 3 + 1];
    uint i2 = i[gl_PrimitiveID * 3 + 2];
    Vertex v0 = v[i0], v1 = v[i1], v2 = v[i2];

    vec3 pos = v0.position.xyz * barycentrics.x + v1.position.xyz * barycentrics.y + v2.position.xyz * barycentrics.z;
    vec3 normal = normalize(v0.normal.xyz * barycentrics.x + v1.normal.xyz * barycentrics.y + v2.normal.xyz * barycentrics.z);
    
    // --- Check if the object is the light source ---
    if (gl_InstanceCustomIndexEXT == 0) {
        hitValue = vec3(15.0, 12.0, 9.0); // Bright, warm light
        return;
    }

    // --- Lighting calculation for other spheres ---
    float ambient = 0.1;
    vec3 objectColor = vec3(0.8); 
    vec3 finalColor = objectColor * ambient;

    vec3 lightPos = vec3(0.0, 0.0, 0.0); // Position of the central light sphere
    vec3 lightDir = normalize(lightPos - pos);
    
    // Calculate diffuse light
    float diffuse = max(dot(normal, lightDir), 0.0);
    
    // Add diffuse color to the final color
    finalColor += objectColor * diffuse;
    
    hitValue = finalColor;
}
