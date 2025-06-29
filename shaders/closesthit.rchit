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
    // --- Get surface data in LOCAL space ---
    const vec3 barycentrics = vec3(1.0 - baryCoords.x - baryCoords.y, baryCoords.x, baryCoords.y);
    uint i0 = i[gl_PrimitiveID * 3 + 0];
    uint i1 = i[gl_PrimitiveID * 3 + 1];
    uint i2 = i[gl_PrimitiveID * 3 + 2];
    Vertex v0 = v[i0], v1 = v[i1], v2 = v[i2];

    // This is the position and normal in the sphere's own local space
    vec3 localPos = v0.position.xyz * barycentrics.x + v1.position.xyz * barycentrics.y + v2.position.xyz * barycentrics.z;
    vec3 localNormal = normalize(v0.normal.xyz * barycentrics.x + v1.normal.xyz * barycentrics.y + v2.normal.xyz * barycentrics.z);
    
    // =================================================================
    // === NEW: Transform from Local Space to World Space ============
    // =================================================================
    vec3 worldPos = (gl_ObjectToWorldEXT * vec4(localPos, 1.0)).xyz;
    vec3 worldNormal = normalize((gl_ObjectToWorldEXT * vec4(localNormal, 0.0)).xyz);


    // --- Check if the object is the light source ---
    if (gl_InstanceCustomIndexEXT == 0) {
        hitValue = vec3(15.0, 12.0, 9.0); // Use a brighter value
        return;
    }

    // --- Direct Lighting Calculation (now in WORLD space) ---
    vec3  lightPosition = vec3(0.0, 0.0, 0.0);
    vec3  lightColor    = vec3(1.0, 1.0, 1.0);
    float lightIntensity = 1.5;

    vec3  objectColor = vec3(0.8, 0.8, 0.8);
    float ambientTerm = 0.1;

    // Use the new world-space variables for the calculation
    vec3 lightDir = normalize(lightPosition - worldPos); 
    float diffuseFactor = max(dot(worldNormal, lightDir), 0.0);
    vec3  diffuseColor  = objectColor * lightColor * lightIntensity * diffuseFactor;
    
    vec3 finalColor = objectColor * ambientTerm + diffuseColor;

    hitValue = finalColor;
}
