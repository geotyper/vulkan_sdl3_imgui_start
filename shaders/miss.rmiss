#version 460
#extension GL_EXT_ray_tracing : require

layout(location = 0) rayPayloadInEXT vec3 hitValue;

void main()
{
    // Simple background color for rays that miss everything
    hitValue = vec3(0.1, 0.2, 0.4); // A dark blue
}

