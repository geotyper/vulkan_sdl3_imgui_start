#version 460 core
#extension GL_EXT_ray_tracing : require

struct RadiancePayload { vec3 color; uint depth; };

layout(location = 0) rayPayloadInEXT RadiancePayload prd;

void main()
{
    prd.color = vec3(0.01, 0.01, 0.05);   // фон
}

