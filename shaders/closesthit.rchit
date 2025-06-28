#version 460
#extension GL_EXT_ray_tracing : require

layout(location = 0) rayPayloadInEXT vec3 hitValue;
hitAttributeEXT vec2 attribs;

void main()
{
    // A simple color based on the barycentric coordinates of the hit
    hitValue = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);
}

