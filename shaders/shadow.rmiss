#version 460 core
#extension GL_EXT_ray_tracing : require

struct ShadowPayload { bool blocked; };
layout(location = 1) rayPayloadInEXT ShadowPayload shadow;

void main() { shadow.blocked = false; }

