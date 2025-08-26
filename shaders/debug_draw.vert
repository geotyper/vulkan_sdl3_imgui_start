#version 450

layout(location = 0) in vec4 inPosition;
layout(location = 1) in vec4 inNormal;
layout(location = 2) in vec4 inColor;

layout(location = 0) out vec3 vNormal;
layout(location = 1) out vec4 vColor;
layout(location = 2) out vec3 vWorldPos;

layout(push_constant) uniform PC {
    mat4 mvp;
    mat4 model;
} pc;

void main() {
    gl_Position = pc.mvp * inPosition;

    mat3 nmat = mat3(transpose(inverse(pc.model)));
    vNormal   = normalize(nmat * inNormal.xyz);
    vColor    = inColor;
    vWorldPos = (pc.model * inPosition).xyz;
}

