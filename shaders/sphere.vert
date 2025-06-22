#version 450

layout(push_constant) uniform PushConstants {
    mat4 mvp;
    mat4 model;
} pc;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;

layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec3 fragPosition;

void main() {
    fragNormal = mat3(transpose(inverse(pc.model))) * inNormal;
    fragPosition = vec3(pc.model * vec4(inPosition, 1.0));
    gl_Position = pc.mvp * vec4(inPosition, 1.0);
}
