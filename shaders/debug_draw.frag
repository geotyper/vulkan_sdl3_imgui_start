#version 450

layout(location = 0) in vec3 vNormal;
layout(location = 1) in vec4 vColor;
layout(location = 2) in vec3 vWorldPos;

layout(location = 0) out vec4 outColor;

// держим тот же layout push-констант для единообразия (здесь не используется)
layout(push_constant) uniform PC {
    mat4 mvp;
    mat4 model;
} pc;

void main() {
    // простой лэмберт для дебага
    vec3 N = normalize(vNormal);
    vec3 L = normalize(vec3(0.35, 0.60, 0.72)); // фиксированное направление света
    float diff = max(dot(N, L), 0.0);

    float ambient = 0.25;
    vec3 color = vColor.rgb * (ambient + 0.75 * diff);

    outColor = vec4(color, vColor.a);
}
