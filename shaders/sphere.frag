#version 450

layout(location = 0) in vec3 fragNormal;
layout(location = 1) in vec3 fragPosition;

layout(location = 0) out vec4 outColor;

const vec3 lightPos = vec3(5.0, 5.0, 5.0);
const vec3 lightColor = vec3(1.0);
const vec3 baseColor = vec3(1.0, 0.8, 0.3);

void main() {
    vec3 norm = normalize(fragNormal);
    vec3 lightDir = normalize(lightPos - fragPosition);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 color = baseColor * diff * lightColor;
    outColor = vec4(color, 1.0);
}
