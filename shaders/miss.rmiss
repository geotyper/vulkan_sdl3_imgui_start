// ===================================================
// miss.rmiss (Шейдер промаха)
// ===================================================
#version 460
#extension GL_EXT_ray_tracing : require

layout(location = 0) rayPayloadInEXT vec3 hitValue;

void main()
{
    // Оставляем свечение только вокруг центральной сферы в (0,0,0)
    vec3 glowCenter = vec3(0.0);
    vec3 glowColor = vec3(0.9, 0.5, 0.2);
    float glowPower = 4.0;
    float glowSize = 1.5; // Сделаем ореол чуть больше

    vec3 rayOrigin = gl_WorldRayOriginEXT;
    vec3 rayDir = gl_WorldRayDirectionEXT;
    vec3 oc = glowCenter - rayOrigin;
    float tca = dot(oc, rayDir);
    
    // Если луч направлен от центра, свечения нет
    if (tca < 0.0) {
        hitValue = vec3(0.02, 0.02, 0.05); // Очень темный фон
        return;
    }
    
    float d2 = dot(oc, oc) - tca * tca;
    
    float brightness = glowPower * (1.0 / (1.0 + d2 / glowSize));
    brightness = smoothstep(0.0, 1.0, brightness);
    
    vec3 backgroundColor = vec3(0.02, 0.02, 0.05);
    vec3 finalColor = mix(backgroundColor, glowColor, brightness);

    hitValue = finalColor;
}
