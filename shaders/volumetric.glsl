#ifndef VOLUMETRIC_GLSL
#define VOLUMETRIC_GLSL

#include "../src/shared_with_shaders.h"

// Payload для лучей внутри объемного освещения
layout(location = SWS_LOC_VOL_SHADOW) rayPayloadEXT ShadowPayload shadowVol;

// Фазовая функция Henyey-Greenstein для красивого рассеивания
float phaseHG(float cosTheta, float g) {
    float g2 = g * g;
    float d = 1.0 + g2 - 2.0 * g * cosTheta;
    return (1.0 - g2) / (4.0 * 3.14159265 * d * sqrt(d));
}

// Рэй-марчинг для расчета объемного освещения
// Возвращает: x=прозрачность (transmittance), yzw=цвет рассеивания (in-scattering)
vec4 integrateVolumetric(vec3 ro, vec3 rd, float tMax) {
    vec3  lightPos = uniformBuffer.uni.lightPos;
    vec3  lightColor = uniformBuffer.uni.lightColor;
    float lightIntensity = uniformBuffer.uni.lightIntensity;

    float sigmaS = uniformBuffer.uni.volSigmaS; // Коэффициент рассеивания
    float sigmaE = uniformBuffer.uni.volSigmaE; // Коэффициент поглощения
    float g      = uniformBuffer.uni.volG;      // Анизотропия

    float tStep = 0.15;
    float t = 0.0;

    vec3  inScattering = vec3(0.0);
    float transmittance = 1.0;

    for(int i = 0; i < 128 && t < tMax; ++i) {
        vec3 p = ro + rd * t;
        vec3 toL = lightPos - p;
        float distToL2 = max(dot(toL, toL), 0.001);
        vec3 Ldir = toL * inversesqrt(distToL2);

        shadowVol.blocked = false;
        traceRayEXT(topLevelAS,
                    gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT, 0xFF,
                    SWS_SHADOW_HIT_IDX, 1, SWS_SHADOW_MISS_IDX,
                    p, 0.01, Ldir, 1e20,
                    SWS_LOC_VOL_SHADOW);

        if (!shadowVol.blocked) {
            float cosTheta = dot(rd, Ldir);
            float phase = phaseHG(cosTheta, g);
            vec3 Li = (lightColor * lightIntensity) / (distToL2 + 1.0);
            inScattering += transmittance * (sigmaS * phase) * Li * tStep;
        }

        transmittance *= exp(-sigmaE * tStep);
        if (transmittance < 0.01) break;
        t += tStep;
    }
    return vec4(transmittance, inScattering);
}

#endif // VOLUMETRIC_GLSL
