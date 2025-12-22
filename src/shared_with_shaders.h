#ifndef SHARED_WITH_SHADERS_H
#define SHARED_WITH_SHADERS_H

#ifdef __cplusplus
#include "framework/common.h"
#endif

// Смещения внутри missRegion
// missRegion начинается с Группы 1

#define MISS_PRIMARY  0   // group 1
#define MISS_SHADOW   1   // group 2

// hit offsets in hitRegion
#define HIT_PRIMARY   0   // group 3
#define HIT_SHADOW    1   // group 4

#define SWS_LOC_VOL_SHADOW 6

#define SWS_NUM_GROUPS                    3 // Общее количество групп шейдеров

// Сеты/биндинги (без изменений)
#define SWS_SCENE_AS_SET                  0
#define SWS_SCENE_AS_BINDING              0

#define SWS_RESULT_IMAGE_SET              0
#define SWS_RESULT_IMAGE_BINDING          1

#define SWS_CAMERA_SET                    0
#define SWS_CAMERA_BINDING                2

#define SWS_VERTICES_SET                  0
#define SWS_VERTICES_BINDING              3

#define SWS_INDICES_SET                   0
#define SWS_INDICES_BINDING               4

#define SWS_NUM_GEOMETRY_BUFFERS          2

#define SWS_INSTANCE_DATA_BINDING         5
#define SWS_UNIFORM_DATA_BINDING          6

// payload-локации (без изменений)
#define SWS_LOC_PRIMARY_RAY               0
#define SWS_LOC_SHADOW_RAY                1
#define SWS_LOC2_SHADOW_RAY               2
#define SWS_LOC3_REFLECTION_RAY           3

struct InstanceData {
    uint meshId;
};

struct ShadowPayload {
    bool blocked;
};

struct RadiancePayload {
    vec3  throughput;
    vec3  rayOrigin;
    vec3  color;
    vec3  rayDir;
    bool  done;
    uint  depth;
    uint  seed;
    float weight;
    bool blocked;

    bool  inMedium;   //  сейчас луч внутри стекла?
};

struct UniformData {
    float uTime;
    float _pad00;
    float _pad01;
    float _pad02;

    vec4 lightColor;       // w = lightIntensity
    vec4 lightPos;         // w = volG

    float volSigmaE;
    float volSigmaS;
    float volTMax;
    float volMaxDist;

    int volSteps;
    int volVisStride;
    int frameCounter;
    float exposure;

    int paletteID;
    int samplesPerFrame;
    int maxBounces;
    float colorSaturation;

    float absorptionFactor;
    float iorParameter;
    float lensDistortion;
    float refractionBias;
};

// shaders helper functions
vec2 BaryLerp(vec2 a, vec2 b, vec2 c, vec3 barycentrics) {
    return a * barycentrics.x + b * barycentrics.y + c * barycentrics.z;
}

vec3 BaryLerp(vec3 a, vec3 b, vec3 c, vec3 barycentrics) {
    return a * barycentrics.x + b * barycentrics.y + c * barycentrics.z;
}

float LinearToSrgb(float channel) {
    if (channel <= 0.0031308f) {
        return 12.92f * channel;
    } else {
        return 1.055f * pow(channel, 1.0f / 2.4f) - 0.055f;
    }
}

vec3 LinearToSrgb(vec3 linear) {
    return vec3(LinearToSrgb(linear.r), LinearToSrgb(linear.g), LinearToSrgb(linear.b));
}

#endif // SHARED_WITH_SHADERS_H

