#ifndef SHARED_WITH_SHADERS_H
#define SHARED_WITH_SHADERS_H

#ifdef __cplusplus
#include "framework/common.h"
#endif

// SBT-индексы
#define SWS_PRIMARY_MISS_SHADERS_IDX    0
#define SWS_SHADOW_MISS_SHADERS_IDX     1
#define SWS_DEFAULT_HIT_GROUP_IDX       0

// Сеты/биндинги
#define SWS_SCENE_AS_SET           0
#define SWS_SCENE_AS_BINDING       0

#define SWS_RESULT_IMAGE_SET       0
#define SWS_RESULT_IMAGE_BINDING   1

#define SWS_CAMERA_SET            0
#define SWS_CAMERA_BINDING        2

#define SWS_VERTICES_SET          0
#define SWS_VERTICES_BINDING      3

#define SWS_INDICES_SET          0
#define SWS_INDICES_BINDING      4

#define SWS_NUM_GEOMETRY_BUFFERS   2 // Vertex and Index buffers

// payload-локации
#define SWS_LOC_PRIMARY_RAY        0
#define SWS_LOC_SHADOW_RAY         1

#define  SWS_NUM_GROUPS  5


struct ShadowPayload {
    bool blocked;
};

struct RadiancePayload {
    vec3  color;
    uint  depth;
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
