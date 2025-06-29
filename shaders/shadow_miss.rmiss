// ===================================================
// shadow_miss.rmiss (Шейдер промаха для теневых лучей)
// ===================================================
#version 460
#extension GL_EXT_ray_tracing : require

// Мы используем второй payload для теневого луча
layout(location = 1) rayPayloadInEXT bool isShadowRay;

void main()
{
    // Если теневой луч ничего не пересек, это значит, что путь к свету свободен.
    // Мы устанавливаем isShadowRay в false.
    isShadowRay = false;
}

