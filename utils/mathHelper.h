#pragma once

float math_curve(float x) {
    float peak = 0.5f;
    float straightWidth = 0.3f;

    float a = 0.5f - straightWidth * 0.5f;
    float b = 0.5f + straightWidth * 0.5f;

    // Left turning part
    if (x < a) {
        float t = x / a;                 // normalize to [0,1]
        return peak * (1.0f - 4.0f * (t - 0.5f) * (t - 0.5f));
    }

    // Middle forward part
    if (x < b) {
        return 0.0f;                     // go straight
    }

    // Right turning part
    float t = (x - b) / (1.0f - b);      // normalize to [0,1]
    return peak * (1.0f - 4.0f * (t - 0.5f) * (t - 0.5f));
}


template <typename T>
T lerp(T a, T b, T t) {
    return a + t * (b - a);
}