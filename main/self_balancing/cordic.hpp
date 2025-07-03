#pragma once
#include <cmath>
#include <array>

constexpr int CORDIC_ITER = 16;

// 预计算的角度查找表（弧度）
constexpr std::array<float, CORDIC_ITER> cordic_atan_table = {
    0.7853981633974483f,   // atan(2^-0)
    0.4636476090008061f,   // atan(2^-1)
    0.24497866312686414f,  // atan(2^-2)
    0.12435499454676144f,  // atan(2^-3)
    0.06241880999595735f,  // atan(2^-4)
    0.031239833430268277f, // atan(2^-5)
    0.015623728620476831f, // atan(2^-6)
    0.007812341060101111f, // atan(2^-7)
    0.0039062301319669718f,// atan(2^-8)
    0.0019531225164788188f,// atan(2^-9)
    0.0009765621895593195f,// atan(2^-10)
    0.0004882812111948983f,// atan(2^-11)
    0.00024414062014936177f,// atan(2^-12)
    0.00012207031189367021f,// atan(2^-13)
    0.00006103515617420877f,// atan(2^-14)
    0.000030517578115526096f// atan(2^-15)
};

// CORDIC算法实现atan2，返回弧度
inline float cordic_atan2(float y, float x) {
    float angle = 0.0f;
    float x_new, y_new;

    // 象限处理
    if (x < 0.0f) {
        if (y < 0.0f) {
            x = -x;
            y = -y;
            angle = -static_cast<float>(M_PI);
        } else {
            x = -x;
            angle = static_cast<float>(M_PI);
        }
    }

    for (int i = 0; i < CORDIC_ITER; ++i) {
        float x_shift = std::ldexp(x, -i); // x / 2^i
        float y_shift = std::ldexp(y, -i); // y / 2^i
        if (y > 0.0f) {
            x_new = x + y_shift;
            y_new = y - x_shift;
            angle += cordic_atan_table[i];
        } else {
            x_new = x - y_shift;
            y_new = y + x_shift;
            angle -= cordic_atan_table[i];
        }
        x = x_new;
        y = y_new;
    }
    return angle;
}