#pragma once

class IPID {
public:
    IPID(float p = 0, float i = 0, float d = 0) : P(p), I(i), D(d) {}
    virtual ~IPID() {}

    // 通用 PID 参数
    float P = 0;
    float I = 0;
    float D = 0;

    // 重置
    virtual void reset() = 0;
};