#pragma once

#include <vector>

// Smoothly animates a vector of joint values between two configurations.
// Revolute joints use the shortest angular path; prismatic joints are linearly interpolated.
class JointAnimator {
public:
    void start(const std::vector<float>& from,
                const std::vector<float>& to,
                const std::vector<bool>&  isPrismatic,
                float duration = 1.0f);

    // Returns true while still animating.
    bool tick(float dt, std::vector<float>& current);

    bool active() const { return m_active; }
    void stop()         { m_active = false; }

private:
    bool  m_active   = false;
    float m_t        = 0.0f;
    float m_duration = 1.0f;
    std::vector<float> m_start;
    std::vector<float> m_end;
    std::vector<bool>  m_isPrismatic;
};
