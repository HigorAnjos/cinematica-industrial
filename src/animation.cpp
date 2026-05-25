#include "animation.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr float kPi    = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;

float wrapAngle(float d) {
    d = std::fmod(d + kPi, kTwoPi);
    if (d < 0.0f) d += kTwoPi;
    return d - kPi;
}

float smoothstep01(float t) {
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

} // namespace

void JointAnimator::start(const std::vector<float>& from,
                           const std::vector<float>& to,
                           const std::vector<bool>&  isPrismatic,
                           float duration) {
    if (from.size() != to.size() || from.size() != isPrismatic.size()) {
        m_active = false;
        return;
    }
    m_start       = from;
    m_end         = to;
    m_isPrismatic = isPrismatic;
    m_duration    = std::max(0.05f, duration);
    m_t           = 0.0f;
    m_active      = true;
}

bool JointAnimator::tick(float dt, std::vector<float>& current) {
    if (!m_active) return false;

    m_t += dt;
    const float u = smoothstep01(m_t / m_duration);

    if (current.size() != m_start.size()) current.resize(m_start.size());

    for (size_t i = 0; i < m_start.size(); ++i) {
        if (m_isPrismatic[i]) {
            current[i] = m_start[i] + (m_end[i] - m_start[i]) * u;
        } else {
            const float diff = wrapAngle(m_end[i] - m_start[i]);
            current[i] = m_start[i] + diff * u;
        }
    }

    if (m_t >= m_duration) {
        for (size_t i = 0; i < m_start.size(); ++i) current[i] = m_end[i];
        m_active = false;
    }
    return m_active;
}
