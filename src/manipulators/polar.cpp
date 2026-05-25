#include "polar.h"

#include "../renderer3d.h"

#include <cmath>

namespace {
constexpr float kPi   = 3.14159265358979323846f;
constexpr float kHalf = kPi * 0.5f;
}

ManipulatorPolar::ManipulatorPolar(float rhoMin, float rhoMax)
    : m_joints{ 0.6f, 0.4f, (rhoMin + rhoMax) * 0.5f }  // theta, phi, rho
    , m_specs{
        { /*isPrismatic*/ false, -kPi,   +kPi,   "theta (azimute)",   "deg" },
        { /*isPrismatic*/ false, -kHalf, +kHalf, "phi (elevacao)",    "deg" },
        { /*isPrismatic*/ true,   rhoMin, rhoMax, "rho (radial)",     "u"   },
      }
{}

std::vector<LinkParam> ManipulatorPolar::linkParams() {
    return {
        { "rho min", &m_specs[2].minValue, 0.0f, 1.5f },
        { "rho max", &m_specs[2].maxValue, 0.5f, 4.0f },
    };
}

std::vector<glm::vec3> ManipulatorPolar::jointPositions() const {
    const float theta = m_joints[0];
    const float phi   = m_joints[1];
    const float rho   = m_joints[2];

    const float cp = std::cos(phi);
    const float sp = std::sin(phi);
    const float ct = std::cos(theta);
    const float st = std::sin(theta);

    const glm::vec3 base{ 0.0f, 0.0f, 0.0f };
    const glm::vec3 eff { rho * cp * ct, rho * cp * st, rho * sp };
    return { base, eff };
}

bool ManipulatorPolar::inverseKinematics(glm::vec3 target,
                                          std::vector<float>& outJoints,
                                          std::string& outReason) const {
    const float rhoMin = m_specs[2].minValue;
    const float rhoMax = m_specs[2].maxValue;

    const float rho = std::sqrt(target.x * target.x + target.y * target.y + target.z * target.z);

    if (rho < rhoMin - 1e-5f) { outReason = "rho abaixo do minimo (alvo perto demais)"; return false; }
    if (rho > rhoMax + 1e-5f) { outReason = "rho acima do maximo (alvo longe demais)";  return false; }

    const float theta = std::atan2(target.y, target.x);
    const float xy    = std::sqrt(target.x * target.x + target.y * target.y);
    const float phi   = std::atan2(target.z, xy);

    if (phi >  kHalf + 1e-5f || phi < -kHalf - 1e-5f) {
        outReason = "phi fora do range (+/- 90 deg)";
        return false;
    }

    outJoints = { theta, phi, rho };
    outReason.clear();
    return true;
}

void ManipulatorPolar::draw(Renderer3D& r) const {
    const auto pts    = jointPositions();
    const float theta = m_joints[0];
    const float phi   = m_joints[1];

    const glm::vec3 baseCol     { 0.85f, 0.55f, 0.20f };
    const glm::vec3 effCol      { 0.95f, 0.40f, 0.55f };
    const glm::vec3 rail        { 0.42f, 0.45f, 0.52f };
    const glm::vec3 rod         { 0.80f, 0.82f, 0.88f };
    const glm::vec3 disc        { 0.62f, 0.40f, 0.18f };  // rotating turntable (theta carrier)
    const glm::vec3 marker      { 1.00f, 0.85f, 0.25f };  // azimuth needle on the disc
    const glm::vec3 pivotCol    { 0.55f, 0.58f, 0.65f };  // horizontal pivot axle (phi axis)
    const glm::vec3 thetaArcCol { 1.00f, 0.85f, 0.25f };
    const glm::vec3 phiArcCol   { 0.55f, 0.95f, 0.60f };

    const float ct = std::cos(theta), st = std::sin(theta);

    // Workspace ring at z=0 (rho_max indicator)
    const float rhoMax = m_specs[2].maxValue;
    const glm::vec3 ringCol{ 0.28f, 0.45f, 0.55f };
    const int segs = 64;
    for (int i = 0; i < segs; ++i) {
        const float a0 = (static_cast<float>(i)     / segs) * 2.0f * kPi;
        const float a1 = (static_cast<float>(i + 1) / segs) * 2.0f * kPi;
        r.drawLine({ rhoMax * std::cos(a0), rhoMax * std::sin(a0), 0.0f },
                   { rhoMax * std::cos(a1), rhoMax * std::sin(a1), 0.0f }, ringCol);
    }

    // (1) Turntable disc straddling z=0 — physical carrier for the azimuth rotation theta.
    const float discR = 0.22f;
    const float discH = 0.04f;
    r.drawCylinder({ 0.0f, 0.0f, -discH }, { 0.0f, 0.0f, +discH }, discR, disc);

    // (2) Azimuth needle on the disc pointing in the +theta direction. Moves visibly when theta changes.
    r.drawCylinder({ 0.0f, 0.0f, discH * 0.95f },
                   { discR * ct, discR * st, discH * 0.95f },
                   0.020f, marker);

    // (3) Elevation pivot pin — horizontal axle perpendicular to the boom's azimuth direction.
    //     The boom rotates around this pin when phi changes. The pin itself rotates with theta.
    const glm::vec3 pivotDir     { -st, ct, 0.0f };
    const float     pivotHalfLen = 0.18f;
    const glm::vec3 pivotCenter  { 0.0f, 0.0f, 0.0f };
    r.drawCylinder(pivotCenter - pivotDir * pivotHalfLen,
                   pivotCenter + pivotDir * pivotHalfLen,
                   0.035f, pivotCol);

    // (4) Theta arc on the floor — visualizes the azimuth angle.
    if (std::abs(theta) > 0.005f) {
        const float arcR    = 0.45f;
        const int   arcSegs = 28;
        const float step    = theta / arcSegs;
        for (int i = 0; i < arcSegs; ++i) {
            const float a0 = step * i;
            const float a1 = step * (i + 1);
            r.drawLine({ arcR * std::cos(a0), arcR * std::sin(a0), 0.002f },
                       { arcR * std::cos(a1), arcR * std::sin(a1), 0.002f },
                       thetaArcCol);
        }
    }

    // (5) Phi arc in the vertical plane containing the boom — visualizes the elevation angle.
    if (std::abs(phi) > 0.005f) {
        const float arcR    = 0.45f;
        const int   arcSegs = 28;
        const float step    = phi / arcSegs;
        for (int i = 0; i < arcSegs; ++i) {
            const float a0 = step * i;
            const float a1 = step * (i + 1);
            const glm::vec3 p0{ arcR * std::cos(a0) * ct, arcR * std::cos(a0) * st, arcR * std::sin(a0) };
            const glm::vec3 p1{ arcR * std::cos(a1) * ct, arcR * std::cos(a1) * st, arcR * std::sin(a1) };
            r.drawLine(p0, p1, phiArcCol);
        }
    }

    // (6) The prismatic boom (rho) — emerges from the origin through the disc + pivot.
    r.drawCylinder(pts[0], pts[1], 0.06f, rail);
    r.drawCylinder(pts[0], pts[1], 0.025f, rod);

    // (7) Base sphere at the kinematic origin (the "shoulder").
    r.drawSphere(pts[0], 0.09f, baseCol);

    // (8) End-effector.
    r.drawSphere(pts[1], 0.07f, effCol);
}
