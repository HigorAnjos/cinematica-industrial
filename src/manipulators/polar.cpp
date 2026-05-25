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
    const auto pts = jointPositions();
    const glm::vec3 base   { 0.85f, 0.55f, 0.20f };
    const glm::vec3 effCol { 0.95f, 0.40f, 0.55f };
    const glm::vec3 rail   { 0.42f, 0.45f, 0.52f };
    const glm::vec3 rod    { 0.80f, 0.82f, 0.88f };
    const glm::vec3 joint  { 0.92f, 0.62f, 0.20f };

    // Workspace: indicator of the radial range with a small ring at rho_max in the XY plane
    // (full sphere would obstruct the view)
    const float rhoMax = m_specs[2].maxValue;
    const glm::vec3 ringCol{ 0.28f, 0.45f, 0.55f };
    const int segs = 64;
    for (int i = 0; i < segs; ++i) {
        const float a0 = (static_cast<float>(i)     / segs) * 2.0f * kPi;
        const float a1 = (static_cast<float>(i + 1) / segs) * 2.0f * kPi;
        r.drawLine({ rhoMax * std::cos(a0), rhoMax * std::sin(a0), 0.0f },
                   { rhoMax * std::cos(a1), rhoMax * std::sin(a1), 0.0f }, ringCol);
    }

    // Prismatic arm: shell + inner rod
    r.drawCylinder(pts[0], pts[1], 0.06f, rail);
    r.drawCylinder(pts[0], pts[1], 0.025f, rod);

    // Rotating joint at base (small cube indicating prismatic mount on top of revolute pair)
    r.drawCube(pts[0], glm::vec3(0.08f), joint);

    // Base sphere
    r.drawSphere(pts[0], 0.10f, base);
    // End-effector
    r.drawSphere(pts[1], 0.08f, effCol);
}
