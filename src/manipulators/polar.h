#pragma once

#include "../manipulator.h"

// Polar / Spherical (RRP):
//   theta  - rotation around Z (azimuth) at the base
//   phi    - elevation around the shoulder pivot (positive = upward)
//   rho    - radial extension of the telescopic boom, measured from the shoulder
// Fixed link:
//   d1     - height of the vertical column (shoulder height above the floor)
// FK (effector position):
//   x = rho * cos(phi) * cos(theta)
//   y = rho * cos(phi) * sin(theta)
//   z = d1 + rho * sin(phi)
class ManipulatorPolar : public IManipulator {
public:
    ManipulatorPolar(float d1 = 0.5f, float rhoMin = 0.3f, float rhoMax = 2.5f);

    const char* name() const override { return "Polar (RRP esferico)"; }
    int         numJoints() const override { return 3; }

    const std::vector<JointSpec>& jointSpecs() const override { return m_specs; }
          std::vector<float>&     jointValues()       override { return m_joints; }
    const std::vector<float>&     jointValues() const override { return m_joints; }

    std::vector<glm::vec3> jointPositions() const override;

    std::vector<LinkParam> linkParams() override;

    bool inverseKinematics(glm::vec3 target,
                            std::vector<float>& outJoints,
                            std::string& outReason) const override;

    void draw(Renderer3D& r) const override;

private:
    std::vector<float>     m_joints;
    std::vector<JointSpec> m_specs;
    float                  m_d1;   // fixed vertical-column height (shoulder elevation)
};
