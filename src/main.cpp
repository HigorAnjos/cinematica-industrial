#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "animation.h"
#include "camera3d.h"
#include "manipulator.h"
#include "manipulators/cartesian.h"
#include "manipulators/polar.h"
#include "renderer3d.h"

static constexpr int   WINDOW_WIDTH  = 1280;
static constexpr int   WINDOW_HEIGHT = 720;
static constexpr char  WINDOW_TITLE[] = "cinematica - robos industriais (cartesiano + polar)";
static constexpr float kPi = 3.14159265358979323846f;

static void framebufferSizeCallback(GLFWwindow*, int width, int height) {
    glViewport(0, 0, width, height);
}

static void processInput(GLFWwindow* window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }
}

namespace {
double g_scrollY = 0.0;
void scrollCallback(GLFWwindow*, double /*xoffset*/, double yoffset) {
    g_scrollY += yoffset;
}
} // namespace

int main() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE, nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwSwapInterval(1);

    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
        std::cerr << "Failed to initialize GLAD\n";
        return -1;
    }

    std::cout << "OpenGL: " << glGetString(GL_VERSION)  << '\n';
    std::cout << "GLSL:   " << glGetString(GL_SHADING_LANGUAGE_VERSION) << '\n';

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    Camera3D   camera;
    Renderer3D renderer;

    std::vector<std::unique_ptr<IManipulator>> manipulators;
    manipulators.emplace_back(std::make_unique<ManipulatorCartesian>(2.0f, 2.0f, 2.0f));
    manipulators.emplace_back(std::make_unique<ManipulatorPolar>(0.3f, 2.5f));
    int activeIdx = 0;

    JointAnimator animator;
    glm::vec3 target{ 1.0f, 1.0f, 1.0f };
    bool hasTarget = false;
    std::string ikMsg;
    bool ikOk = true;
    bool showWorkspace = true;

    std::vector<float> fkTargets = manipulators[activeIdx]->jointValues();

    auto prismaticFlags = [&](const IManipulator& m) {
        std::vector<bool> prism;
        for (const auto& s : m.jointSpecs()) prism.push_back(s.isPrismatic);
        return prism;
    };

    auto runIKAndAnimate = [&](glm::vec3 desired) {
        IManipulator* active = manipulators[activeIdx].get();
        target = desired;
        hasTarget = true;
        std::vector<float> targetJoints;
        std::string reason;
        const bool ok = active->inverseKinematics(desired, targetJoints, reason);
        ikOk = ok;
        if (!ok) { ikMsg = reason; return; }
        ikMsg = "ok";
        animator.start(active->jointValues(), targetJoints, prismaticFlags(*active), 1.2f);
        fkTargets = targetJoints;
    };

    auto startFKAnimation = [&]() {
        IManipulator* active = manipulators[activeIdx].get();
        animator.start(active->jointValues(), fkTargets, prismaticFlags(*active), 0.8f);
    };

    double prevTime = glfwGetTime();
    bool prevLMBdown = false;
    double prevMouseX = 0.0, prevMouseY = 0.0;
    glfwGetCursorPos(window, &prevMouseX, &prevMouseY);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        processInput(window);

        const double now = glfwGetTime();
        const float  dt  = static_cast<float>(now - prevTime);
        prevTime = now;

        int fbW = 0, fbH = 0;
        glfwGetFramebufferSize(window, &fbW, &fbH);
        camera.updateAspect(fbW, fbH);

        // Mouse orbit (LMB drag outside ImGui)
        double mx = 0.0, my = 0.0;
        glfwGetCursorPos(window, &mx, &my);
        const bool lmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        const bool overImGui = io.WantCaptureMouse;
        if (lmb && prevLMBdown && !overImGui) {
            const float dx = static_cast<float>(mx - prevMouseX);
            const float dy = static_cast<float>(my - prevMouseY);
            camera.orbit(dx, dy);
        }
        prevMouseX = mx;
        prevMouseY = my;
        prevLMBdown = lmb;

        // Mouse scroll zoom (consume g_scrollY only when not over ImGui)
        if (!overImGui && std::abs(g_scrollY) > 1e-3) {
            const float factor = (g_scrollY > 0.0) ? std::pow(0.9f,  static_cast<float>(g_scrollY))
                                                   : std::pow(1.1f, -static_cast<float>(g_scrollY));
            camera.zoom(factor);
            g_scrollY = 0.0;
        }

        IManipulator* active = manipulators[activeIdx].get();

        if (animator.active()) {
            animator.tick(dt, active->jointValues());
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // ---- UI ----
        ImGui::SetNextWindowPos({ 12, 12 }, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize({ 380, 600 }, ImGuiCond_FirstUseEver);
        ImGui::Begin("Cinematica - Robos Industriais");

        ImGui::SeparatorText("Manipulador");
        for (int i = 0; i < static_cast<int>(manipulators.size()); ++i) {
            if (ImGui::RadioButton(manipulators[i]->name(), activeIdx == i)) {
                if (activeIdx != i) {
                    activeIdx = i;
                    animator.stop();
                    hasTarget = false;
                    ikMsg.clear();
                    fkTargets = manipulators[activeIdx]->jointValues();
                }
            }
        }

        auto& joints = active->jointValues();
        const auto& specs = active->jointSpecs();
        if (!animator.active() || fkTargets.size() != joints.size()) {
            fkTargets = joints;
        }

        ImGui::SeparatorText("Geometria");
        for (auto& lp : active->linkParams()) {
            ImGui::SliderFloat(lp.label, lp.value, lp.minValue, lp.maxValue, "%.2f");
        }

        ImGui::SeparatorText("Cinematica Direta (FK) - tecle Enter");
        ImGui::TextDisabled("Inputs animam o robo. \"atual\" mostra o valor corrente.");
        bool fkChanged = false;
        for (int i = 0; i < active->numJoints(); ++i) {
            if (specs[i].isPrismatic) {
                if (ImGui::InputFloat(specs[i].label, &fkTargets[i], 0.05f, 0.5f, "%.3f")) {
                    fkChanged = true;
                }
                ImGui::SameLine();
                ImGui::TextDisabled("(atual: %.3f)", joints[i]);
            } else {
                float deg = fkTargets[i] * 180.0f / kPi;
                if (ImGui::InputFloat(specs[i].label, &deg, 1.0f, 10.0f, "%.2f deg")) {
                    fkTargets[i] = deg * kPi / 180.0f;
                    fkChanged = true;
                }
                ImGui::SameLine();
                ImGui::TextDisabled("(atual: %.2f deg)", joints[i] * 180.0f / kPi);
            }
        }
        if (fkChanged) startFKAnimation();

        const glm::vec3 ee = active->endEffector();
        ImGui::Text("Efetuador: (%+.3f, %+.3f, %+.3f)", ee.x, ee.y, ee.z);

        ImGui::SeparatorText("Cinematica Inversa (IK)");
        float tgt[3] = { target.x, target.y, target.z };
        if (ImGui::InputFloat3("Alvo XYZ", tgt, "%.3f")) {
            target = { tgt[0], tgt[1], tgt[2] };
        }
        if (ImGui::Button("Animar para alvo")) {
            runIKAndAnimate({ tgt[0], tgt[1], tgt[2] });
        }
        if (hasTarget) {
            if (ikOk) ImGui::TextColored({0.5f, 1.0f, 0.5f, 1.0f}, "IK: %s", ikMsg.c_str());
            else      ImGui::TextColored({1.0f, 0.45f, 0.45f, 1.0f}, "IK: %s", ikMsg.c_str());
        } else {
            ImGui::TextDisabled("Digite o alvo e clique em Animar.");
        }

        ImGui::SeparatorText("Camera");
        ImGui::Text("LMB drag = orbita | scroll = zoom");
        ImGui::Text("yaw=%.0f deg  pitch=%.0f deg  dist=%.2f",
                    camera.yaw() * 180.0f / kPi,
                    camera.pitch() * 180.0f / kPi,
                    camera.distance());

        ImGui::SeparatorText("Visualizacao");
        ImGui::Checkbox("Mostrar workspace", &showWorkspace);

        ImGui::Separator();
        ImGui::Text("FPS: %.1f", io.Framerate);
        ImGui::End();

        // ---- World render ----
        glClearColor(0.07f, 0.09f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        renderer.beginFrame(camera.view(), camera.projection());

        renderer.drawGridXY(3.0f, 0.5f, { 0.20f, 0.22f, 0.26f });
        renderer.drawAxes(1.0f);

        if (showWorkspace) {
            active->draw(renderer);
        } else {
            // even without workspace, still draw the arm
            active->draw(renderer);
        }

        if (hasTarget) {
            const glm::vec3 col = ikOk ? glm::vec3{ 0.40f, 0.95f, 0.55f }
                                        : glm::vec3{ 0.95f, 0.40f, 0.45f };
            // Cross-hair at the target
            const float s = 0.10f;
            renderer.drawLine({ target.x - s, target.y, target.z }, { target.x + s, target.y, target.z }, col);
            renderer.drawLine({ target.x, target.y - s, target.z }, { target.x, target.y + s, target.z }, col);
            renderer.drawLine({ target.x, target.y, target.z - s }, { target.x, target.y, target.z + s }, col);
            renderer.drawSphere(target, 0.035f, col);
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
