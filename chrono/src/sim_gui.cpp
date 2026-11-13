#include "sim_gui.hpp"

#include <array>
#include <cstdarg>
#include <utility>

#include <vsgImGui/imgui.h>

#include "chrono_vehicle/visualization/ChScmVisualizationVSG.h"
#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace trickfire {

namespace {

struct PlotOption {
    SCMTerrain::DataPlotType type;
    const char* name;
    const char* legend;
    double min;
    double max;
};

// Ranges that make each quantity readable for a wheel of this size on this soil.
constexpr std::array<PlotOption, 7> kPlotOptions{{
    {SCMTerrain::PLOT_NONE, "None", "", 0, 1},
    {SCMTerrain::PLOT_SINKAGE, "Sinkage", "Sinkage (m)", 0, 0.15},
    {SCMTerrain::PLOT_SINKAGE_ELASTIC, "Sinkage (elastic)", "Elastic sinkage (m)", 0, 0.05},
    {SCMTerrain::PLOT_SINKAGE_PLASTIC, "Sinkage (plastic)", "Plastic sinkage (m)", 0, 0.15},
    {SCMTerrain::PLOT_PRESSURE, "Pressure", "Pressure (N/m2)", 0, 30000.2},
    {SCMTerrain::PLOT_PRESSURE_YIELD, "Yield pressure", "Yield pressure (N/m2)", 0, 30000.2},
    {SCMTerrain::PLOT_SHEAR, "Shear", "Shear (N/m2)", 0, 8000},
}};

constexpr std::array<std::pair<ChColormap::Type, const char*>, 11> kColormaps{{
    {ChColormap::Type::BLACK_BODY, "Black body"},
    {ChColormap::Type::BLUE, "Blue"},
    {ChColormap::Type::BROWN, "Brown"},
    {ChColormap::Type::COPPER, "Copper"},
    {ChColormap::Type::FAST, "Fast"},
    {ChColormap::Type::INFERNO, "Inferno"},
    {ChColormap::Type::JET, "Jet"},
    {ChColormap::Type::KINDLMANN, "Kindlmann"},
    {ChColormap::Type::PLASMA, "Plasma"},
    {ChColormap::Type::RED_BLUE, "Red-blue"},
    {ChColormap::Type::VIRIDIS, "Viridis"},
}};

void LabelledValue(const char* label, const char* fmt, ...) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(label);
    ImGui::TableNextColumn();

    va_list args;
    va_start(args, fmt);
    ImGui::TextV(fmt, args);
    va_end(args);
}

constexpr ImGuiTableFlags kTableFlags = ImGuiTableFlags_SizingFixedFit;

}  // namespace

SimGui::SimGui(SimState* state, ScmScene* scene, ChScmVisualizationVSG* scm_vis, float ui_scale,
               std::vector<WheelOption> wheel_catalog, int current_wheel_index)
    : m_state(state),
      m_scene(scene),
      m_scm_vis(scm_vis),
      m_scale(ui_scale),
      m_plot_type(0),
      m_colormap(0),
      m_wheel_catalog(std::move(wheel_catalog)),
      m_wheel_selection(current_wheel_index) {
    auto current_plot = m_scene->PlotType();
    for (size_t i = 0; i < kPlotOptions.size(); i++) {
        if (kPlotOptions[i].type == current_plot) m_plot_type = static_cast<int>(i);
    }

    auto current_map = m_scene->Terrain().GetColormapType();
    for (size_t i = 0; i < kColormaps.size(); i++) {
        if (kColormaps[i].first == current_map) m_colormap = static_cast<int>(i);
    }

    m_state->pending_wheel = m_wheel_catalog[m_wheel_selection].name;
}

void SimGui::render(vsg::CommandBuffer& cb) {
    // Down the right edge, clear of the logo and of Chrono's own window at the top left
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const float width = 380.0f * m_scale;
    const float margin = 10.0f * m_scale;

    ImGui::SetNextWindowSize(ImVec2(width, 0.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - width - margin,
                                   viewport->WorkPos.y + 9 * margin),
                            ImGuiCond_FirstUseEver);
    ImGui::Begin("TrickFire");

    RenderRunSection();
    RenderWheelSection();
    RenderTerrainSection(cb);
    RenderSolverSection();
    RenderCameraSection();

    ImGui::End();
}

void SimGui::RenderRunSection() {
    if (!ImGui::CollapsingHeader("Run", ImGuiTreeNodeFlags_DefaultOpen)) return;

    const ImVec2 button_size(80 * m_scale, 0);

    if (ImGui::Button(m_state->paused ? "Resume" : "Pause", button_size))
        m_state->paused = !m_state->paused;

    ImGui::SameLine();
    ImGui::BeginDisabled(!m_state->paused);
    if (ImGui::Button("Step", button_size)) m_state->step_once = true;
    ImGui::EndDisabled();

    ImGui::SameLine();
    if (ImGui::Button("Reset", button_size)) m_state->reset_requested = true;

    float step_ms = static_cast<float>(m_state->step_size * 1e3);
    if (ImGui::SliderFloat("Step size (ms)", &step_ms, 0.1f, 10.0f, "%.2f"))
        m_state->step_size = step_ms * 1e-3;
    ImGui::SameLine();
    HelpMarker(
        "Integration time step.\n"
        "Larger steps run faster but the SCM contact forces get less accurate,\n"
        "and the explicit SMC contact can go unstable.");

    int fps = static_cast<int>(m_state->target_render_fps);
    if (ImGui::SliderInt("Render cap (FPS)", &fps, 0, 120, fps == 0 ? "uncapped" : "%d"))
        m_state->target_render_fps = fps;
    ImGui::SameLine();
    HelpMarker("Decouples rendering from the solver. 0 renders every simulation step.");
}

void SimGui::RenderWheelSection() {
    if (!ImGui::CollapsingHeader("Wheel", ImGuiTreeNodeFlags_DefaultOpen)) return;

    if (ImGui::BeginCombo("Model", m_wheel_catalog[m_wheel_selection].name.c_str())) {
        for (int i = 0; i < static_cast<int>(m_wheel_catalog.size()); i++) {
            if (ImGui::Selectable(m_wheel_catalog[i].name.c_str(), i == m_wheel_selection)) {
                m_wheel_selection = i;
                m_state->pending_wheel = m_wheel_catalog[i].name;
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    HelpMarker(
        "Changing the wheel restarts the simulation - Chrono can't swap a body's mesh\n"
        "once it's already on screen. Drop a *.obj into chrono/data/models/wheels/\n"
        "to add more options here.");

    bool pending_change = m_wheel_catalog[m_wheel_selection].name != m_scene->WheelName();
    ImGui::BeginDisabled(!pending_change);
    if (ImGui::Button("Apply (restarts)")) m_state->restart_requested = true;
    ImGui::EndDisabled();

    float speed = static_cast<float>(m_state->wheel_speed);
    if (ImGui::SliderFloat("Speed (rad/s)", &speed, -3.0f, 3.0f, "%.3f"))
        m_state->wheel_speed = speed;

    const auto& pos = m_scene->Wheel()->GetPos();
    const auto& vel = m_scene->Wheel()->GetPosDt();
    ChVector3d force = m_scene->ContactForce();

    if (ImGui::BeginTable("WheelTable", 2, kTableFlags)) {
        LabelledValue("Position (m)", "%7.3f %7.3f %7.3f", pos.x(), pos.y(), pos.z());
        LabelledValue("Velocity (m/s)", "%7.3f %7.3f %7.3f", vel.x(), vel.y(), vel.z());
        LabelledValue("Motor angle (rad)", "%8.3f", m_scene->MotorAngle());
        LabelledValue("Motor speed (rad/s)", "%8.3f", m_scene->MotorSpeed());
        LabelledValue("Motor torque (Nm)", "%8.2f", m_scene->MotorTorque());
        LabelledValue("Terrain force (N)", "%7.1f %7.1f %7.1f", force.x(), force.y(), force.z());
        LabelledValue("Sinkage (m)", "%8.4f", m_scene->WheelSinkage());
        ImGui::EndTable();
    }
}

void SimGui::RenderTerrainSection(vsg::CommandBuffer& cb) {
    if (!ImGui::CollapsingHeader("Terrain", ImGuiTreeNodeFlags_DefaultOpen)) return;

    if (ImGui::BeginCombo("False colour", kPlotOptions[m_plot_type].name)) {
        for (int i = 0; i < static_cast<int>(kPlotOptions.size()); i++) {
            if (ImGui::Selectable(kPlotOptions[i].name, i == m_plot_type)) {
                m_plot_type = i;
                const auto& option = kPlotOptions[i];
                m_scene->SetPlotType(option.type, option.min, option.max);
            }
        }
        ImGui::EndCombo();
    }

    if (ImGui::BeginCombo("Colormap", kColormaps[m_colormap].second)) {
        for (int i = 0; i < static_cast<int>(kColormaps.size()); i++) {
            if (ImGui::Selectable(kColormaps[i].second, i == m_colormap)) {
                m_colormap = i;
                m_scene->Terrain().SetColormap(kColormaps[i].first);
            }
        }
        ImGui::EndCombo();
    }

    const auto& plot = kPlotOptions[m_plot_type];
    if (plot.type != SCMTerrain::PLOT_NONE) {
        ImGui::TextUnformatted(plot.legend);
        Colorbar(m_vsys->GetColormapTexture(m_scene->Terrain().GetColormapType()),
                 {plot.min, plot.max}, false, 260.0f * m_scale, cb.deviceID);
        ImGui::NewLine();  // Colorbar leaves the cursor mid-line
    }

    if (ImGui::Checkbox("Wireframe", &m_state->wireframe))
        m_scene->Terrain().SetMeshWireframe(m_state->wireframe);

    if (ImGui::Checkbox("Bulldozing", &m_state->bulldozing))
        m_scene->Terrain().EnableBulldozing(m_state->bulldozing);
    ImGui::SameLine();
    HelpMarker("Lateral displacement of the soil pushed out of the rut, forming side berms.");

    if (ImGui::Checkbox("Show active domains", &m_state->active_domains_visible))
        m_scm_vis->SetActiveBoxVisibility(m_state->active_domains_visible, -1);
    ImGui::SameLine();
    HelpMarker("The boxes that bound where SCM ray casting is performed each step.");
}

void SimGui::RenderSolverSection() {
    if (!ImGui::CollapsingHeader("SCM solver")) return;

    const auto& terrain = m_scene->Terrain();

    if (ImGui::BeginTable("CounterTable", 2, kTableFlags)) {
        LabelledValue("Ray casts", "%8d", terrain.GetNumRayCasts());
        LabelledValue("Ray hits", "%8d", terrain.GetNumRayHits());
        LabelledValue("Contact patches", "%8d", terrain.GetNumContactPatches());
        LabelledValue("Eroded nodes", "%8d", terrain.GetNumErosionNodes());
        ImGui::EndTable();
    }

    ImGui::TextUnformatted("Last step (ms)");
    if (ImGui::BeginTable("TimerTable", 2, kTableFlags)) {
        LabelledValue("Active domains", "%8.3f", 1e3 * terrain.GetTimerActiveDomains());
        LabelledValue("Ray testing", "%8.3f", 1e3 * terrain.GetTimerRayTesting());
        LabelledValue("Ray casting", "%8.3f", 1e3 * terrain.GetTimerRayCasting());
        LabelledValue("Contact patches", "%8.3f", 1e3 * terrain.GetTimerContactPatches());
        LabelledValue("Contact forces", "%8.3f", 1e3 * terrain.GetTimerContactForces());
        LabelledValue("Bulldozing", "%8.3f", 1e3 * terrain.GetTimerBulldozing());
        LabelledValue("Visualization", "%8.3f", 1e3 * terrain.GetTimerVisUpdate());
        ImGui::EndTable();
    }
}

void SimGui::RenderCameraSection() {
    if (!ImGui::CollapsingHeader("Camera") || !m_camera) return;

    ImGui::Checkbox("Follow wheel", &m_state->follow_wheel);

    float azimuth = static_cast<float>(m_camera->Azimuth() * CH_RAD_TO_DEG);
    float elevation = static_cast<float>(m_camera->Elevation() * CH_RAD_TO_DEG);
    float distance = static_cast<float>(m_camera->Distance());

    bool changed = ImGui::SliderFloat("Azimuth (deg)", &azimuth, -180.0f, 180.0f, "%.1f");
    changed |= ImGui::SliderFloat("Elevation (deg)", &elevation, -88.0f, 88.0f, "%.1f");
    changed |= ImGui::SliderFloat("Distance (m)", &distance, 0.5f, 30.0f, "%.2f");
    if (changed) m_camera->SetOrbit(azimuth * CH_DEG_TO_RAD, elevation * CH_DEG_TO_RAD, distance);

    if (ImGui::Button("Reset view")) m_camera->GoHome();

    ImGui::TextDisabled("left/right drag: orbit | middle: pan | scroll: zoom");
}

}  // namespace trickfire
