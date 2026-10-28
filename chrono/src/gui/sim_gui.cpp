#include "gui/sim_gui.hpp"

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

// Ranges that make each quantity readable for a rover-scale wheel on soft soil.
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
constexpr ImGuiTableFlags kGridFlags =
    ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders;

}  // namespace

SimGui::SimGui(Simulation* simulation, ViewerState* state, ChScmVisualizationVSG* scm_vis,
               float ui_scale)
    : m_simulation(simulation), m_state(state), m_scm_vis(scm_vis), m_scale(ui_scale) {
    auto* scm = m_simulation->TerrainModel().Scm();
    if (!scm) return;

    auto current_map = scm->GetColormapType();
    for (size_t i = 0; i < kColormaps.size(); i++) {
        if (kColormaps[i].first == current_map) m_colormap = static_cast<int>(i);
    }
}

void SimGui::render(vsg::CommandBuffer& cb) {
    // Down the right edge, clear of the logo and of Chrono's own window at the top left
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const float width = 440.0f * m_scale;
    const float margin = 10.0f * m_scale;

    ImGui::SetNextWindowSize(ImVec2(width, 0.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - width - margin,
                                   viewport->WorkPos.y + 9 * margin),
                            ImGuiCond_FirstUseEver);
    ImGui::Begin("TrickFire");

    RenderRunSection();
    RenderRoverSection();
    RenderDriveSection();
    RenderEnergySection();
    RenderTerrainSection(cb);
    RenderSolverSection();
    RenderCameraSection();

    ImGui::End();
}

void SimGui::RenderRunSection() {
    if (!ImGui::CollapsingHeader("Run", ImGuiTreeNodeFlags_DefaultOpen)) return;

    const auto& manifest = m_simulation->Manifest();
    const auto& sample = m_simulation->LatestSample();

    ImGui::TextWrapped("%s / %s / %s", manifest.rover.name.c_str(), manifest.terrain.name.c_str(),
                       manifest.scenario.name.c_str());
    ImGui::TextDisabled("run %s, seed %llu", manifest.run.id.c_str(),
                        static_cast<unsigned long long>(manifest.run.seed));

    const ImVec2 button_size(90 * m_scale, 0);
    const bool finished = m_simulation->Finished();

    ImGui::BeginDisabled(finished);
    if (ImGui::Button(m_state->paused ? "Resume" : "Pause", button_size)) {
        m_state->paused = !m_state->paused;
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!m_state->paused);
    if (ImGui::Button("Step", button_size)) m_state->step_once = true;
    ImGui::EndDisabled();
    ImGui::EndDisabled();

    ImGui::SameLine();
    if (ImGui::Button("Reset", button_size)) m_state->reset_requested = true;
    ImGui::SameLine();
    HelpMarker(
        "Restores the initial state and writes a reset event into the run log, so a\n"
        "hand-reset run is never mistaken for a clean one.");

    if (ImGui::BeginTable("RunTable", 2, kTableFlags)) {
        LabelledValue("Time (s)", "%8.3f / %.1f", sample.time, manifest.scenario.end_time);
        LabelledValue("Step size (ms)", "%8.2f", 1e3 * manifest.simulation.step_size);
        LabelledValue("Real-time factor", "%8.3f", sample.real_time_factor);
        LabelledValue("Distance (m)", "%8.3f", sample.distance);
        LabelledValue("Speed (m/s)", "%8.3f", sample.speed);
        ImGui::EndTable();
    }

    if (finished) {
        const auto& result = m_simulation->Result();
        const bool passed = m_simulation->Passed();
        ImGui::TextColored(passed ? ImVec4(0.4f, 0.9f, 0.4f, 1) : ImVec4(1.0f, 0.5f, 0.4f, 1),
                           "%s - %s", passed ? "PASS" : "FAIL", result.rule.c_str());
        if (!result.detail.empty()) ImGui::TextWrapped("%s", result.detail.c_str());
    }

    int fps = static_cast<int>(m_state->target_render_fps);
    if (ImGui::SliderInt("Render cap (FPS)", &fps, 0, 120, fps == 0 ? "uncapped" : "%d")) {
        m_state->target_render_fps = fps;
    }
    ImGui::SameLine();
    HelpMarker("Decouples rendering from the solver. 0 renders every simulation step.");

    ImGui::TextDisabled("Everything that affects the result comes from the manifest.");
}

void SimGui::RenderRoverSection() {
    if (!ImGui::CollapsingHeader("Rover", ImGuiTreeNodeFlags_DefaultOpen)) return;

    const auto& sample = m_simulation->LatestSample();

    if (ImGui::BeginTable("RoverTable", 2, kTableFlags)) {
        LabelledValue("Position (m)", "%7.2f %7.2f %7.2f", sample.position.x(), sample.position.y(),
                      sample.position.z());
        LabelledValue("Roll / pitch (deg)", "%7.2f %7.2f", sample.chassis_roll * CH_RAD_TO_DEG,
                      sample.chassis_pitch * CH_RAD_TO_DEG);
        LabelledValue("Grade (deg)", "%8.2f", sample.grade * CH_RAD_TO_DEG);
        LabelledValue("Ground clearance (m)", "%8.3f", sample.ground_clearance);
        LabelledValue("Wheels in contact", "%8d / %d", sample.wheels_in_contact,
                      static_cast<int>(sample.wheels.size()));
        LabelledValue("Stability margin", "%8.3f", sample.stability_margin);
        LabelledValue("Path error (m)", "%8.3f", sample.path_error);
        LabelledValue("Total mass (kg)", "%8.2f", m_simulation->RoverModel().TotalMass());
        ImGui::EndTable();
    }

    if (sample.pivots.empty()) {
        ImGui::TextDisabled("No suspension pivots in this model.");
        return;
    }
    if (ImGui::BeginTable("PivotTable", 4, kGridFlags)) {
        ImGui::TableSetupColumn("Pivot");
        ImGui::TableSetupColumn("Angle");
        ImGui::TableSetupColumn("Rate");
        ImGui::TableSetupColumn("Reaction");
        ImGui::TableHeadersRow();

        for (const auto& pivot : sample.pivots) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(pivot.name.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%6.2f", pivot.angle * CH_RAD_TO_DEG);
            ImGui::TableNextColumn();
            ImGui::Text("%6.2f", pivot.velocity * CH_RAD_TO_DEG);
            ImGui::TableNextColumn();
            ImGui::Text("%7.1f", pivot.reaction_torque);
        }
        ImGui::EndTable();
    }
    ImGui::TextDisabled("angle and rate in deg and deg/s, reaction in Nm");
}

void SimGui::RenderDriveSection() {
    if (!ImGui::CollapsingHeader("Drives", ImGuiTreeNodeFlags_DefaultOpen)) return;

    const auto& sample = m_simulation->LatestSample();

    if (ImGui::BeginTable("DriveTable", 7, kGridFlags)) {
        ImGui::TableSetupColumn("Wheel");
        ImGui::TableSetupColumn("rad/s");
        ImGui::TableSetupColumn("Nm");
        ImGui::TableSetupColumn("A");
        ImGui::TableSetupColumn("degC");
        ImGui::TableSetupColumn("Slip");
        ImGui::TableSetupColumn("Sink");
        ImGui::TableHeadersRow();

        for (const auto& wheel : sample.wheels) {
            const auto& drive = wheel.drive;
            ImGui::TableNextRow();
            ImGui::TableNextColumn();

            // Colour the row by whichever limit is active, so a saturated drive is
            // visible without reading every number.
            if (drive.thermal_cutoff) {
                ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1), "%s", wheel.id.c_str());
            } else if (drive.current_saturated || drive.voltage_saturated) {
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1), "%s", wheel.id.c_str());
            } else {
                ImGui::TextUnformatted(wheel.id.c_str());
            }

            ImGui::TableNextColumn();
            ImGui::Text("%6.2f", wheel.speed);
            ImGui::TableNextColumn();
            ImGui::Text("%6.1f", drive.applied_torque);
            ImGui::TableNextColumn();
            ImGui::Text("%5.1f", drive.current_a);
            ImGui::TableNextColumn();
            ImGui::Text("%5.1f", drive.temperature_c);
            ImGui::TableNextColumn();
            ImGui::Text("%5.2f", wheel.longitudinal_slip);
            ImGui::TableNextColumn();
            ImGui::Text("%6.3f", wheel.sinkage);
        }
        ImGui::EndTable();
    }
    ImGui::TextDisabled("amber: current or voltage limited | red: thermal cutoff");
}

void SimGui::RenderEnergySection() {
    if (!ImGui::CollapsingHeader("Energy")) return;

    const auto& battery = m_simulation->LatestSample().battery;

    if (ImGui::BeginTable("EnergyTable", 2, kTableFlags)) {
        LabelledValue("Terminal (V)", "%8.2f", battery.terminal_v);
        LabelledValue("Open circuit (V)", "%8.2f", battery.open_circuit_v);
        LabelledValue("Current (A)", "%8.2f", battery.current_a);
        LabelledValue("Power (W)", "%8.1f", battery.power_w);
        LabelledValue("Energy (Wh)", "%8.3f", battery.cumulative_wh);
        LabelledValue("State of charge", "%8.3f", battery.soc);
        LabelledValue("Peak current (A)", "%8.2f", battery.peak_current_a);
        LabelledValue("Regen rejected (W)", "%8.1f", battery.regen_rejected_w);
        ImGui::EndTable();
    }
}

void SimGui::RenderTerrainSection(vsg::CommandBuffer& cb) {
    if (!ImGui::CollapsingHeader("Terrain", ImGuiTreeNodeFlags_DefaultOpen)) return;

    auto& terrain = m_simulation->TerrainModel();
    const auto& spec = m_simulation->Manifest().terrain;

    ImGui::TextWrapped("%s (v%d)", spec.name.c_str(), spec.version);

    auto* scm = terrain.Scm();
    if (!scm) {
        ImGui::TextDisabled("Rigid terrain - no deformation to visualise.");
        return;
    }

    if (ImGui::BeginCombo("False colour", kPlotOptions[m_plot_type].name)) {
        for (int i = 0; i < static_cast<int>(kPlotOptions.size()); i++) {
            if (ImGui::Selectable(kPlotOptions[i].name, i == m_plot_type)) {
                m_plot_type = i;
                const auto& option = kPlotOptions[i];
                scm->SetPlotType(option.type, option.min, option.max);
            }
        }
        ImGui::EndCombo();
    }

    if (ImGui::BeginCombo("Colormap", kColormaps[m_colormap].second)) {
        for (int i = 0; i < static_cast<int>(kColormaps.size()); i++) {
            if (ImGui::Selectable(kColormaps[i].second, i == m_colormap)) {
                m_colormap = i;
                scm->SetColormap(kColormaps[i].first);
            }
        }
        ImGui::EndCombo();
    }

    const auto& plot = kPlotOptions[m_plot_type];
    if (plot.type != SCMTerrain::PLOT_NONE) {
        ImGui::TextUnformatted(plot.legend);
        Colorbar(m_vsys->GetColormapTexture(scm->GetColormapType()), {plot.min, plot.max}, false,
                 300.0f * m_scale, cb.deviceID);
        ImGui::NewLine();  // Colorbar leaves the cursor mid-line
    }

    if (ImGui::Checkbox("Wireframe", &m_state->wireframe)) {
        scm->SetMeshWireframe(m_state->wireframe);
    }

    if (m_scm_vis && ImGui::Checkbox("Show active domains", &m_state->active_domains_visible)) {
        m_scm_vis->SetActiveBoxVisibility(m_state->active_domains_visible, -1);
    }
    ImGui::SameLine();
    HelpMarker("The boxes that bound where SCM ray casting is performed each step.");

    ImGui::TextDisabled("Bulldozing is %s by the manifest.",
                        spec.bulldozing ? "enabled" : "disabled");
}

void SimGui::RenderSolverSection() {
    if (!ImGui::CollapsingHeader("SCM solver")) return;

    auto* scm = m_simulation->TerrainModel().Scm();
    if (!scm) {
        ImGui::TextDisabled("No deformable terrain in this run.");
        return;
    }

    if (ImGui::BeginTable("CounterTable", 2, kTableFlags)) {
        LabelledValue("Ray casts", "%8d", scm->GetNumRayCasts());
        LabelledValue("Ray hits", "%8d", scm->GetNumRayHits());
        LabelledValue("Contact patches", "%8d", scm->GetNumContactPatches());
        LabelledValue("Eroded nodes", "%8d", scm->GetNumErosionNodes());
        LabelledValue("Rigid contacts", "%8d",
                      static_cast<int>(m_simulation->System().GetNumContacts()));
        ImGui::EndTable();
    }

    ImGui::TextUnformatted("Last step (ms)");
    if (ImGui::BeginTable("TimerTable", 2, kTableFlags)) {
        LabelledValue("Active domains", "%8.3f", 1e3 * scm->GetTimerActiveDomains());
        LabelledValue("Ray testing", "%8.3f", 1e3 * scm->GetTimerRayTesting());
        LabelledValue("Ray casting", "%8.3f", 1e3 * scm->GetTimerRayCasting());
        LabelledValue("Contact patches", "%8.3f", 1e3 * scm->GetTimerContactPatches());
        LabelledValue("Contact forces", "%8.3f", 1e3 * scm->GetTimerContactForces());
        LabelledValue("Bulldozing", "%8.3f", 1e3 * scm->GetTimerBulldozing());
        LabelledValue("Visualization", "%8.3f", 1e3 * scm->GetTimerVisUpdate());
        ImGui::EndTable();
    }
}

void SimGui::RenderCameraSection() {
    if (!ImGui::CollapsingHeader("Camera") || !m_camera) return;

    ImGui::Checkbox("Follow rover", &m_state->follow_rover);

    const auto& wheels = m_simulation->RoverModel().Wheels();
    const char* current =
        m_state->follow_target < 0 ? "chassis" : wheels[m_state->follow_target].id.c_str();
    if (ImGui::BeginCombo("Follow target", current)) {
        if (ImGui::Selectable("chassis", m_state->follow_target < 0)) {
            m_state->follow_target = -1;
        }
        for (int i = 0; i < static_cast<int>(wheels.size()); i++) {
            if (ImGui::Selectable(wheels[i].id.c_str(), m_state->follow_target == i)) {
                m_state->follow_target = i;
            }
        }
        ImGui::EndCombo();
    }

    float azimuth = static_cast<float>(m_camera->Azimuth() * CH_RAD_TO_DEG);
    float elevation = static_cast<float>(m_camera->Elevation() * CH_RAD_TO_DEG);
    float distance = static_cast<float>(m_camera->Distance());

    bool changed = ImGui::SliderFloat("Azimuth (deg)", &azimuth, -180.0f, 180.0f, "%.1f");
    changed |= ImGui::SliderFloat("Elevation (deg)", &elevation, -88.0f, 88.0f, "%.1f");
    changed |= ImGui::SliderFloat("Distance (m)", &distance, 0.5f, 60.0f, "%.2f");
    if (changed) m_camera->SetOrbit(azimuth * CH_DEG_TO_RAD, elevation * CH_DEG_TO_RAD, distance);

    if (ImGui::Button("Reset view")) m_camera->GoHome();

    ImGui::TextDisabled("left/right drag: orbit | middle: pan | scroll: zoom");
}

}  // namespace trickfire
