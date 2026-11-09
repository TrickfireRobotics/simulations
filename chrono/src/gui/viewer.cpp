#include "gui/viewer.hpp"

#include <cstdlib>
#include <string>

#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/visualization/ChScmVisualizationVSG.h"

#include "gui/sim_gui.hpp"
#include "gui/vis_system.hpp"

using namespace chrono;

namespace trickfire {

namespace {

vsg::dvec3 ToVsg(const ChVector3d& v) { return vsg::dvec3(v.x(), v.y(), v.z()); }

}  // namespace

float ReadUiScale(float fallback) {
    const char* env = std::getenv("TRICKFIRE_UI_SCALE");
    if (!env) return fallback;

    try {
        float scale = std::stof(env);
        if (scale > 0.1f && scale < 10.0f) return scale;
    } catch (const std::exception&) {
    }
    return fallback;
}

void RunViewer(Simulation& simulation) {
    const auto& manifest = simulation.Manifest();
    const bool z_up = manifest.simulation.up_axis == SimulationSpec::UpAxis::Z;

    // Chrono's vehicle world frame has to agree with the manifest, or the terrain plugin
    // draws its mesh in the wrong plane.
    if (z_up) {
        vehicle::ChWorldFrame::Set(ChMatrix33<>(1));
    } else {
        vehicle::ChWorldFrame::SetYUP();
    }

    simulation.Initialize();

    ViewerState state;
    const float ui_scale = ReadUiScale();

    auto vis = chrono_types::make_shared<SimVisualSystem>(z_up ? CameraVerticalDir::Z
                                                               : CameraVerticalDir::Y);
    vis->AttachSystem(&simulation.System());

    std::shared_ptr<vehicle::ChScmVisualizationVSG> scm_vis;
    if (auto* scm = simulation.TerrainModel().Scm()) {
        scm_vis = chrono_types::make_shared<vehicle::ChScmVisualizationVSG>(scm);
        vis->AttachPlugin(scm_vis);
        scm->SetMeshWireframe(state.wireframe);
    }

    auto gui = chrono_types::make_shared<SimGui>(&simulation, &state, scm_vis.get(), ui_scale);
    vis->AddGuiComponent(gui);

    const ChVector3d up = z_up ? ChVector3d(0, 0, 1) : ChVector3d(0, 1, 0);
    const ChVector3d start = simulation.LatestSample().position;

    vis->SetWindowTitle("TrickFire Robotics - " + manifest.rover.name + " / " +
                        manifest.scenario.name);
    vis->SetWindowSize(1600, 950);
    vis->SetWindowPosition(100, 100);
    vis->AddCamera(start + ChVector3d(-4, -4, 0) + up * 2.5, start);
    vis->SetCameraAngleDeg(40.0);
    vis->SetGuiFontSize(13.0f * ui_scale);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->SetLogo(TRICKFIRE_LOGO_PATH);
    vis->SetLogoHeight(56.0f * ui_scale);
    vis->Initialize();

    gui->SetCamera(vis->Camera());

    double render_fps = -1;

    while (vis->Run()) {
        if (state.reset_requested) {
            simulation.Reset();
            state.reset_requested = false;
        }

        if (state.target_render_fps != render_fps) {
            render_fps = state.target_render_fps;
            vis->SetTargetRenderFPS(render_fps);
        }

        if (state.follow_rover) {
            const auto& wheels = simulation.RoverModel().Wheels();
            const ChVector3d pivot =
                state.follow_target >= 0 && state.follow_target < static_cast<int>(wheels.size())
                    ? wheels[state.follow_target].body->GetPos()
                    : simulation.RoverModel().Chassis()->GetFrameRefToAbs().GetPos();
            vis->Camera()->SetPivot(ToVsg(pivot));
        }

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // The window stays open after the run terminates so the final state can be
        // inspected; Step() returns false and does nothing once that happens.
        const bool advance = !state.paused || state.step_once;
        state.step_once = false;
        if (advance) simulation.Step();
    }

    // The window may have been closed mid-run, so the summary is written here rather than
    // only on termination.
    simulation.Finish();
}

}  // namespace trickfire
