#include "chrono/core/ChDataPath.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/visualization/ChScmVisualizationVSG.h"

#include "scm_scene.hpp"
#include "sim_config.hpp"
#include "sim_gui.hpp"
#include "vis_system.hpp"

using namespace chrono;
using namespace trickfire;

namespace {

vsg::dvec3 ToVsg(const ChVector3d& v) { return vsg::dvec3(v.x(), v.y(), v.z()); }

}  // namespace

int main() {
    SetChronoDataPath(CHRONO_DATA_PATH);
    vehicle::SetVehicleDataPath(CHRONO_VEHICLE_DATA_PATH);
    vehicle::ChWorldFrame::SetYUP();

    SimConfig config;
    config.ui_scale = ReadUiScale(config.ui_scale);

    SimState state;
    ScmScene scene(config);

    scene.Terrain().EnableBulldozing(state.bulldozing);
    scene.Terrain().SetMeshWireframe(state.wireframe);

    auto scm_vis = chrono_types::make_shared<vehicle::ChScmVisualizationVSG>(&scene.Terrain());
    auto gui = chrono_types::make_shared<SimGui>(&state, &scene, scm_vis.get(), config.ui_scale);

    auto vis = chrono_types::make_shared<SimVisualSystem>(CameraVerticalDir::Y);
    vis->AttachSystem(&scene.System());
    vis->AttachPlugin(scm_vis);
    vis->AddGuiComponent(gui);
    vis->SetWindowTitle("TrickFire Robotics - SCM deformable terrain");
    vis->SetWindowSize(config.window_width, config.window_height);
    vis->SetWindowPosition(100, 100);
    vis->AddCamera(config.camera_eye, config.camera_target);
    vis->SetCameraAngleDeg(40.0);
    vis->SetGuiFontSize(13.0f * config.ui_scale);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->SetLogo(TRICKFIRE_LOGO_PATH);
    vis->SetLogoHeight(56.0f * config.ui_scale);
    vis->Initialize();

    gui->SetCamera(vis->Camera());

    double render_fps = -1;

    while (vis->Run()) {
        if (state.reset_requested) {
            scene.Reset();
            state.reset_requested = false;
        }

        if (state.target_render_fps != render_fps) {
            render_fps = state.target_render_fps;
            vis->SetTargetRenderFPS(render_fps);
        }

        if (state.follow_wheel) vis->Camera()->SetPivot(ToVsg(scene.Wheel()->GetPos()));

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        bool advance = !state.paused || state.step_once;
        state.step_once = false;

        if (advance) {
            scene.SetWheelSpeed(state.wheel_speed);
            scene.AdvanceTo(state.step_size);
        }
    }

    return 0;
}
