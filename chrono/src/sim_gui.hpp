#pragma once

#include <vector>

#include "chrono_vsg/ChGuiComponentVSG.h"

#include "orbit_camera.hpp"
#include "scm_scene.hpp"
#include "sim_config.hpp"
#include "wheel_catalog.hpp"

namespace chrono {
namespace vehicle {
class ChScmVisualizationVSG;
}
}  // namespace chrono

namespace trickfire {

/// The TrickFire control panel: run controls, wheel telemetry, terrain rendering options
/// and SCM solver counters. Chrono's own "Simulation" window already covers model time,
/// real-time factor, FPS and body counts, so none of that is repeated here.
class SimGui : public chrono::vsg3d::ChGuiComponentVSG {
  public:
    SimGui(SimState* state, ScmScene* scene, chrono::vehicle::ChScmVisualizationVSG* scm_vis,
           float ui_scale, std::vector<WheelOption> wheel_catalog, int current_wheel_index);

    void SetCamera(OrbitCamera* camera) { m_camera = camera; }

    void render(vsg::CommandBuffer& cb) override;

  private:
    void RenderRunSection();
    void RenderWheelSection();
    void RenderTerrainSection(vsg::CommandBuffer& cb);
    void RenderSolverSection();
    void RenderCameraSection();

    SimState* m_state;
    ScmScene* m_scene;
    chrono::vehicle::ChScmVisualizationVSG* m_scm_vis;
    OrbitCamera* m_camera = nullptr;
    float m_scale;

    int m_plot_type;
    int m_colormap;

    std::vector<WheelOption> m_wheel_catalog;
    int m_wheel_selection;
};

}  // namespace trickfire
