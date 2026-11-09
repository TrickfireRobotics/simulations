#pragma once

#include "chrono_vsg/ChGuiComponentVSG.h"

#include "gui/orbit_camera.hpp"
#include "gui/viewer.hpp"
#include "simulation.hpp"

namespace chrono {
namespace vehicle {
class ChScmVisualizationVSG;
}
}  // namespace chrono

namespace trickfire {

/// The TrickFire control panel: run controls, rover and drive telemetry, terrain
/// rendering options and SCM solver counters.
///
/// Everything here reads from Simulation's latest sample, which is the same data the
/// telemetry file gets. Chrono's own "Simulation" window already covers model time,
/// real-time factor and body counts, so none of that is repeated.
class SimGui : public chrono::vsg3d::ChGuiComponentVSG {
  public:
    SimGui(Simulation* simulation, ViewerState* state,
           chrono::vehicle::ChScmVisualizationVSG* scm_vis, float ui_scale);

    void SetCamera(OrbitCamera* camera) { m_camera = camera; }

    void render(vsg::CommandBuffer& cb) override;

  private:
    void RenderRunSection();
    void RenderRoverSection();
    void RenderDriveSection();
    void RenderEnergySection();
    void RenderTerrainSection(vsg::CommandBuffer& cb);
    void RenderSolverSection();
    void RenderCameraSection();

    Simulation* m_simulation;
    ViewerState* m_state;
    chrono::vehicle::ChScmVisualizationVSG* m_scm_vis;
    OrbitCamera* m_camera = nullptr;
    float m_scale;

    int m_plot_type = 0;
    int m_colormap = 0;
};

}  // namespace trickfire
