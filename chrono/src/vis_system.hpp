#pragma once

#include "chrono_vsg/ChVisualSystemVSG.h"

#include "orbit_camera.hpp"

namespace trickfire {

/// Chrono's VSG visual system with a turntable camera instead of a free trackball.
///
/// The stock system installs a vsg::Trackball, which rolls the view freely, and gates it
/// behind the protected m_camera_trackball flag - hence the subclass. OrbitCamera takes
/// its place and keeps the world up vector fixed.
class SimVisualSystem : public chrono::vsg3d::ChVisualSystemVSG {
  public:
    explicit SimVisualSystem(chrono::CameraVerticalDir up);

    void Initialize() override;

    OrbitCamera* Camera() const { return m_orbit; }

  private:
    vsg::dvec3 m_up;
    vsg::ref_ptr<OrbitCamera> m_orbit;
};

}  // namespace trickfire
