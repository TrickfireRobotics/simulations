#include "vis_system.hpp"

using namespace chrono;

namespace trickfire {

SimVisualSystem::SimVisualSystem(CameraVerticalDir up)
    : m_up(up == CameraVerticalDir::Y ? vsg::dvec3(0, 1, 0) : vsg::dvec3(0, 0, 1)) {
    SetCameraVertical(up);
    m_camera_trackball = false;
}

void SimVisualSystem::Initialize() {
    // ChVisualSystem documents that a derived Initialize must run only once. The base
    // guards itself with m_initialized and returns early, so without this check a second
    // call would install a second OrbitCamera; both would write the shared LookAt every
    // frame and the stale one would win.
    if (m_initialized) return;

    size_t user_components = m_gui.size();

    ChVisualSystemVSG::Initialize();

    // Hide the GUI windows contributed by plugins. The SCM plugin's window caches its
    // colorbar legend at init, so it goes stale the moment the plot type is changed from
    // our own panel - which shows the same data, live.
    for (size_t i = user_components; i < m_gui.size(); i++) {
        if (m_gui[i] != m_base_gui && i != m_camera_gui) m_gui[i]->SetVisibility(false);
    }

    // m_lookAt is only built by the base Initialize, so the orbit state can be derived
    // from whatever AddCamera was given.
    m_orbit = OrbitCamera::create(m_vsg_camera, m_up);
    m_viewer->addEventHandler(m_orbit);
}

}  // namespace trickfire
