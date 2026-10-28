#pragma once

#include <string>

#include "simulation.hpp"

namespace trickfire {

/// Runtime state the viewer owns. None of it changes the physics: the manifest is the
/// only thing that decides what is simulated, and the viewer is a window onto it.
///
/// Pause, step and reset are the exceptions, and reset writes an event into the run's
/// log so a hand-reset run is never mistaken for a clean one.
struct ViewerState {
    bool paused = false;
    bool step_once = false;
    bool reset_requested = false;

    double target_render_fps = 60;
    bool follow_rover = true;
    int follow_target = -1;  // -1 = chassis, otherwise a wheel index

    bool wireframe = false;
    bool bulldozing_visible = true;
    bool active_domains_visible = false;
};

/// Open the window and drive `simulation` until the run terminates or the window closes.
/// The simulation's own Step() is what advances time, so the viewer and the headless
/// runner cannot diverge.
void RunViewer(Simulation& simulation);

/// Read TRICKFIRE_UI_SCALE, falling back to the given default if unset or invalid.
/// VSG renders into physical pixels and neither it nor Chrono looks up the display
/// scale, so the panels come out tiny on a HiDPI screen without this.
float ReadUiScale(float fallback = 1.0f);

}  // namespace trickfire
