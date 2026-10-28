#pragma once

#include <vsg/all.h>

namespace trickfire {

/// Turntable camera handler, replacing VSG's free trackball.
///
/// The world up vector is fixed, so horizontal drags yaw around it and vertical drags
/// pitch between the poles. The horizon never tilts and the view never rolls.
///
/// The camera orbits `pivot + pan`. Feeding the pivot a moving body each frame turns it
/// into a follow camera; panning offsets the view from that body without fighting it.
class OrbitCamera : public vsg::Inherit<vsg::Visitor, OrbitCamera> {
  public:
    OrbitCamera(vsg::ref_ptr<vsg::Camera> camera, const vsg::dvec3& up);

    /// Adopt the camera's current eye/centre as the orbit state, and as the home view.
    void SyncFromCamera();

    /// Return to the orbit angles and distance captured by SyncFromCamera.
    void GoHome();

    void SetPivot(const vsg::dvec3& pivot) { m_pivot = pivot; }
    void SetOrbit(double azimuth, double elevation, double distance);

    double Azimuth() const { return m_azimuth; }
    double Elevation() const { return m_elevation; }
    double Distance() const { return m_distance; }
    vsg::dvec3 Pan() const { return m_pan; }

    void ClearPan() { m_pan = vsg::dvec3(0, 0, 0); }

    /// Write the orbit state into the camera's LookAt. Called once per frame.
    void Apply();

    void apply(vsg::ButtonPressEvent& event) override;
    void apply(vsg::ButtonReleaseEvent& event) override;
    void apply(vsg::MoveEvent& event) override;
    void apply(vsg::ScrollWheelEvent& event) override;
    void apply(vsg::FrameEvent& event) override;

    // Radians per pixel of drag.
    double orbit_speed = 0.006;
    // Fraction of the orbit distance panned per pixel of drag.
    double pan_speed = 0.0015;
    // Multiplicative zoom per scroll notch.
    double zoom_speed = 0.1;
    // Keeps the eye off the poles, where the up vector becomes degenerate.
    double max_elevation = vsg::radians(88.0);
    double min_distance = 0.2;
    double max_distance = 500.0;

  private:
    enum class Drag { NONE, ORBIT, PAN };

    void Zoom(double amount);

    vsg::ref_ptr<vsg::Camera> m_camera;
    vsg::dvec3 m_up;
    // Orthonormal basis of the horizontal plane; azimuth is measured from m_east.
    vsg::dvec3 m_east;
    vsg::dvec3 m_north;

    vsg::dvec3 m_pivot = vsg::dvec3(0, 0, 0);
    vsg::dvec3 m_pan = vsg::dvec3(0, 0, 0);
    double m_azimuth = 0;
    double m_elevation = 0;
    double m_distance = 1;

    double m_home_azimuth = 0;
    double m_home_elevation = 0;
    double m_home_distance = 1;

    Drag m_drag = Drag::NONE;
    int32_t m_last_x = 0;
    int32_t m_last_y = 0;
};

}  // namespace trickfire
