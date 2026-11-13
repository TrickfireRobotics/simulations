#include "orbit_camera.hpp"

#include <algorithm>
#include <cmath>

namespace trickfire {

OrbitCamera::OrbitCamera(vsg::ref_ptr<vsg::Camera> camera, const vsg::dvec3& up)
    : m_camera(camera), m_up(vsg::normalize(up)) {
    // Any reference direction not parallel to up gives a valid horizontal basis;
    // (m_east, m_north, m_up) comes out right-handed either way.
    vsg::dvec3 reference = (std::abs(m_up.z) < 0.9) ? vsg::dvec3(0, 0, 1) : vsg::dvec3(1, 0, 0);
    m_east = vsg::normalize(vsg::cross(reference, m_up));
    m_north = vsg::cross(m_up, m_east);

    SyncFromCamera();
}

void OrbitCamera::SyncFromCamera() {
    auto look_at = m_camera->viewMatrix.cast<vsg::LookAt>();
    if (!look_at) return;

    m_pivot = look_at->center;
    m_pan = vsg::dvec3(0, 0, 0);

    vsg::dvec3 offset = look_at->eye - look_at->center;
    m_distance = std::clamp(vsg::length(offset), min_distance, max_distance);

    double vertical = vsg::dot(offset, m_up);
    m_elevation = std::clamp(std::asin(vertical / m_distance), -max_elevation, max_elevation);
    m_azimuth = std::atan2(vsg::dot(offset, m_north), vsg::dot(offset, m_east));

    m_home_azimuth = m_azimuth;
    m_home_elevation = m_elevation;
    m_home_distance = m_distance;

    Apply();
}

void OrbitCamera::GoHome() {
    SetOrbit(m_home_azimuth, m_home_elevation, m_home_distance);
    ClearPan();
}

void OrbitCamera::SetOrbit(double azimuth, double elevation, double distance) {
    m_azimuth = azimuth;
    m_elevation = std::clamp(elevation, -max_elevation, max_elevation);
    m_distance = std::clamp(distance, min_distance, max_distance);
}

void OrbitCamera::Apply() {
    auto look_at = m_camera->viewMatrix.cast<vsg::LookAt>();
    if (!look_at) return;

    vsg::dvec3 horizontal = m_east * std::cos(m_azimuth) + m_north * std::sin(m_azimuth);
    vsg::dvec3 offset =
        (horizontal * std::cos(m_elevation) + m_up * std::sin(m_elevation)) * m_distance;

    look_at->center = m_pivot + m_pan;
    look_at->eye = look_at->center + offset;
    look_at->up = m_up;
}

void OrbitCamera::Zoom(double amount) {
    m_distance = std::clamp(m_distance * std::exp(-amount), min_distance, max_distance);
}

void OrbitCamera::apply(vsg::ButtonPressEvent& event) {
    if (event.handled) return;

    // X11 button numbering: 1 left, 2 middle, 3 right. Left and right both orbit;
    // zooming is on the scroll wheel, where a horizontal drag cannot silently do nothing.
    switch (event.button) {
        case 1:
        case 3:
            m_drag = Drag::ORBIT;
            break;
        case 2:
            m_drag = Drag::PAN;
            break;
        default:
            return;
    }

    m_last_x = event.x;
    m_last_y = event.y;
    event.handled = true;
}

void OrbitCamera::apply(vsg::ButtonReleaseEvent& event) { m_drag = Drag::NONE; }

void OrbitCamera::apply(vsg::MoveEvent& event) {
    int32_t dx = event.x - m_last_x;
    int32_t dy = event.y - m_last_y;
    m_last_x = event.x;
    m_last_y = event.y;

    if (m_drag == Drag::NONE || event.handled) return;

    switch (m_drag) {
        case Drag::ORBIT:
            m_azimuth -= dx * orbit_speed;
            m_elevation = std::clamp(m_elevation + dy * orbit_speed, -max_elevation, max_elevation);
            break;
        case Drag::PAN: {
            // Screen right and screen up, so the pivot tracks the cursor
            vsg::dvec3 right = m_east * -std::sin(m_azimuth) + m_north * std::cos(m_azimuth);
            vsg::dvec3 forward = -(m_east * std::cos(m_azimuth) + m_north * std::sin(m_azimuth)) *
                                     std::cos(m_elevation) -
                                 m_up * std::sin(m_elevation);
            vsg::dvec3 screen_up = vsg::cross(right, forward);
            double scale = m_distance * pan_speed;
            m_pan += right * (-dx * scale) + screen_up * (dy * scale);
            break;
        }
        case Drag::NONE:
            break;
    }

    event.handled = true;
}

void OrbitCamera::apply(vsg::ScrollWheelEvent& event) {
    if (event.handled) return;

    Zoom(event.delta.y * zoom_speed);
    event.handled = true;
}

void OrbitCamera::apply(vsg::FrameEvent& event) { Apply(); }

}  // namespace trickfire
