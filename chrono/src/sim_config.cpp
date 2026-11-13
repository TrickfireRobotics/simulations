#include "sim_config.hpp"

#include <cstdlib>
#include <string>

namespace trickfire {

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

}  // namespace trickfire
