#pragma once

#include <cmath>
#include <cstdio>
#include <string>

/// Assertions for the test binaries.
///
/// Hand-rolled rather than GoogleTest: Chrono bundles gtest but does not export it, and
/// these tests only ever need "is this number within tolerance". Each binary prints one
/// line per check and exits non-zero if any failed, which is all CTest needs.
namespace check {

inline int failures = 0;
inline int total = 0;

inline void Report(bool ok, const std::string& what, const std::string& detail = "") {
    total++;
    if (!ok) failures++;
    std::printf("%s  %s%s%s\n", ok ? "ok  " : "FAIL", what.c_str(), detail.empty() ? "" : "  -  ",
                detail.c_str());
}

inline void True(bool value, const std::string& what) { Report(value, what); }

inline void Near(double actual, double expected, double tolerance, const std::string& what) {
    const double error = std::abs(actual - expected);
    char detail[160];
    std::snprintf(detail, sizeof(detail), "got %.6g, expected %.6g +/- %.3g (error %.3g)", actual,
                  expected, tolerance, error);
    Report(std::isfinite(actual) && error <= tolerance, what, detail);
}

/// Relative tolerance, for quantities whose scale is not known in advance.
inline void Close(double actual, double expected, double fraction, const std::string& what) {
    const double tolerance = std::abs(expected) * fraction;
    Near(actual, expected, tolerance > 0 ? tolerance : fraction, what);
}

inline void Less(double actual, double limit, const std::string& what) {
    char detail[120];
    std::snprintf(detail, sizeof(detail), "got %.6g, limit %.6g", actual, limit);
    Report(std::isfinite(actual) && actual < limit, what, detail);
}

inline void Greater(double actual, double limit, const std::string& what) {
    char detail[120];
    std::snprintf(detail, sizeof(detail), "got %.6g, limit %.6g", actual, limit);
    Report(std::isfinite(actual) && actual > limit, what, detail);
}

inline int Finish(const char* suite) {
    std::printf("\n%s: %d/%d passed\n", suite, total - failures, total);
    return failures == 0 ? 0 : 1;
}

}  // namespace check
