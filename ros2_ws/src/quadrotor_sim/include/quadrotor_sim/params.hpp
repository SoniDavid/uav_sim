#pragma once
#include <cmath>

namespace qsim {

constexpr double MASS      = 0.045;
constexpr double G         = 9.81;
constexpr double ARM       = 0.075;   // S/2, moment arm (m)

constexpr double IXX = 7.921e-5;
constexpr double IYY = 13.604e-5;
constexpr double IZZ = 7.317e-5;
constexpr double JR  = 1.4961e-7;

constexpr double KT = 2.64e-8;
constexpr double KQ = 5.4e-9;

constexpr double TM        = 0.01;
constexpr double OMEGA_MAX = 2640.0;

constexpr double DT_SIM            = 0.01;
constexpr double DT_PUB            = 0.02;
constexpr int    SIM_STEPS_PER_PUB = 2;

constexpr double T_HOVER = MASS * G;
constexpr double T_MAX   = 4.0 * KT * OMEGA_MAX * OMEGA_MAX;

// Computed at startup (std::sqrt is not constexpr in C++17)
inline const double OMEGA_HOVER = std::sqrt(T_HOVER / (4.0 * KT));

} // namespace qsim
