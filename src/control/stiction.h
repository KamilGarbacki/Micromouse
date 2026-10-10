// Static/kinetic friction switch with hysteresis (pure).
#pragma once

namespace micromouse {

/// Next "rolling" state for the filtered speed `speedTpp`: becomes true at
/// or above `enterTpp`, false again only below `exitTpp` (exitTpp <
/// enterTpp), so a single backlash tick cannot toggle it.
constexpr bool updateRolling(const bool rolling, const float speedTpp,
                             const float enterTpp, const float exitTpp) {
  return rolling ? speedTpp >= exitTpp : speedTpp >= enterTpp;
}

/// Stiction offset / wheel-floor base for the current state.
constexpr float stictionPwm(const bool rolling, const float staticPwm,
                            const float kineticPwm) {
  return rolling ? kineticPwm : staticPwm;
}

}  // namespace micromouse
