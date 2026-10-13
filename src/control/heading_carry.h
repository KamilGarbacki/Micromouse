// Heading-error carry between moves (pure arithmetic of the move loop).
//
// The carry is the residual heading error in "left minus right" encoder
// ticks (+ = robot rotated clockwise relative to the grid). A straight move
// starts with the carry as its heading setpoint offset; a turn absorbs it
// into its target. After every move the carry is updated from the final
// encoder counts - also after a settle timeout, because the move has already
// acted on the old carry (keeping it would correct a phantom error).
#pragma once

#include <stdint.h>

#include "util/math_utils.h"

namespace micromouse::heading {

/// Carry after a straight move driven in direction `dir` (+1 forward, -1
/// backward) that ended with `left` / `right` (direction-blind) ticks.
/// Forward, the left wheel running further turns the robot clockwise; in
/// reverse the same count difference turns it COUNTER-clockwise, so the
/// difference enters with the commanded sign.
constexpr int32_t carryAfterStraight(const int32_t carry, const int8_t dir,
                                     const int32_t left, const int32_t right,
                                     const int32_t limit) {
  return util::clamp<int32_t>(carry + dir * (left - right), -limit, limit);
}

/// Carry after a forward straight move that ended with `left` / `right`.
constexpr int32_t carryAfterForward(const int32_t carry, const int32_t left,
                                    const int32_t right,
                                    const int32_t limit) {
  return carryAfterStraight(carry, 1, left, right, limit);
}

/// Target ticks per wheel of a spin with nominal length `nominal`, in
/// direction `sign` (+1 clockwise, -1 counter-clockwise). A robot already
/// rotated clockwise (carry > 0) turns less clockwise / more
/// counter-clockwise. Clamped to [nominal / 2, 1.5 * nominal].
constexpr int32_t turnTarget(const int32_t nominal, const int8_t sign,
                             const int32_t carry) {
  return util::clamp<int32_t>(nominal - sign * carry / 2, nominal / 2,
                              nominal + nominal / 2);
}

/// Carry after a spin, before the limit: in a spin both (direction-blind)
/// counters count up and the heading changes by sign * (L + R)
/// left-minus-right ticks, so new = old + actual rotation - intended
/// rotation (2 * nominal). If this exceeds the limit, the carry no longer
/// describes the encoders' belief (the turn learning skips such a turn).
constexpr int32_t carryAfterTurnUnclamped(const int32_t carry,
                                          const int8_t sign,
                                          const int32_t left,
                                          const int32_t right,
                                          const int32_t nominal) {
  return carry + sign * (left + right - 2 * nominal);
}

/// Carry after a spin (carryAfterTurnUnclamped, clamped to +-limit).
constexpr int32_t carryAfterTurn(const int32_t carry, const int8_t sign,
                                 const int32_t left, const int32_t right,
                                 const int32_t nominal, const int32_t limit) {
  return util::clamp<int32_t>(
      carryAfterTurnUnclamped(carry, sign, left, right, nominal), -limit,
      limit);
}

}  // namespace micromouse::heading
