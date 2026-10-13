// Run-time calibration: the motor model of each wheel (self-calibration) and
// the values learned while driving. Stored in EEPROM as two independent POD
// records, each with its own magic, version, key and CRC-16, so that
//  - a geometry change (wheel, cell, wheelbase) keeps the motor model, which
//    only depends on the ticks per wheel revolution, and
//  - a brown-out while saving one record cannot corrupt the other.
//
// Pure (no hardware): the app moves the bytes through hal/eeprom.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "config/config.h"
#include "control/learning.h"

namespace micromouse {

/// Open-loop motor model of one wheel:
///   rolling: PWM = ffOffsetPwm + ffSlopePwmPerTpp * speed
///   at rest: PWM = staticPwm + ffSlopePwmPerTpp * speed
/// kineticPwm is the lowest PWM that keeps the wheel turning (wheel floor).
struct WheelCalibration {
  float staticPwm;         ///< [PWM] breakaway from rest
  float kineticPwm;        ///< [PWM] keeps the wheel rolling (<= staticPwm)
  float ffOffsetPwm;       ///< [PWM] intercept of the feed-forward line
  float ffSlopePwmPerTpp;  ///< [PWM per tpp] slope of the feed-forward line
};

enum class Wheel : uint8_t { kLeft = 0, kRight = 1 };
enum class MoveKind : uint8_t { kForward = 0, kTurn = 1 };

/// Motor record (written by the 'c' self-calibration). No interior padding
/// on the AVR or the host (the CRC covers raw bytes).
struct MotorRecord {
  uint16_t magic;             ///< calibration::kMotorMagic
  uint8_t version;            ///< calibration::kMotorVersion
  uint8_t reserved;           ///< 0
  uint16_t ticksPerWheelRev;  ///< key: the FF slope is per tick
  uint16_t reserved2;         ///< 0
  WheelCalibration wheel[2];  ///< lifted self-calibration, [Wheel]
  uint16_t crc;               ///< CRC-16 of all bytes before this field
};

/// Learned-geometry record (written by the online learning).
struct LearnedRecord {
  uint16_t magic;   ///< calibration::kLearnedMagic
  uint8_t version;  ///< calibration::kLearnedVersion
  uint8_t flags;    ///< joint pitch estimator state (kLearnedFlagJoint...)
  /// Key: the DEFAULT tick counts (config/geometry.h). A geometry change
  /// invalidates this record (not the motor record).
  uint16_t geometryTicksPerCell;
  uint16_t geometryTicksPer90;
  float mmPerTick;         ///< learned wheel scale [mm per tick]
  /// Learned cell pitch [mm]: the joint solution if kLearnedFlagJoint,
  /// else the provisional estimate (delta = 0 assumed).
  float cellPitchMm;
  float wallOffsetMm;      ///< joint front-offset error delta [mm]
  float ticksPer90;        ///< learned [ticks per wheel]
  float brakeLeadForward;  ///< learned [ticks]
  float brakeLeadTurn;     ///< learned [ticks]
  learning::PitchSums pitchSums;  ///< joint (P, delta) normal equations
  uint16_t scaleSamples;   ///< accepted wheel-scale samples
  uint16_t pitchSamples;   ///< accepted provisional pitch samples
  uint16_t turnSamples;    ///< accepted ticks-per-90 samples
  uint16_t crc;            ///< CRC-16 of all bytes before this field
};

/// Everything the robot runs with (RAM copy of both records).
struct Calibration {
  MotorRecord motor;
  LearnedRecord learned;
  bool motorsCalibrated;  ///< the motor record is valid
};

namespace calibration {

constexpr uint16_t kMotorMagic = 0x4D4D;    // "MM"
constexpr uint16_t kLearnedMagic = 0x4C4D;  // "ML"
constexpr uint8_t kMotorVersion = 1;
// v1 was the single combined record, v2 had no front-offset error, v3 put
// the joint pitch solution in force without a minimum row count (its
// flags byte had no rows field: joint bit set with rows = 0).
constexpr uint8_t kLearnedVersion = 4;
/// The flags byte is the packed state of the joint pitch estimator
/// (learning::kPitchJointBit: joint (P, delta) in force; rows count).
constexpr uint8_t kLearnedFlagJoint = learning::kPitchJointBit;
/// Magic of erased EEPROM.
constexpr uint16_t kErasedMagic = 0xFFFF;
constexpr size_t kMotorCrcOffset = offsetof(MotorRecord, crc);
constexpr size_t kLearnedCrcOffset = offsetof(LearnedRecord, crc);
static_assert(kMotorCrcOffset == 40, "unexpected padding in MotorRecord");
static_assert(kLearnedCrcOffset == 58, "unexpected padding in LearnedRecord");
#if defined(__AVR__)
static_assert(sizeof(MotorRecord) == 42, "MotorRecord layout changed");
static_assert(sizeof(LearnedRecord) == 60, "LearnedRecord layout changed");
#endif
static_assert(config::kMotorRecordAddress + sizeof(MotorRecord) <=
                      config::kLearnedRecordAddress &&
                  config::kLearnedRecordAddress + sizeof(LearnedRecord) <=
                      1024,
              "EEPROM records overlap or exceed the 1 KB EEPROM");

enum class Validity : uint8_t {
  kValid = 0,
  kEmpty,     ///< erased EEPROM (magic 0xFFFF)
  kBadMagic,  ///< another / older layout
  kBadVersion,
  kBadCrc,
  kGeometryChanged,
  kOutOfRange,
};

/// Default records (sealed): the configured motor defaults and the
/// geometry-derived values with no samples.
MotorRecord defaultMotor();
LearnedRecord defaultLearned();
/// Both defaults, motorsCalibrated = false.
Calibration defaults();

uint16_t computeCrc(const MotorRecord& record);
uint16_t computeCrc(const LearnedRecord& record);

/// Set magic, version, key and the CRC.
void seal(MotorRecord& record);
void seal(LearnedRecord& record);

/// Magic, version, CRC, key and that every field is in its plausible range
/// (NaN fails).
Validity validate(const MotorRecord& record);
Validity validate(const LearnedRecord& record);

/// Seed samples are only kept in RAM: a sample count below the minimum
/// (an interrupted seed) restarts the seed after a reboot.
void restartIncompleteSeeds(LearnedRecord& record);

/// Effective values: the learned estimate once it has its minimum number of
/// samples (pitch: or a joint solution is in force), the geometry default
/// before.
float mmPerTick(const Calibration& cal);
float cellPitchMm(const Calibration& cal);
/// The front-offset error delta in force (0 until a joint solution).
float wallOffsetMm(const Calibration& cal);
bool pitchJoint(const Calibration& cal);
/// round(cellPitchMm / mmPerTick).
int32_t ticksPerCell(const Calibration& cal);
int32_t ticksPer90(const Calibration& cal);

/// Lifted motor model plus floor load: each intercept (static, kinetic,
/// FF offset) becomes value * scale + offsetPwm, the slope is unchanged.
WheelCalibration applyLoad(const WheelCalibration& lifted, float scale,
                           float offsetPwm);

/// The motor model the move loop uses for `wheel` in a move of `kind`: the
/// self-calibration with the configured floor load of that kind if
/// motorsCalibrated, else the configured defaults of that kind.
WheelCalibration wheelModel(const Calibration& cal, Wheel wheel,
                            MoveKind kind);

}  // namespace calibration
}  // namespace micromouse
