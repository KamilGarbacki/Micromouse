# Micromouse
In this project I designed, built and programmed my own Micromouse.
This project is being carried out as part of the SKN Creative AGH science club.

## What is a micromouse
A Micromouse is a small autonomous robot designed to navigate through a maze, with the goal of finding the optimal route to reach the center.
The mouse needs to keep track of where it is, discover walls as it explores, map out the maze and detect when it has reached the goal.
Having reached the goal, the mouse will typically perform additional searches of the maze until it has found an optimal route from the start to the finish.
Once the optimal route has been found, the mouse will traverse that route in the shortest achievable time.

## Project Developed with:
- PlatformIO
- C++ 17
- Arduino UNO (ATmega328P: 2 KB SRAM, 32 KB flash)
- Ultrasonic Sensors US-015 (x3: left, front, right)
- Micro DC Motors with Encoder-SJ02 SKU FIT0458 (single-channel hall encoders)
- Quad Motor Driver Shield for Arduino SKU DRI0039 (TB6612FNG)

## How does it work?
The robot knows only the maze size, the cell size, its start cell and heading, and the goal cell.
In every cell it stops, maps the walls around it with the ultrasonic sensors, recomputes a flood fill
(unknown walls are treated as open), and drives one cell towards the neighbour closest to the goal.
Straight moves and spins in place run a 100 Hz closed loop: a speed profile, feed-forward plus a PI on the
average wheel speed, and a heading PID on the left/right encoder difference whose residual is carried into the next move.

## Architecture

```
src/
  main.cpp              setup()/loop() -> app::RobotApp (nothing else)
  config/               the ONLY place for numbers
    pins.h              pin map (+ DRI0039 M1/M4 warning)
    geometry.h          wheel, encoder, gear, wheelbase, cell -> ticks per cell / per 90 deg
    tuning.h            every tunable with units and calibration step
    maze_config.h       maze size, start pose, goal, static capacity
    features.h          feature flags, log level
    config.h            umbrella + static_asserts on all of the above
  util/                 header-only, dependency-free
    status.h            Status result codes (printed by FAULT)
    ring_buffer.h       fixed-capacity FIFO
    median.h, math_utils.h
  hal/                  the only layer that talks to the Arduino core
    time, gpio (PWM + Timer2 fast-PWM fix), encoders (ISRs, atomic counters), log (Serial)
  drivers/              Motor, Encoder, UltrasonicSensor (NewPing hidden in the .cc)
  control/              Pid, MotionProfile, WheelMixer, WallCentering,
                        heading_carry.h, stiction.h (pure)
                        MotionController (the move loop; uses hal + drivers)
  navigation/           pose.h (Cell, Direction, heading math), Maze (sticky walls),
                        FloodFill, Explorer (per-step decisions), maze_render (pure)
  app/                  RobotApp state machine, wall sensing, Serial maze printer
test/                   Unity host tests for the pure modules
```

Layers, bottom to top. Each one may only use the layers below it:

| Layer        | Responsibility                                            | Arduino.h?                      |
|--------------|-----------------------------------------------------------|---------------------------------|
| `config`     | constants, derived values, compile-time checks            | no                              |
| `util`       | status codes, containers, small math                      | no                              |
| `hal`        | registers, ISRs, time, Serial                             | yes (the only layer)            |
| `drivers`    | one class per device                                      | only `ultrasonic_sensor.cc` (NewPing) |
| `control`    | PID, profile, mixer, centering; the move loop             | no (`motion_controller.cc` uses `avr/pgmspace.h`) |
| `navigation` | grid math, wall map, flood fill                           | no                              |
| `app`        | state machine, sensing policy, reporting                  | no                              |

`navigation/pose.h` is a dependency-free vocabulary header, so `config/maze_config.h` can use `Cell`/`Direction`.
Everything under `config`, `util`, `navigation` and the pure `control` modules builds on the host (see Tests).

### State machine (`app/robot_app.*`)

```
INIT -> SENSE -> PLAN -> MOVE -> SENSE -> ... -> GOAL_REACHED -> IDLE
            \        \       \
             +--------+-------+--> FAULT   (motors braked, error reported on Serial)
```

- **INIT**: motors braked, encoders and sensors set up, maze storage reset.
- **SENSE**: if the robot is in the goal cell, go to GOAL_REACHED. Otherwise ping left/front/right and hand the observations to the `Explorer` (see "Wall map policy"). A failed sensor -> FAULT.
- **PLAN**: flood fill, then pick a direction. Ties are broken in favour of the current heading, then a 90 deg turn over a 180. No path, or more than `kExplorationBudgetFactor` x cells driven -> FAULT.
- **MOVE**: turn if needed and drive one cell. Stall or timeout -> FAULT, and the pose is not updated.
- **GOAL_REACHED**: stop, report, print the discovered maze. Then the `Mission` decides what comes next (currently: IDLE).
- **FAULT**: the motors stay braked and `FAULT code=<n> (<NAME>) in <STATE> at (<row>,<col>) facing <U|R|D|L>` is repeated every 5 s. Codes are listed in `util/status.h`. The LED is not used because D13 is the right sensor's trigger.

All decisions of SENSE/PLAN/MOVE (map update, sensor plausibility, planning, pose, budget) live in the
pure `navigation/explorer.*`, which the host tests drive through complete simulated runs; `RobotApp` only
sequences them with the hardware.

FAULT codes (`util/status.h`): 1 INVALID_ARGUMENT, 2 OUT_OF_BOUNDS, 3 NO_PATH, 4 STALL, 5 MOVE_TIMEOUT,
6 NOT_INITIALIZED, 7 QUEUE_OVERFLOW, 8 SENSOR_FAILURE, 9 EXPLORATION_LIMIT.

Extension point: `Mission` in `robot_app.h`. A *return to start* is `explorer_.setGoal(start)`, then SENSE again.
A *speed run* is PLAN on known walls only, with faster `MoveParams`. Both would be dispatched in `onGoalReached()`.

### Conventions
- Coordinates: `Cell{row, col}`, with row 0 at the top and col 0 at the left. `Direction::kUp` means row - 1 and `kRight` means col + 1. Config uses the same convention: start (4,0) facing Up is bottom-left, goal (0,3) is top-right.
- Naming: types `PascalCase`, functions and variables `camelCase`, members `camelCase_`, constants and enumerators `kPascalCase`, macros `MM_UPPER_CASE` (log level and feature switches only), files `snake_case`.
- Formatting follows `.clang-format` (Google).
- No heap. There is no `new`, `malloc`, `String` or container library; all storage is static, sized from `config/maze_config.h`.
- Constant strings are wrapped in `F()`. The constant parameter tables of the move loop live in `PROGMEM`.
- Errors are returned as `Status`, never silently ignored. A failed move, a missing path or an internal error ends in FAULT.

## Build and test

```
pio run -e uno                  # build (warnings enabled for project sources: -Wall -Wextra)
pio run -e uno -t upload        # flash
pio device monitor -b 115200    # serial log
pio test -e native              # host unit tests (Unity)
```

The native tests need `gcc`/`g++` on `PATH`. On this Windows machine the CLion-bundled MinGW works:

```
PATH="$LOCALAPPDATA/Programs/CLion/bin/mingw/bin:$PATH" pio test -e native
```

Test suites:
- `test_navigation`: heading math, walls, sticky walls, flood fill, tie-break, maze rendering.
- `test_explorer`: end-to-end SENSE -> PLAN -> MOVE simulation on known mazes (open, dead end, serpentine, enclosed goal -> NO_PATH), flickering wall readings, exploration budget, sensor failure.
- `test_pid`: including anti-windup.
- `test_motion_profile`: continuity, creep before the stop point, short moves.
- `test_control_utils`: wheel mixer, wall centering, heading carry, stiction hysteresis, median, ring buffer.

Compile-time options (add to `build_flags` in `[env:uno]`):

| Flag                    | Default | Meaning                                                    |
|-------------------------|---------|------------------------------------------------------------|
| `-D MM_LOG_LEVEL=n`     | 2       | 0 none, 1 errors, 2 info, 3 debug (calibration output)     |
| `-D MM_WALL_CENTERING=1`| 0       | side-wall centering during straight moves                  |

Memory (`pio run -e uno`, log level 2). The RAM figure is static RAM; the rest is left for the stack:

|          | RAM (static)                          | Flash   |
|----------|---------------------------------------|---------|
| phase 1  | 450 B + heap (maze, sensors, queue)   | 14920 B |
| phase 2  | 702 B, no heap                        | 14188 B |

(64 B of the phase-2 RAM are the per-side contradiction counters of the sticky wall map.)

## Sensing
- **Mapping** happens only while the robot is stopped. Each sensor is pinged `kSonarSamples` (3) times, round-robin, with `kInterPingGapMs` (30 ms) of silence between any two pings, so the sensors do not hear each other's echoes. The median decides the wall (`< kWallThresholdCm`, 20 cm). "No echo" counts as far away, so one dropout cannot fake a wall. NewPing's range is clamped to 2 cells (44 cm), which bounds every ping to about 3 ms.
- **Wall centering** (optional, `MM_WALL_CENTERING=1`, default off): during straight moves a side sensor is pinged every `kCenteringPingPeriods` (2) control periods, alternating left and right: L, -, R, -, so there are 20 ms between any two pings.
  - The ping runs after the motor outputs are written. Normally it blocks for at most ~1.7 ms (20 cm range). A dead sensor can block ~8.2 ms (NewPing's 5.8 ms start timeout plus the echo windows); a static_assert keeps that under one control period.
  - The echo time is used in microseconds, which gives sub-cm resolution.
  - A side is used only if the maze map says a wall is there: in the current cell for the first half of the move, in the next cell for the second half. The reading must also be within +-5 cm of nominal.
  - The offset from nominal, times the gain, is low-pass filtered (`kCenteringFilter`), clamped to +-40 ticks and added to the heading error. See calibration step 10.

### Wall map policy
- An unknown side becomes known with its first observation. A known side flips only after `kWallFlipObservations` (2) consecutive contradicting observations; a confirming one resets the count. The count is shared by both cells of a wall. Readings that flicker around the threshold therefore cannot make the map oscillate. Setting it to 1 gives "latest reading wins".
- A sensor that gets no echo on any of its samples while the map already has a wall right beside it (e.g. the outer boundary) is implausible. That observation is not applied to the map, so a dead sensor cannot erase walls. After `kSensorFailurePasses` (3) such passes in a row -> FAULT SENSOR_FAILURE. Any echo resets the count.
  - Plain "no echo" facing a side that is not known to be walled is normal (an open corridor, especially for the front sensor), so it is neutral.
  - Limitation: a dead sensor that never faces a known wall is not detected here. It then reports "open", and the robot will stall against the real wall (FAULT STALL).
  - Rare false alarm: a wrongly mapped wall in front of a > 44 cm open corridor looks the same as a dead sensor.
- Exploration is bounded: more than `kExplorationBudgetFactor` (4) x rows x cols cells driven -> FAULT EXPLORATION_LIMIT. This backstops any livelock the sticky map does not prevent.

## Calibration

Build with `-D MM_LOG_LEVEL=3` and open the monitor at 115200 baud. All constants are in `src/config/tuning.h`
(geometry in `geometry.h`). Calibrate in this order:

1. **staticPwm** (`forward::kStaticPwm`, `turn::kStaticPwm`): lift the robot and command each wheel with increasing PWM. Note the lowest PWM at which BOTH wheels start from standstill, and add ~5. For the turn profile, measure it on the floor while spinning in place (tyre scrub).
2. **kineticPwm** (`forward::kKineticPwm`, `turn::kKineticPwm`): with the wheels already turning, lower the PWM until a wheel stops, and add ~3. It must be <= staticPwm. On the robot you can also lower it until the creep phase stutters or a `STALL` fault appears, then back off.
   - The full staticPwm offset (and a staticPwm wheel floor) applies until the filtered speed reaches `kRollingEnterTpp` (1.0 tpp).
   - The kinetic offset and floor then apply until the speed drops below `kRollingExitTpp` (0.25 tpp). This hysteresis stops single backlash ticks from toggling between the two.
   - The per-wheel floor is `kWheelFloorFraction * (staticPwm or kineticPwm)`.
3. **Feed-forward** (`kFfPwmPerTpp`): must be recalibrated against kineticPwm. Drive a cell, read `cruisePwm` and `cruiseV` from the `move` line, and set `kFfPwmPerTpp = (cruisePwm - kineticPwm) / cruiseV`. The speed PI then only corrects small errors (battery sag, load).
   - The provisional 5.25 reproduces the phase-1 cruise PWM, but is ~6 PWM lower at creep speed (the PI makes up the difference).
   - If creep is sluggish after calibration, check the creep phase separately. A single line through (0, kineticPwm) is the model.
4. **Trim** (`kLeftPwmTrim`, `kRightPwmTrim`): set `kHeadingKp`/`kHeadingKi` to 0, drive a cell on the floor and compare L and R. Lower the trim of the faster wheel by the ratio (e.g. L=2100, R=2000 -> `kLeftPwmTrim = 2000/2100 = 0.95`). Restore the gains afterwards.
5. **Ticks per cell / per 90 deg**: drive 1 cell and measure the real distance d_mm. Scale `kTicksPerCellOverride` by cell_mm / d_mm. For turns, measure the angle over 4 turns and scale `kTicksPer90Override` the same way. Alternatively, measure the geometry (`kWheelDiameterMm`, `kWheelbaseMm`, `kCellSizeMm`, encoder CPR, gear ratio) and set the overrides to 0 to use the derived values. The current geometry values were back-derived from the calibrated counts; they are not measurements.
6. **Speeds** (`kCruiseSpeed`, `kCreepSpeed`): raise the cruise speed until the wheels slip or moves get unreliable, then back off ~20%. The creep speed must be slow but smooth (no stuttering). A static_assert checks that cruise speed is reachable within a cell.
7. **Brake lead** (`kBrakeLeadTicks`): add the average `over` of ~5 runs to `kBrakeLeadTicks`, and repeat until `over` averages ~0.
8. **Heading PID** (`kHeadingKp/Ki/Kd`): raise Kp until the robot wobbles, then halve it. Raise Ki until a remaining constant curve disappears. Add a little Kd only if it oscillates. `H=` should stay small from cell to cell. `kHeadingCarryRampPeriods` sets how gently the carried error is re-applied (default 10 periods = 100 ms).
9. **Speed PI** (`kSpeedKp/Ki`): if `cruiseV` oscillates, lower Kp. If `cruiseV` settles below `kCruiseSpeed`, raise Ki.
    - Exploration robustness (`kWallFlipObservations`, `kSensorFailurePasses`, `kExplorationBudgetFactor`) normally needs no tuning.
    - Raise `kWallFlipObservations` (max 4) if the maze dump shows walls toggling.
10. **Wall centering** (optional): build with `-D MM_WALL_CENTERING=1`.
    1. Place the robot centred in a cell with walls on both sides. Set `kCenteringNominalLeftCm`/`RightCm` to the readings (the `sense` debug line shows them).
    2. Start with a low `kCenteringGainTicksPerCm` (e.g. 3) and raise it until the robot visibly re-centres within a cell without weaving. Lower `kCenteringFilter` if the correction is jittery. Raise `kCenteringPingPeriods` if the side sensors disturb each other.
    3. `kCenteringMaxTicks` bounds the correction; keep it below `kHeadingCarryLimit`.
    4. `kCenteringMaxDeviationCm` rejects wall gaps and angled echoes.
    5. The centering deliberately rotates the robot, and that rotation ends up in the heading carry `H`. If straight-line accuracy gets worse, lower the gain.

Anti-windup is automatic: the PID integrators freeze while their output is saturated in the direction of the error.
That covers both their own limit and the external PWM floor/ceiling, which the move loop reports back.

## Log output

Info (default):
```
Micromouse: maze 5x4 start (4,0) facing U goal (0,3)
at (3,0) facing U
...
Solved: goal (0,3) reached after 7 cells
+---+---+---+---+
|  3   2   1   0|
+   +---+ . +   +
...
```
The maze dump shows the flood distance of each cell, `@` for the robot and `-` for unreachable cells.
Walls are drawn as `---`/`|`, known-open sides as blanks, and sides not yet observed as ` . `/`:`.

Errors: `FAULT code=4 (STALL) in MOVE at (2,1) facing R moves=9 - halted`

Debug (`MM_LOG_LEVEL=3`, replaces phase-1 `ENGINE_DEBUG`), per cell:
```
sense L/F/R cm= 7(W) far(-) 35(-)        median per sensor, (W) = wall
<maze dump after every plan>
move target=2068 L=2071 R=2066 over=0 cruisePwm=66.93 cruiseV=7.98
H=5
```
- `target`: the commanded ticks (turns include the carry correction).
- `L`/`R`: the final encoder counts after settling.
- `over`: (L+R)/2 - target, the input for calibration step 7.
- `cruisePwm`/`cruiseV`: the average base PWM and filtered speed [tpp] while at cruise speed (step 3).
- Optional fields: `overruns=N` (control periods that ran late) and `settle=TIMEOUT` (the wheels were still ticking after 500 ms). After a settle timeout the carry is still updated from the counts at the timeout, because the move has already acted on the old carry; this is shown as `H=... (unsettled)`.
- A failed move prints `move FAILED status=<code> ...`.
- `H`: the heading error carried into the next move [left - right ticks].

## DONE:
- ~~Simple Pid System to ensure balanced speed of both motors~~
- ~~Flood Fill Algorithm for maze solving~~
- ~~Sensor integration for detecting maze walls in real time~~
- ~~Maze Management system for updating maze state in real time~~
- ~~Fixed-rate motion control: speed profile, speed PI with feed-forward, heading PID with carry~~
- ~~Layered, heap-free firmware with host unit tests~~

## TODO:
- Return to start and speed run on the known shortest path (see `Mission`)
- Calibrate the optional side-wall centering on the real maze
