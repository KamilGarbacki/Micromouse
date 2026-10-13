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
Straight moves and spins in place run a 100 Hz closed loop: a speed profile, a per-wheel feed-forward (from the motor
self-calibration) plus a PI on the average wheel speed, and a heading PID on the left/right encoder difference whose
residual is carried into the next move. After a straight move towards a wall the robot re-centres itself on the front
wall, and while it drives it refines its own calibration (wheel scale, cell pitch and front-offset error, ticks per
90 deg, brake leads).

## Architecture

```
src/
  main.cpp              setup()/loop() -> app::RobotApp (nothing else)
  config/               the ONLY place for numbers
    pins.h              pin map (+ DRI0039 M1/M4 warning)
    geometry.h          measured wheel, encoder, gear, wheelbase, cell, sensor offsets -> default ticks + learning bounds
    tuning.h            every tunable with units and calibration step
    maze_config.h       maze size, start pose, goal, static capacity
    features.h          feature flags, log level
    config.h            umbrella + static_asserts on all of the above
  util/                 header-only, dependency-free
    status.h            Status result codes (printed by FAULT)
    crc16.h             CRC-16/CCITT-FALSE (calibration store)
    flash.h             constant tables in PROGMEM (plain memory on the host)
    ring_buffer.h       fixed-capacity FIFO
    median.h, math_utils.h
  hal/                  the only layer that talks to the Arduino core
    time, gpio (PWM + Timer2 fast-PWM fix), encoders (ISRs, atomic counters), log (Serial in/out), eeprom
  drivers/              Motor, Encoder, UltrasonicSensor (NewPing hidden in the .cc)
  control/              Pid, MotionProfile, WheelMixer, WallCentering,
                        heading_carry.h, stiction.h, encoder_health.h (pure)
                        calibration (stored record, validation, per-wheel motor model; pure)
                        learning (online estimators, brake lead, alignment math; pure)
                        ff_fit (least-squares feed-forward line; pure)
                        MotionController (the move loop; uses hal + drivers)
  navigation/           pose.h (Cell, Direction, heading math), Maze (sticky walls),
                        FloodFill, Explorer (per-step decisions), maze_render (pure)
                        run_log (compact EEPROM record of the last run; pure)
  app/                  RobotApp state machine, wall sensing + ranging, Serial maze printer,
                        calibration_tools (startup menu modes e/c/p/l), online_learning (gating + save policy)
                        run_log_printer (Serial rendering of run_log), status_name (shared Status -> text)
test/                   Unity host tests for the pure modules
```

Layers, bottom to top. Each one may only use the layers below it:

| Layer        | Responsibility                                            | Arduino.h?                      |
|--------------|-----------------------------------------------------------|---------------------------------|
| `config`     | constants, derived values, compile-time checks            | no                              |
| `util`       | status codes, containers, small math                      | no                              |
| `hal`        | registers, ISRs, time, Serial, EEPROM                     | yes (the only layer)            |
| `drivers`    | one class per device                                      | only `ultrasonic_sensor.cc` (NewPing) |
| `control`    | PID, profile, mixer, centering; the move loop             | no (`motion_controller.cc` uses `avr/pgmspace.h`) |
| `navigation` | grid math, wall map, flood fill                           | no                              |
| `app`        | state machine, sensing policy, reporting                  | no                              |

`navigation/pose.h` is a dependency-free vocabulary header, so `config/maze_config.h` can use `Cell`/`Direction`.
Everything under `config`, `util`, `navigation` and the pure `control` modules builds on the host (see Tests).

### State machine (`app/robot_app.*`)

```
INIT -> SELECT_MODE -> START_RUN -> SENSE -> PLAN -> MOVE -> SENSE -> ... -> GOAL_REACHED -> IDLE
           |  ^                       \        \       \
           v  |  e/c/r/p                +--------+-------+--> FAULT   (motors braked, error reported on Serial)
```

- **INIT**: motors braked, encoders and sensors set up, calibration loaded from EEPROM (validated; defaults on failure).
- **SELECT_MODE** (only in the `uno_setup` firmware, `MM_MENU=1`): prints the menu and waits `kModeSelectTimeoutMs` (3 s) for a key: `e` encoder check, `c` motor self-calibration, `r` reset calibration, `p` print calibration, `l` print the stored run log (see "Run log"), `g` run. No key -> RUN. Only these keys count: any other byte (line noise, CR/LF) is ignored and does not stop the timeout. After a tool the menu waits without timeout (the robot may still be lifted). In the competition firmware `uno` (and with `MM_LOG_LEVEL=0`) the robot runs at once.
- **START_RUN**: prints the previous run's log (see "Run log"), waits `kPreRunDelayMs`, writes the new run log's START RUN header, then resets the maze storage and explorer.
- **SENSE**: if the robot is in the goal cell, go to GOAL_REACHED. Otherwise ping left/front/right and hand the observations to the `Explorer` (see "Wall map policy"). A failed sensor -> FAULT (logging a final run-log record first, since no move was attempted).
- **PLAN**: flood fill, then pick a direction. Ties are broken in favour of the current heading, then a 90 deg turn over a 180. Appends the run-log record for this cell (pose, walls, chosen direction). No path, or more than `kExplorationBudgetFactor` x cells driven -> FAULT (logging a final "no move" record instead).
- **MOVE**: turn if needed and drive one cell, take fresh sensor readings, feed the online learning, align on the front wall. Stall or timeout -> FAULT, and the pose is not updated.
- **GOAL_REACHED**: stop, close the run log (END RUN, GOAL_REACHED), save the learned calibration if it changed, report, print the discovered maze. Then the `Mission` decides what comes next (currently: IDLE).
- **FAULT**: the motors stay braked, the run log is closed (END RUN, FAULT) and `END RUN result FAULT <NAME> at <row>,<col>@<U|R|D|L> n=<moves>` is repeated every 5 s. Codes are listed in `util/status.h`. The LED is not used because D13 is the right sensor's trigger.

All decisions of SENSE/PLAN/MOVE (map update, sensor plausibility, planning, pose, budget) live in the
pure `navigation/explorer.*`, which the host tests drive through complete simulated runs; `RobotApp` only
sequences them with the hardware.

FAULT codes (`util/status.h`): 1 INVALID_ARGUMENT, 2 OUT_OF_BOUNDS, 3 NO_PATH, 4 STALL, 5 MOVE_TIMEOUT,
6 NOT_INITIALIZED, 7 QUEUE_OVERFLOW, 8 SENSOR_FAILURE, 9 EXPLORATION_LIMIT, 10 ENCODER_FAULT (one encoder
silent while the other counts: once one wheel has > 20 mm the other must have >= 30 % of it, see the
troubleshooting table).

Extension point: `Mission` in `robot_app.h`. A *return to start* is `explorer_.setGoal(start)`, then SENSE again.
A *speed run* is PLAN on known walls only, with faster `MoveParams`. Both would be dispatched in `onGoalReached()`.

### Conventions
- Coordinates: `Cell{row, col}`, with row 0 at the top and col 0 at the left. `Direction::kUp` means row - 1 and `kRight` means col + 1. Config uses the same convention: start (4,0) facing Up is bottom-left, goal (0,2) is top-right.
- Naming: types `PascalCase`, functions and variables `camelCase`, members `camelCase_`, constants and enumerators `kPascalCase`, macros `MM_UPPER_CASE` (log level and feature switches only), files `snake_case`.
- Formatting follows `.clang-format` (Google).
- No heap. There is no `new`, `malloc`, `String` or container library; all storage is static, sized from `config/maze_config.h`.
- Constant strings are wrapped in `F()`. The constant parameter tables of the move loop live in `PROGMEM`.
- Errors are returned as `Status`, never silently ignored. A failed move, a missing path or an internal error ends in FAULT.

## Build and test

There are three firmwares (envs). **Flash `uno_setup` first, run `e` and `c` on the bench, then flash `uno` for the
maze** (or `uno_debug` to watch the learning live). All three use the same EEPROM layout, so the calibration stored by
one is loaded by the others.

| Env          | Purpose                                                                     | Menu | Learning | Log level |
|--------------|-----------------------------------------------------------------------------|------|----------|-----------|
| `uno_setup`  | **bench tools only**: `e` encoder check, `c` self-calibration, `r` reset, `p` print the stored/learned values | yes | no | 3 debug |
| `uno`        | **competition** (default env): learning + front alignment                   | no   | yes      | 2 info    |
| `uno_debug`  | **competition + live learning logs**: `uno` with the debug log (`learn ...`, `move ...`, maze after every plan) | no | yes | 3 debug |

`uno_setup` has no online learning (it would not fit next to the menu); if it is left to time out into RUN it drives
with the stored calibration but does not refine it.

```
pio run -e uno_setup -t upload  # bench firmware (menu)
pio run -e uno -t upload        # competition firmware
pio run -e uno_debug -t upload  # competition firmware + debug log
pio device monitor -b 115200    # serial log
pio test -e native              # host unit tests (Unity)
```

Warnings are enabled for project sources (`-Wall -Wextra`). The AVR envs use `-mcall-prologues -Wl,--relax
-fno-inline-small-functions` (about -1 KB together; `-fno-inline-small-functions` alone measured -218 B in `uno`,
-82 B in `uno_debug` and +404 B in `uno_setup`, which has room). It costs a call (a few us) per small function, far
below the 10 ms control period.

The native tests need `gcc`/`g++` on `PATH`. On this Windows machine the CLion-bundled MinGW works:

```
PATH="$LOCALAPPDATA/Programs/CLion/bin/mingw/bin:$PATH" pio test -e native
```

Test suites:
- `test_navigation`: heading math, walls, sticky walls, flood fill, tie-break, maze rendering.
- `test_explorer`: end-to-end SENSE -> PLAN -> MOVE simulation on known mazes (open, dead end, serpentine, enclosed goal -> NO_PATH), flickering wall readings, exploration budget, sensor failure.
- `test_pid`: including anti-windup.
- `test_motion_profile`: continuity, creep before the stop point, short moves.
- `test_control_utils`: wheel mixer (per-wheel base and floor), wall centering, heading carry (incl. reverse adjust), stiction hysteresis, encoder health (silent wheel, start skew, thresholds), median, ring buffer.
- `test_online_learning`: the app-level learning glue (`OnlineLearning`) driven with fake move reports and readings of a simulated robot in corridors with open cells between their end walls: open cells keep the pitch reference (n >= 2 observations), the provisional path engages, the joint solve converges to the true pitch and front offset, the provisional pitch is used while all runs are alike, and lost alignments, 90 deg turns and mid-corridor U-turns drop the reference.
- `test_calibration`: geometry derivation (960 ticks/rev), profile scaling, physical gains / thresholds at the true resolution, CRC-16, both store records (corrupt byte, empty vs foreign magic, wrong version/key, out-of-range fields, seed restart), feed-forward fit, the robust estimator (median seeding, the 10-15 % band, isolated outliers, lock-in recovery, stale EEPROM start, bounds), wheel-scale samples, the joint pitch / front-offset estimator on a simulated maze whose TRUE wall thickness and sensor offset differ from the configured ones (exactness of the observation equations, joint convergence of P and delta, the provisional path, minimum rows, seeded restart after a maze change, conditioning by the a / b ratio), ticks-per-90 (CW/CCW x left/right wall x over/under rotation, encoder-visible heading subtracted, straight-line curvature subtracted, carry-clip detection, and a geometric pose -> sensor simulation of the whole chain), brake lead, alignment target and brake lead.
- `test_run_log`: step encode/decode round trip (all row/col/heading/wall/move/noMove combinations), the absolute-wall derivation (boundary fallback for the unsensed back side, matching the worked example in "Run log"), header CRC (detects a corrupt byte in the header or in a step), an interrupted run (power loss mid-run: no END RUN, `state` stays `kInProgress`), truncation at capacity (further steps are dropped, the `truncated` flag is set, the run keeps going), a blank/foreign/wrong-version EEPROM, the EEPROM region not overlapping the calibration records (and the capacity formula), and a full round trip of a goal-reached and a fault (with its final "no move" record) run through the real `RunLog` writer and the reader functions.

## Simulator / visualisation

`tools/sim/` is a host program that watches the robot explore a maze without any hardware. It links the REAL
pure navigation code (`navigation/explorer.*`, `maze.*`, `flood_fill.*`, `maze_render.*`) and the same
firmware config (`config::kStartCell`/`kStartHeading`, `kGoalCell`, `kWallFlipObservations`,
`kSensorFailurePasses`, `kExplorationBudgetFactor`) that `RobotApp` uses - it does not re-implement any
decision logic, only the SENSE -> PLAN -> MOVE sequencing of `robot_app.cc` (goal check, observe the 3
sensors left/front/right relative to the heading, plan, turn + move, advance). **Scope: navigation-level
only** - cell-to-cell moves, no motor physics, no timing.

Build once, then run the program directly with its own CLI flags (two-step; this is what works with
PlatformIO `native` on Windows). Like `env:native`, this builds on the host and needs `gcc`/`g++` on `PATH`;
on this Windows machine the CLion-bundled MinGW works:

```
PATH="$LOCALAPPDATA/Programs/CLion/bin/mingw/bin:$PATH" pio run -e sim
./.pio/build/sim/program.exe tools/sim/mazes/config_maze.txt
```

(the binary is `.pio/build/sim/program` without `.exe` on Linux/macOS, where a system `g++` is normally
already on `PATH`). Options:

| Flag                   | Meaning                                                                      |
|------------------------|-------------------------------------------------------------------------------|
| `--noise <p>`          | probability (0..1) of flipping a wall reading per observation (sticky walls / flicker) |
| `--seed <n>`           | RNG seed for `--noise` (default 1)                                            |
| `--dead-sensor <L\|F\|R>` | that sensor always reports no echo (exercises `SENSOR_FAILURE`)            |
| `--out <file.html>`    | output HTML path (default `tools/sim/out/<maze>.html`)                        |

It prints a short text summary (start/goal, options, outcome) and the final discovered maze
(`navigation/maze_render`) to stdout, and writes a single self-contained HTML file (inline CSS/JS, the run
trace inlined as JSON, no external libraries - open it straight from `file://`) that animates the run on a
canvas: true walls (light grey) vs discovered walls (black) vs known-open sides (small blue dot), the flood
distance in every cell, the robot as an arrow with a trail of visited cells, start/goal highlighted, and
play/pause, step forward/back, a speed slider and a step counter. The status line shows the current state
(SENSE/PLAN/MOVE), the planned direction, moves so far, and the final outcome
(`GOAL_REACHED after N moves`, or `FAULT <NAME>`, e.g. `NO_PATH` / `SENSOR_FAILURE` / `EXPLORATION_LIMIT` /
`STALL` if noise tricks the map into an opening that is really a wall).

Maze format: the same ASCII art as `maze_render`'s output (3-char cells, `+` corners, `---`/`|` walls, blank
= open), with `S`/`G` marking the start/goal; if either is absent the firmware's configured start cell,
heading and goal are used (and the maze must then be at least that big). Size is validated against
`config::kMazeMaxRows/kMazeMaxCols` with a clear error. The first line of cells is row 0 (top), same as
`Cell{row, col}` everywhere else in the firmware (`navigation/pose.h`). Example, a 2x3 maze with the start
at the top-left (row 0, col 0) and the goal at the bottom-right (row 1, col 2):

```
+---+---+---+
| S |       |
+   +---+   +
|       | G |
+---+---+---+
```

Example mazes in `tools/sim/mazes/`:
- `config_maze.txt` - 5x3, matches the current `config::kMazeRows/kMazeCols/kStartCell/kGoalCell`.
- `dead_ends.txt` - same size, a single-path maze the robot must back out of dead ends to solve.
- `unreachable_goal.txt` - the goal is walled in on both interior sides -> `FAULT NO_PATH`.

```
./.pio/build/sim/program.exe tools/sim/mazes/config_maze.txt --out tools/sim/out/config_maze.html
./.pio/build/sim/program.exe tools/sim/mazes/dead_ends.txt --out tools/sim/out/dead_ends.html
./.pio/build/sim/program.exe tools/sim/mazes/unreachable_goal.txt --out tools/sim/out/unreachable_goal.html
./.pio/build/sim/program.exe tools/sim/mazes/config_maze.txt --noise 0.08 --seed 1 --out tools/sim/out/noise_run.html
./.pio/build/sim/program.exe tools/sim/mazes/dead_ends.txt --dead-sensor L --out tools/sim/out/dead_sensor_run.html
```

Compile-time options (add to `build_flags` of an env):

| Flag                    | Default | Meaning                                                    |
|-------------------------|---------|------------------------------------------------------------|
| `-D MM_LOG_LEVEL=n`     | 2       | 0 none, 1 errors, 2 info, 3 debug (calibration output)     |
| `-D MM_MENU=1`          | 0       | startup menu with the bench tools (set by `uno_setup`)     |
| `-D MM_WALL_CENTERING=1`| 0       | side-wall centering during straight moves                  |
| `-D MM_FRONT_ALIGN=0`   | 1       | front-wall alignment after straight moves                  |
| `-D MM_LEARNING=0`      | 1       | online learning (wheel scale, cell pitch, ticks per 90 deg, brake leads) |

Memory. The RAM figure is static RAM; the rest is left for the stack:

|                      | RAM (static)                          | Flash   |
|----------------------|---------------------------------------|---------|
| phase 1              | 450 B + heap (maze, sensors, queue)   | 14920 B |
| phase 2              | 702 B, no heap                        | 14188 B |
| phase 3 `uno`        | 899 B, no heap                        | 28488 B |
| phase 3 `uno_setup`  | 904 B, no heap                        | 25876 B |
| phase 3 `uno_debug`  | 899 B, no heap                        | 30990 B (1266 B free) |
| phase 4 `uno` (+ run log)        | 929 B, no heap             | 30050 B (2206 B free) |
| phase 4 `uno_setup` (+ run log)  | 936 B, no heap             | 26904 B (5352 B free) |
| phase 4 `uno_debug` (+ run log)  | 929 B, no heap             | 32164 B (**92 B free**) |

Other phase-3 builds (flash of 32256 B): `uno_debug` + centering 32090 B (only ~166 B free - the tightest build: it cannot take any addition; add
features to `uno_debug` only without `MM_WALL_CENTERING`);
measured before the encoder-resolution fix (which added ~300-650 B): `uno` + centering 29208 B; `uno_setup` +
centering 26310 B; `uno` log level 0: 24992 B; `uno` with `MM_LEARNING=0`: 20142 B.
The ~24 KB target for `uno` is exceeded by ~4 KB: the online learning is ~8 KB of soft-float code (robust
estimators, the joint pitch / offset solve, the drift and curvature learning).
The robot object is constant-initialised (constexpr constructors, no start-up code), and constant tables (motion
parameters, learning parameters, default motor model) live in flash (`util/flash.h`).
(64 B of the RAM are the per-side contradiction counters of the sticky wall map; phase 3 adds the two calibration
records (102 B) and the learning state.)

**Flash budget (phase 4, the run log)**: `uno` and `uno_setup` have comfortable room (2206 B / 5352 B free).
`uno_debug` does not: it already spent almost all of its phase-3 margin on the debug log, so the run log (header,
steps, EEPROM I/O, Serial printing) only just fits, at **92 B free** - any further addition to `uno_debug` needs to
either trim something first or stay out of that build. What was done to fit it:
- The per-step line and the START RUN / END RUN lines use `char`s (`'#'`/`'.'`, `U`/`R`/`D`/`L`) instead of words
  (`wallChar()`/`directionLetter()`), and plain non-variadic `hal::log::print()` calls instead of one large
  `fmt()` with many `%`-conversions (measured smaller on the AVR: a long variadic format call costs more code to
  marshal its arguments than a sequence of simple calls).
- `uno_debug` and `uno_setup` (`MM_LOG_LEVEL=3`) print the step's relative walls (`L F R`) only; the plain `uno`
  info build (`MM_LOG_LEVEL=2`) also gets the absolute walls (`N E S W`), see `kPrintAbsoluteWalls` in
  `app/run_log_printer.cc`. The stored EEPROM record is identical either way (3 relative wall bits per step):
  only the on-device *printing* of the derived absolute sides is skipped to save flash.
- The per-move `at (<row>,<col>) facing <H>` info line (after every completed move) was dropped: the same
  position/heading is now in the run log, printed at the next boot.
- The periodic FAULT re-report (`runFault()`, every `kFaultReportIntervalMs`) now reuses the run log's END RUN
  line instead of its own `FAULT code=<n> (<NAME>) in <STATE> ...` text, so there is only one such string table,
  not two (this also means the re-report no longer names the *state* the fault happened in - only the FAULT
  report at the moment of the fault and the run log's stored record do).
- `statusName()` (Status -> text) was de-duplicated into `app/status_name.*`, shared by the FAULT report and the
  run log instead of each having its own copy.
- "truncated" / "interrupted" / "no log stored" use short, fixed wording (not configurable per build).

## Sensing
- **Mapping** happens only while the robot is stopped. Each sensor is pinged `kSonarSamples` (3) times, round-robin, with `kInterPingGapMs` (30 ms) of silence between any two pings, so the sensors do not hear each other's echoes. The median echo time decides the wall (`< kWallThresholdCm`, 20 cm). "No echo" counts as far away, so one dropout cannot fake a wall. NewPing's range is clamped to 2 cells (44 cm), which bounds every ping to about 3 ms.
- **Distances** are echo times converted with a temperature-corrected speed of sound: `kSonarUsPerCm = 20000 / (331.3 + 0.606 * kAirTemperatureC)` (58.2 us/cm at 20 degC; ~0.17 %/degC). The ultrasonic sensor is the ruler of the wheel-scale and cell-pitch / front-offset learning, so set `kAirTemperatureC` to the room temperature. (NewPing's fixed `kNewPingUsPerCm` = 57 us/cm, ~32 degC, is only used for its range limit.)
- **Ranging** (alignment and learning): after every cell the front sensor, and the side sensors beside known walls, are pinged `kRangingSamples` (3) times and the median is used (adds up to ~0.3 s per cell).
- **Wall centering** (optional, `MM_WALL_CENTERING=1`, default off): during straight moves a side sensor is pinged every `kCenteringPingPeriods` (2) control periods, alternating left and right: L, -, R, -, so there are 20 ms between any two pings.
  - The ping runs after the motor outputs are written. Normally it blocks for at most ~1.7 ms (20 cm range). A dead sensor can block ~8.2 ms (NewPing's 5.8 ms start timeout plus the echo windows); a static_assert keeps that under one control period.
  - The echo time is used in microseconds, which gives sub-cm resolution.
  - A side is used only if the maze map says a wall is there: in the current cell for the first half of the move, in the next cell for the second half. The reading must also be within +-5 cm of nominal.
  - The offset from nominal, times the gain, is low-pass filtered (`kCenteringFilter`), clamped to +-3.25 deg (`kCenteringMaxDeg`, 40 ticks of L - R) and added to the heading error. See "Wall centering" in the troubleshooting table.

### Wall map policy
- An unknown side becomes known with its first observation. A known side flips only after `kWallFlipObservations` (2) consecutive contradicting observations; a confirming one resets the count. The count is shared by both cells of a wall. Readings that flicker around the threshold therefore cannot make the map oscillate. Setting it to 1 gives "latest reading wins".
- A sensor that gets no echo on any of its samples while the map already has a wall right beside it (e.g. the outer boundary) is implausible. That observation is not applied to the map, so a dead sensor cannot erase walls. After `kSensorFailurePasses` (3) such passes in a row -> FAULT SENSOR_FAILURE. Any echo resets the count.
  - Plain "no echo" facing a side that is not known to be walled is normal (an open corridor, especially for the front sensor), so it is neutral.
  - Limitation: a dead sensor that never faces a known wall is not detected here. It then reports "open", and the robot will stall against the real wall (FAULT STALL).
  - Rare false alarm: a wrongly mapped wall in front of a > 44 cm open corridor looks the same as a dead sensor.
- Exploration is bounded: more than `kExplorationBudgetFactor` (4) x rows x cols cells driven -> FAULT EXPLORATION_LIMIT. This backstops any livelock the sticky map does not prevent.

## Calibration

Measure with a ruler, run one self-calibration command, then just drive the maze - the robot refines the rest itself.

> **AFTER THE ENCODER-RESOLUTION FIX (8 pulses per motor revolution, 960 ticks per wheel revolution) YOU MUST
> RERUN `e` AND `c` IN `uno_setup`.** The stored motor record is keyed on the CONFIGURED resolution (1920 ticks/rev).
> The self-calibration counted real ticks, so the model itself is probably still physically valid, but the record is
> rejected because the key changed, and the boot log says
> `cal: WARNING motor record is for another configured resolution (1920 ticks/rev, now 960): rerun 'c' to confirm`.
> The learned values (keyed on the old 2069 / 1108 ticks per cell / 90 deg) are discarded too
> (`cal: learned values GEOMETRY_CHANGED - defaults`) and relearnt while driving. Until `c` has been rerun the
> robot drives on the default motor model.

1. Flash the bench firmware `uno_setup` and open the monitor (115200 baud). After a reset it prints
   `MENU: e=encoder check c=motor self-cal (LIFTED) r=reset cal p=print cal g=run (RUN in 3 s)`.
2. Steps 1 and 3 below (`e`, `c`).
3. Flash the competition firmware `uno` (or `uno_debug` to see the learning live) and drive the maze (step 4). It
   loads the stored calibration. To look at the learned values later, flash `uno_setup` and press `p`.

### Step 1 - encoder check (`e`): confirm the ticks per wheel revolution

> **The 2x question - settled.** The firmware counts RISING edges of ONE hall channel. The encoder gives
> **8 pulses per motor revolution x 120:1 = 960 ticks per wheel revolution** (0.2127 mm per tick). This is
> confirmed by the real robot: configured with 16 (1920 ticks/rev) it drove **exactly 2 cells per move**. It also
> matches the original hand-tuned firmware, which turned with 554 ticks per 90 deg (= the value derived at 960),
> while its 2068 ticks per cell (= 2 cells at 960) was the "drives too far" bug. The `e` mode checks this on any
> robot in a minute.

1. Press `e`. The motors stay braked and `enc L=<n> R=<n> exp=960 L/exp=<ratio> R/exp=<ratio>` is printed whenever
   a count changes.
2. Mark a tyre and turn that wheel by hand **exactly one revolution** (either direction - the encoders are direction blind).
3. Expect **960** (= `kEncoderPulsesPerMotorRev` x `kGearRatio`), i.e. a ratio of **1.00** (+-0.02 for an
   imprecise hand turn). Ratio ~0.5 on both wheels: `kEncoderPulsesPerMotorRev` in `src/config/geometry.h` is 2x
   too high (the robot drives 2 cells per move); ~2.0: 2x too low - fix it there; every tick count, profile
   distance, speed, gain and threshold is derived from it. A wheel that stays at ~0 has a dead / unplugged encoder.
   Repeat for the other wheel. Any key exits.

### Step 2 - measure the geometry into `src/config/geometry.h`

| Constant                     | Value     | How to measure                                                              |
|------------------------------|-----------|-----------------------------------------------------------------------------|
| `kWheelDiameterMm`           | 65        | outer tyre diameter                                                         |
| `kWheelWidthMm`              | 26        | tyre width (only sets the turn-learning bounds)                             |
| `kWheelbaseMm`               | 150       | between the MIDDLES of the two tyres                                        |
| `kCellSizeMm`                | 220       | cell pitch (wall centre to wall centre); the default of the learned pitch   |
| `kMinCellSizeMm` / `kMaxCellSizeMm` | 210 / 222 | bounds of the learned pitch                                           |
| `kWallThicknessMm`           | 0         | **measure**; 0 = infinitely thin walls (the error is learned, see step 4)    |
| `kFrontSensorToCenterMm`     | 60        | **placeholder - measure**: rotation centre (midpoint between the wheels) to the front sensor face (the error is learned, see step 4) |
| `kSideSensorAheadOfCenterMm` | 40        | **placeholder - measure**: how far the side sensors sit ahead of the rotation centre |
| `kEncoderPulsesPerMotorRev`  | 8         | step 1 (confirmed on the robot)                                             |
| `kGearRatio`                 | 120       | gearbox                                                                     |

Derived at compile time (static_asserts check them): 960 ticks per wheel revolution, 0.2127 mm per tick,
**1034 ticks per cell**, **554 ticks per 90 deg**. These are the defaults; there are no override constants.
The configured centred front distance is `pitch / 2 - kWallThicknessMm / 2 - kFrontSensorToCenterMm` (50 mm now);
at run time the learned pitch and the learned front-offset error `delta` are used (`nominal(P) + delta`).
Ticks per cell = cell pitch / wheel scale, and the two are learned SEPARATELY (step 4), within these bounds:
- **wheel scale** (mm per tick): -5 % / +3 % (`kWheelScale*Fraction`) - tyre compression under the robot's weight
  makes the effective rolling radius 1-3 % smaller, plus +-1 mm (1.5 %) measuring a 65 mm tyre; a loaded tyre does
  not grow, so the upper bound is the measuring tolerance only;
- **cell pitch**: 210..222 mm; **front-offset error** `delta`: +-15 mm (`kMaxWallOffsetMm`);
- **ticks per 90 deg**: +-15 % (`kTurnLearnFraction`: the 26 mm tyre makes the effective wheelbase 137..163 mm, plus scrub).

**Units policy** (`tuning.h`): every tunable is given in physical units - profile speeds and distances in mm,
mm/s and mm/s^2; gains in PWM per mm/s and PWM per degree of heading; heading thresholds (carry limit, turn-learning
gate, centering) in degrees; the encoder-fault threshold in mm - and converted to ticks at compile time with the
geometry, so the encoder resolution enters in ONE place. Exceptions, in ticks on purpose because they are about tick
quantisation: the stiction hysteresis (`kRollingEnterTpp`/`ExitTpp`) and the self-calibration motion detection
(`kSelfCalMovingTicks`). The gains and thresholds keep the PWM per PHYSICAL tick the robot already ran with (the
encoder hardware never changed, only its configured resolution); the profile keeps its mm values, so the robot now
really drives 85 mm/s cruise (4.0 tpp), 32 mm/s creep (1.5 tpp) and 30 mm/s turn creep (1.41 tpp) - with the
wrong 16 pulses it physically drove twice as fast. The compile-time profile checks use the worst case of the
run-time values (210 mm with the largest scale = 958 ticks per cell; the largest brake leads), and assert that both
creep speeds stay >= 1.2x the rolling threshold (tick quantisation).

### Step 3 - motor self-calibration (`c`), robot lifted

Put the robot on a stand so **both wheels spin freely**, press `c`, then `y` to confirm (any other key cancels;
any key during the run aborts with the motors braked). For each wheel separately (~20 s each):

1. **staticPwm**: the PWM ramps up from 1 by 1 every 100 ms until the wheel moves (>= 3 ticks) in **2 consecutive**
   steps (a single burst can be gearbox backlash); the first of them is the breakaway (+3 PWM margin).
2. **Feed-forward line**: the steady-state speed (tpp) is measured at 5 PWM levels from staticPwm + 10 up to 160,
   and `PWM = offset + slope * tpp` is fitted by least squares (rejected if r^2 < 0.95).
3. **kineticPwm**: from the lowest level the PWM ramps down (never above that level, never below 1) until the
   wheel stops (+2 PWM margin, <= staticPwm).

The result is validated, saved to the motor record and printed (`L: static=.. kinetic=.. ff=..+..*tpp`). Each wheel's
PWM is computed from its own line for the target speed - this replaces the old `LEFT/RIGHT_PWM_TRIM`. The speed
PI (average speed) and the heading PID still act on top; mixer, wheel floor and anti-windup are unchanged.
Lifted values lack the floor load, so the move loop adds `kForwardLoadScale/OffsetPwm` (1.0 / 0) to straight moves
and `kTurnLoadScale/OffsetPwm` (1.0 / +8 PWM) to turns, whose tyre scrub needs more torque. Without a stored
self-calibration both wheels use the phase-2 defaults (`forward::kDefault*`, `turn::kDefault*`) and a warning is printed.

### Step 4 - drive the maze; learning is automatic

Put the robot on the start cell, centred, and reset. While it explores it refines, within the bounds of step 2:

- **Every learned value** uses the same robust estimator: before it has `minSamples` samples only the hard bounds
  gate, and the estimate is the **median** of the first `minSamples` samples (one wild early sample cannot bias it);
  until then the default is used. Afterwards samples beyond the outlier gate are rejected and the rest enter an EMA.
  After `kLearnReacquireCount` (4) **consecutive** outliers that agree with each other (all within the gate of their
  median, median within the bounds) the estimate is **re-seeded** from them - a stale stored value (new tyres,
  another maze) or a bad early estimate cannot lock the learning out.
- **Wheel scale** (mm per tick; front sensor as the ruler). For a plain straight move (no turn before it) with a
  wall ahead in ruler range BEFORE the move (the SENSE reading, 30..400 mm) and AFTER it (a fresh reading before any
  alignment, below the wall threshold): `sample = (d_before - d_after) / ticks`. Only for successful, settled moves,
  and only after the next SENSE has put that wall on the map. Gate 4 %, EMA 0.15, seeded from 3 samples.
- **Cell pitch P and front-offset error delta**, estimated **jointly**. The firmware knows only the configured wall
  thickness `w_c` and front-sensor offset `s_c`; the error that matters is
  `delta = (w_c - w) / 2 + (s_c - s)` (configured minus true; a truly centred robot reads `nominal(P) + delta`).
  From a known longitudinal reference the robot drives `n` straight cells (distance `D` from the encoders and the
  learned scale) to a wall ahead and reads `r_e` (>= 20 mm, the sensor minimum) before aligning. The references
  are the start cell (placed centred by hand) and an alignment on a wall, whose final, re-measured reading `r_f`
  (residual included) is stored. Open cells on the way keep the reference: a move without a wall right ahead does
  not align (`AlignResult::kNotAttempted`, the robot does not move) and the cell count simply grows. The reference
  is kept through a 180 deg turn only if the robot has not moved since it (a U-turn right after the alignment), and
  is dropped by any 90 deg turn, an alignment that lost the echo after moving (`kLost`) or a failed move. With the
  wall centre lines at multiples of P:
  - start cell: `y = D + r_e + w_c / 2 + s_c = (n + 1/2) P + delta`;
  - aligned reference: `y = D + r_e + r_f + w_c + 2 s_c = (n + 1) P + 2 delta`.

  The wall thickness and sensor offset do **not** cancel: assuming `delta = 0` biases P by `+delta / (n + 1/2)` or
  `+2 delta / (n + 1)` (12 mm walls configured as 0, `delta = -6`, make a 220 mm maze look like 214 mm from aligned
  1-cell runs, 216 mm from a 1-cell run from the start cell). So every observation `a P + b delta = y` enters a
  2-state least squares with forgetting (`kPitchJointForget` 0.9; the five sums are stored in EEPROM). Hard bounds
  (P 210..222 mm, |delta| <= 15 mm) gate every observation; once a solution is in force, residuals beyond 3 % of P
  per cell are outliers and 4 consecutive ones restart the estimator, re-seeded with the observation that
  triggered the restart. The solve is put in force only after `kPitchMinRows` (3) accepted observations since the
  last restart AND when it is **conditioned** (`1 - corr^2 >= kPitchMinDecorrelation`): that needs observations
  with a different ratio `a / b`, not merely a different `n` (an aligned run of 2k cells is collinear with a
  start-cell run of k cells). Until then a **provisional** pitch is learned from runs of >= 2 cells with
  `delta = 0` assumed (robust estimator: median seed of 3, gate 3 %, EMA 0.15), where the bias is smallest. The learned `delta`
  also corrects the alignment target, so the robot ends truly centred. Measuring `w` and `s` still helps: `delta`
  is then small, the provisional pitch is nearly unbiased and the joint solve converges faster.
  On a 210 mm maze ticks per cell become 987 instead of overdriving 10 mm per cell.
- **Ticks per 90 deg** (side-wall drift). Along a side wall that the map knows in the start AND end cell, the side
  distance at the start and end of a straight move gives the heading error the encoders cannot see
  (`eps`, + = clockwise):
  `eps = atan((sigma * (d_end - d_start) - lever * dtheta_enc - integral(theta_enc ds)) / D) ~= (...) / D`
  with `sigma` +1 for the left, -1 for the right wall and `theta_enc` the encoder heading (carry + L - R).
  Subtracting the encoder-visible heading (its integral over the move, accumulated in the move loop, and the lever
  swing of the side sensors) leaves only the residual, so the carry correction is not counted twice. `eps` is
  absolute and accumulates over turns, so a sample is the CHANGE of `eps` between the measurement before and after
  **exactly one** turn, minus the **straight-line curvature** (a wheel-diameter mismatch makes the robot curve on
  straights): the curvature [rad/mm] is learned from consecutive measurements with NO turn in between and
  subtracted over the distance between the two measurements (midpoint to midpoint). Then
  `T_true = 1 / (1/T_used + deps / (sign * pi/2 * N))` (`sign` +1 CW / -1 CCW, `N` = ticks per wheel of the turn).
  Over-rotating (deps with the sign of the turn) means fewer ticks per 90 deg.
  Gates: settled moves, |carry| at the start <= 3.25 deg (`kTurnLearnMaxCarryDeg`, 40 ticks), side readings 20..150 mm, |d_end - d_start| <= 25 mm; a
  turn whose carry was clipped (`heading::carryAfterTurnUnclamped` beyond the limit) is skipped (the baseline is
  dropped). The alignment moves count as straight distance for the curvature.
  Estimate: gate 10 %, EMA 0.1, seeded from 4 samples, +-15 % bounds (a true value 10-15 % off the default is
  reachable through the seed).
  With `MM_WALL_CENTERING=1` the centering steers only through the heading loop, i.e. its rotation is
  encoder-visible and is contained in `theta_enc`; the same formula still isolates `eps`, so learning stays on.
- **Brake leads**: after every settled move `brakeLead += 0.2 * over`, clamped to [0, 5 mm] forward and [0, 3 mm]
  for turns; |over| beyond 3x the respective maximum (a slip or bump) is ignored.
- **Front-wall alignment** (`MM_FRONT_ALIGN`, default on): after each straight move with a wall within 60 mm of the
  centred distance, the robot creeps forward or backward by the error (deadband 3 mm, max 30 mm per step, 2 rounds)
  towards `nominal(P) + delta`. The adjustments brake with the learned forward lead (they brake from the same creep
  speed) but at most half the move, so a 3 mm correction is not swallowed; the position is re-measured after every
  adjustment and the final reading becomes the pitch reference. Encoders are
  direction blind, so a reverse adjust counts its ticks with the commanded sign. It holds equal L/R with no carry
  setpoint, and the carry is then updated with the commanded sign (`heading::carryAfterStraight`): the adjustment
  does not change the heading the carry describes, so the carry is neither reset nor double counted.

Learning progress is a debug-level log, visible with `uno_debug`:

```
cal: ticks/cell=1034 ticks/90=554                  at boot: values in use (info)
learn scale S sample=213.40 est=212.71 n=1         wheel scale [um/tick]; S = seed sample
learn pitch S n=2a y/a=210.10 P=220.00 d=0.00 j=0 use=1034
                                                   joint estimator: run of 2 cells from an alignment (a) or the
                                                   start cell (s); j=0 = not yet conditioned
learn pitch(prov) + sample=210.10 est=210.30 n=3   provisional pitch (delta = 0 assumed); + = accepted
learn pitch + n=1a y/a=208.20 P=214.10 d=-5.90 j=1 use=1007   joint solution in force
learn t90 O sample=630.10 est=550.50 n=5           O = outlier, R = re-acquired / restarted, ! = invalid
learn eps=3.12 curve=0.040                         heading error [mrad], curvature [mrad/m]
brake f=7.4 t=5.0                                  learned brake leads [ticks]
align 58.2 adj=39                                  front reading [mm] and adjustment [ticks]; "align skip 150.0"
cal: saved                                         (info)
```

The estimate of a robust estimator is in force once `n` reaches its minimum (scale 3, provisional pitch 3, ticks/90 4).

`p` (in `uno_setup`) prints both records: scale, pitch, front-offset error `d`, joint flag `j` and rows since the
last restart, ticks per cell / per 90 deg with sample counts, brake leads, motor model; `r` invalidates both (back to
the defaults).

**EEPROM layout**: two independent records, each with its own magic, version, key and CRC-16/CCITT-FALSE, so a
geometry change keeps the self-calibration (it only depends on the ticks per wheel revolution) and a brown-out
while saving one record cannot corrupt the other.

Motor record (address `kMotorRecordAddress` = 0, 42 bytes, written only by `c`):

| Offset | Size | Field                                   |
|--------|------|-----------------------------------------|
| 0      | 2    | magic 0x4D4D ("MM")                     |
| 2      | 1+1  | version (1), reserved                   |
| 4      | 2+2  | key: ticks per wheel revolution (960), reserved |
| 8      | 32   | wheel[L], wheel[R]: staticPwm, kineticPwm, ffOffsetPwm, ffSlopePwmPerTpp (float) |
| 40     | 2    | CRC-16 of bytes 0..39                   |

Learned record (address `kLearnedRecordAddress` = 64, 60 bytes):

| Offset | Size | Field                                   |
|--------|------|-----------------------------------------|
| 0      | 2    | magic 0x4C4D ("ML")                     |
| 2      | 1+1  | version (4; v1 was the single combined record, v2 had no front-offset error, v3 had no minimum row count for the joint solution), joint pitch estimator state (bit 0: solution in force, bits 1..3: accepted rows since the last restart) |
| 4      | 2+2  | key: default ticks per cell / per 90 deg (a geometry change invalidates this record) |
| 8      | 4    | wheel scale [mm per tick] (float)       |
| 12     | 4    | cell pitch [mm] (float; joint solution, or the provisional estimate) |
| 16     | 4    | front-offset error delta [mm] (float)   |
| 20     | 4    | ticks per 90 deg (float)                |
| 24     | 4+4  | brake lead forward, turn [ticks] (float) |
| 32     | 20   | joint pitch / offset normal equations: sum a^2, a b, b^2, a y, b y (float) |
| 52     | 2+2+2| scale / provisional pitch / turn samples |
| 58     | 2    | CRC-16 of bytes 0..57                   |

At boot each record must pass magic, version, CRC, key and a range check of every field (NaN fails); otherwise its
defaults are used and a message names the reason (`EMPTY` for erased EEPROM, `INVALID/OLD` for another or an older
layout, `OLD_VERSION`, `BAD_CRC`, `GEOMETRY_CHANGED`, `OUT_OF_RANGE`). Seed samples live only in RAM: a sample count below the minimum restarts that seed after a reboot.
**Wear policy**: the learned record is written on GOAL_REACHED (if changed) and after every `kLearnSaveEveryUpdates`
(8) accepted samples - never per move - and only changed bytes are written (`EEPROM.update`). Nothing is saved on
FAULT. With ~100k cycles per byte this lasts for many thousands of runs.

### Step 5 (optional) - troubleshooting

| You see                                                        | Change                                                                      |
|----------------------------------------------------------------|-----------------------------------------------------------------------------|
| **robot drives 2 cells per move** (or 2x too far, 180 deg instead of 90) | encoder resolution or a dead encoder: run `e` in `uno_setup`. Both ratios ~0.5: halve `kEncoderPulsesPerMotorRev` (geometry.h); one wheel ~0: dead / unplugged encoder (the move loop also stops with `ENCODER_FAULT`). Then rerun `c` |
| `e` ratio ~0.5 / ~2.0 on both wheels                           | `kEncoderPulsesPerMotorRev` (geometry.h) /2 / x2, then run `c` again        |
| `FAULT code=10 (ENCODER_FAULT)`, `ENCODER FAULT L=.. R=..`      | the wheel with the low count has a silent encoder: connector, wiring, magnet; confirm with `e`. Thresholds: `kEncoderFaultMinTravelMm` / `kEncoderFaultMinRatio` |
| boot: `cal: WARNING motor record is for another configured resolution (1920 ticks/rev, now 960)` | expected once after the resolution fix: run `c` in `uno_setup`             |
| `c` prints "no motion"                                          | motor / encoder wiring; `kSelfCalMaxStartPwm`                               |
| `c` prints "bad fit/stop"                                       | slipping wheel or loose encoder magnet; `kSelfCalSettleMs` up               |
| turns start hesitantly or end in `STALL`                       | `kTurnLoadOffsetPwm` up (e.g. +4)                                           |
| turns jerk / overshoot at the start                            | `kTurnLoadOffsetPwm` down                                                   |
| straight moves start slowly                                     | `kForwardLoadOffsetPwm` up                                                  |
| many `O` (outlier) learn lines and no `R`                      | noisy readings: `kRangingSamples` up (5); check `kAirTemperatureC`          |
| `R` (re-acquired) learn lines repeating                        | the quantity really varies (e.g. mixed mazes); widen the gate               |
| `learn pitch ... j=0` for a long time                          | all runs have the same a / b (one corridor length); normal - the provisional pitch is used |
| `d=` (delta) large, e.g. beyond +-5 mm                          | measure `kFrontSensorToCenterMm` / `kWallThicknessMm` again (faster convergence) |
| robot stops consistently short/long of the wall after alignment | before `j=1`: `kFrontSensorToCenterMm` / `kWallThicknessMm`; afterwards delta corrects it |
| alignment oscillates forward/back                              | `kAlignDeadbandMm` up                                                       |
| wheels slip, moves unreliable                                  | `forward::kCruiseMmPerS` / `kAccelMmPerS2` down ~20 %                       |
| creep stutters                                                  | `forward::kCreepMmPerS` up                                                  |
| robot wobbles on straights                                      | `kHeadingKpPerDeg` down (halve); constant curve left: `kHeadingKiPerDeg` up |
| `cruiseV` (debug `move` line) oscillates / settles low          | `kSpeedKpPerMmPerS` down / `kSpeedKiPerMm` up                               |
| robot slower than before the resolution fix                    | expected (it used to drive 2x the configured speed): raise `forward::kCruiseMmPerS` gradually |
| walls toggle in the maze dump                                  | `kWallFlipObservations` up (max 4)                                          |
| want to start learning from scratch                            | `r` in the menu (`uno_setup`)                                               |

**Wall centering** (optional, `-D MM_WALL_CENTERING=1`): place the robot centred between two walls, set
`kCenteringNominalLeftCm`/`RightCm` to the `sense` debug readings, start with a low `kCenteringGainDegPerCm`
(e.g. 0.25) and raise it until the robot re-centres within a cell without weaving; lower `kCenteringFilter` if jittery,
raise `kCenteringPingPeriods` if the side sensors disturb each other, keep `kCenteringMaxDeg` below
`kHeadingCarryLimitDeg`. Its rotation ends up in the heading carry `H`.

Anti-windup is automatic: the PID integrators freeze while their output is saturated in the direction of the error.
That covers both their own limit and the external PWM floor/ceiling, which the move loop reports back.

## Run log

Opening a serial monitor resets the Uno (the USB CDC auto-reset), so a run cannot be watched live after the robot
has been disconnected and sent off into the maze on its own. Instead, every run is recorded to EEPROM as it
happens (`navigation/run_log.*`, `app/run_log_printer.*`), and **the previous run's log is printed at the next
boot**, before the new run starts and overwrites it. Only the last run is kept.

**How to read it**: upload, disconnect the USB cable, let the robot run the maze (battery power), then reconnect
the USB cable and open a serial monitor:
```
pio device monitor -b 115200
```
Opening the monitor resets the Uno, which starts the boot sequence: it prints the previous run's log first,
then waits `kPreRunDelayMs` (default 2000 ms, `config/tuning.h`) before writing the new run's START RUN header
and driving off. **If you want to keep reading the old log instead of starting a new run, lift the robot (or
otherwise stop it) within that window** - once START RUN is written the old log is gone. Set `kPreRunDelayMs`
to 0 to start immediately (no window) if you do not need to read the log that way.

In `uno_setup` (the bench firmware) the menu key `l` prints the stored log on demand, without starting a run.

**Live markers**: `START RUN ...` is printed when a run begins and `END RUN result ...` when it ends (goal
reached or a fault), at info level - so in the competition firmware `uno` too, not just the debug builds.

**Format** (see `app/run_log_printer.cc`; compact, see "Flash budget" above for why there are two variants):
```
=== LAST RUN LOG ===
START RUN 5x3 4,0@U g=0,2
 1 4,0@U  #..  ..##  U
 2 3,0@U  #..  ...#  U
 3 2,0@U  ##.  #..#  R
 4 2,1@R  ...  ....  U
 5 1,1@U  ...  ....  R
 6 1,2@R  .#.  .#..  U
END RUN result GOAL_REACHED OK at 0,2@U n=6
=== END LOG ===
```
- `START RUN <rows>x<cols> <row>,<col>@<heading> g=<row>,<col>`: the maze size, the start pose and the goal.
- One line per step: `<n> <row>,<col>@<heading>  <L><F><R>  <N><E><S><W>  <move>`, where `n` is the move number
  (1-based; the one possible final "no move" record of a fault that halted before a move is shown as `-` instead
  of a move letter and does not get a number), `<row>,<col>@<heading>` is the pose *before* the move, `<L><F><R>`
  are the relative walls that were sensed there (left/front/right of the heading) and `<N><E><S><W>` the same
  walls translated to absolute compass sides (**`uno` only**, see "Flash budget"; the back side, not sensed from
  there, falls back to whether it is an outer-boundary wall - `navigation::absoluteWalls()`), `#` = wall, `.` =
  open, and `<move>` is the direction driven (`U`/`R`/`D`/`L`, same letters as the heading).
- `END RUN result <GOAL_REACHED|FAULT> <OK|status name> at <row>,<col>@<heading> n=<moves>`: the outcome, the
  `Status` name if it was a fault (`util/status.h`) or `OK` if the goal was reached, the final cell/heading
  (the goal, or where the fault happened) and the number of moves driven.
- `(truncated)` if the EEPROM capacity was reached (logging stopped, but the run kept going; see Capacity below).
- `(interrupted)` instead of an END RUN line if the stored run never got one (`state` still "in progress"): a
  power loss or reset happened mid-run.
- `no log stored` instead of the whole block if the EEPROM is blank (erased) or does not hold a valid run log
  (wrong magic/version, or a CRC mismatch - a corrupted write).

**Write policy** (`app::RobotApp`, see the state machine above): the header "in progress, 0 steps" is written
once at START_RUN (this *is* the START RUN marker, written only after the previous log has been printed). One
step record is appended per cell, from SENSE/PLAN while the robot is stopped - **never from the 100 Hz move
loop**. At GOAL_REACHED or FAULT the header is updated to "complete" with the outcome (this *is* the END RUN
marker). A power loss mid-run simply leaves the header "in progress": the next boot prints what was recorded so
far and `(interrupted)` instead of an END RUN line.

**EEPROM layout** (`navigation/run_log.h`): the run log starts right after the learned-calibration record
(`kLearnedRecordAddress` + its size = 124), so it can never overlap the two calibration records; a
`static_assert` enforces that and that everything still fits in the 1 KB EEPROM.

| Offset | Size | Field |
|--------|------|-------|
| 0      | 2    | magic 0x524C ("RL") |
| 2      | 1+1  | version (1), state (1 = in progress, 2 = complete) |
| 4      | 1+1  | maze rows, cols |
| 6      | 1+1+1| start row, col, heading |
| 9      | 1+1  | goal row, col |
| 11     | 1    | flags (bit 0: truncated) |
| 12     | 2    | step count (<= capacity) |
| 14     | 1+1  | outcome (1 = GOAL_REACHED, 2 = FAULT), fault `Status` code |
| 16     | 1+1+1| fault row, col, heading |
| 19     | 1    | reserved (keeps the CRC at an even offset) |
| 20     | 2    | CRC-16/CCITT-FALSE of bytes 0..19 **and** every step record that follows |
| 22     | 2 x N | step records (see below), N = step count |

Each step record is **16 bits, no spare bits**: row (4), col (4), heading (2), wall L/F/R (1 each, 3 total),
move direction (2), a "no move" flag (1) for the one possible final record of a fault that halted before a move
(see `encodeStep`/`decodeStep`).

**Capacity**: `kRunLogCapacity` = `kMazeMaxRows * kMazeMaxCols * kExplorationBudgetFactor + 1` = 8 x 8 x 4 + 1 =
257 steps (514 bytes) - the worst case exploration budget at the *maximum static* maze size (not just the
currently configured one), so a future `kMazeRows`/`kMazeCols` change within that capacity never needs to touch
this. Header (22 B) + steps (514 B) = 536 B, at address 124: ends at 660, well inside the 1 KB EEPROM. If the
capacity were ever reached, further steps are simply not written (the `truncated` flag is set once and the run
keeps going; nothing above step 257 is lost in a way that matters, since the robot would be well past any sane
`kExplorationBudgetFactor x cells` budget already).

**CRC and power-loss safety**: the CRC covers the header fields up to (not including) the CRC itself, plus every
step byte that is currently stored - recomputed fresh on every write (header write, each step append, and the
final outcome write), so changing `state`/`outcome`/the fault fields afterwards never invalidates it. The write
order per step is: step bytes, then step count, then CRC (`RunLog::appendStep`). A power loss exactly between
the step-count write and the CRC write (a window of a couple of `EEPROM.update()` calls, a few ms) can make the
very last record look corrupt on the next boot; everything before it is unaffected. This is the same order of
risk the calibration records already accept for their own seal/CRC writes, and is far smaller than the chance of
simply losing USB/battery power at an arbitrary instant during normal operation.

**Wear estimate** (`EEPROM.update()`, only changed bytes are actually written, ~100k erase/write cycles per
byte): per run, one step write (2 B) plus a header write (only `stepCount` and `crc`, 4 B, actually change after
the first write) per step, plus one header write at the very start and one at the very end. For a budget-limit
run of the maximum 257 steps that is at most 257 x 6 + 2 x 22 = 1586 B of *changed* bytes, spread over the small
header region (worst case ~257 writes to the same 4 bytes) and the step region (1 write each, never rewritten).
At a few runs a day that is many thousands of runs before the header bytes approach their rated endurance -
not a practical concern for a competition robot.

**Tool**: `tools/serial_runlog.py` (optional, needs `pyserial`: `pip install pyserial`) opens the COM port,
waits for `=== LAST RUN LOG ===`, and saves everything up to `=== END LOG ===` to a file:
```
python tools/serial_runlog.py COM5 run.txt
```

## Log output

Info (default). At boot the previous run's log is printed first (see "Run log"), then the new run starts:
```
cal: ticks/cell=1034 ticks/90=554
=== LAST RUN LOG ===
START RUN 5x3 4,0@U g=0,2
 1 4,0@U  #..  ..##  U
...
END RUN result GOAL_REACHED OK at 0,2@U n=6
=== END LOG ===
START RUN 5x3 4,0@U g=0,2
END RUN result GOAL_REACHED OK at 0,2@U n=6
+---+---+---+
|  2   1   0|
+   +---+ . +
...
```
The maze dump shows the flood distance of each cell, `@` for the robot and `-` for unreachable cells.
Walls are drawn as `---`/`|`, known-open sides as blanks, and sides not yet observed as ` . `/`:`.

Errors: the run log's END RUN line is repeated every `kFaultReportIntervalMs` (5 s), e.g.
`END RUN result FAULT STALL at 2,1@R n=9`.

Debug (`MM_LOG_LEVEL=3`, replaces phase-1 `ENGINE_DEBUG`), per cell:
```
sense L/F/R cm= 7.1(W) far(-) 35.4(-)    median per sensor, (W) = wall
<maze dump after every plan>
move target=1034 L=1036 R=1033 over=0 cruisePwm=46.10 cruiseV=3.99
H=3
brake f=7.4 t=5.0
align 58.2 adj=39
```
(plus the `learn ...` lines of "Calibration", step 4; `uno_debug` / `uno_setup`)
- `target`: the commanded ticks (turns include the carry correction).
- `L`/`R`: the final encoder counts after settling.
- `over`: (L+R)/2 - target, the input of the brake-lead learning.
- `cruisePwm`/`cruiseV`: the average base PWM (mean of both wheels) and filtered speed [tpp] while at cruise speed.
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
- ~~Self-calibration and online learning of the calibration~~
- ~~EEPROM-backed run log, printed at the next boot (no live Serial needed while driving)~~

## TODO:
- Return to start and speed run on the known shortest path (see `Mission`)
- Calibrate the optional side-wall centering on the real maze
