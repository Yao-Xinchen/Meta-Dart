# Decision FSM and Launcher Coordination

This note describes the current launcher decision logic implemented by
`DecisionModule`, `SpringStretcherModule`, and `TriggerModule`.

## Core Idea

The launcher prepares the spring in parallel with dart search and pickup. The
arm is only allowed to load a dart after the spring side reports that it is
armed and the stretcher motors have retracted clear of the track.

The normal cycle is:

```text
trigger high/hold
spring stretcher prearms
vision finds a stable dart
arm picks the dart
wait until spring is ArmedAndClear
arm loads dart
arm resets/clears
trigger releases
trigger returns high
next cycle starts
```

The trigger is command-only in this version. Its state is inferred from the
last command plus `TRIGGER_SETTLE_TIME_S`; it is not sensor-confirmed.

## Module Responsibilities

`DecisionModule` is the coordinator. It reads detection, arm, spring stretcher,
and trigger state, then decides when to move between search, pickup, loading,
firing, and safety recovery.

`SpringStretcherModule` owns the two M3508 motors. It calibrates to the top
stops, stretches the spring, retracts the stretcher motors after the spring is
latched by the trigger, and can run a slow safe-retract sequence.

`TriggerModule` owns the launch trigger motor. It exposes a binary interface:
high/hold and low/release. The current hardware write function is a TODO stub.

## Main Decision States

`DecisionModule::State` currently has these states:

```text
Idle
Homing
Searching
ExecutingTrajectory
SafeDeenergize
ResetCooldown
```

`Homing` moves the arm through `search_ready_sequence` to the configured search
pose. While homing, it also tries to start spring prearm in parallel.

`Searching` keeps the arm at the search pose, reads the latest `Detection`, and
waits until the dart is stable inside one configured slot for
`stable_dart_time_s`. It also keeps trying to prearm the spring if not already
armed.

`ExecutingTrajectory` runs the selected slot trajectory. It still supports
disturb mode: if the dart moves during the guarded approach, the arm pauses,
waits for relocation, and restarts pickup from the new stable slot.

`SafeDeenergize` is the safety path for a spring-loaded failure. It does not
leave the trigger holding spring energy indefinitely. The stretcher first takes
the spring load, then the trigger releases, then the stretcher slowly retracts
to home.

`ResetCooldown` is an optional delay between cycles when configured.

## Spring Stretcher Phases

`SpringStretcherState::phase` describes what the spring side is doing:

```text
Offline
Calibrating
Home
Stretching
HoldingStretched
Retracting
ArmedAndClear
TakingLoad
SafeRetracting
Fault
```

Important meanings:

- `Home`: spring is relaxed and the stretcher is at home.
- `Stretching`: stretcher motors are pulling to the stretch target.
- `HoldingStretched`: stretcher is at the stretched target and holding load.
- `Retracting`: normal post-latch retract; the trigger is expected to hold the
  spring while the stretcher clears the track.
- `ArmedAndClear`: spring is held by the trigger and stretcher motors are clear.
- `TakingLoad`: safety path; stretcher is moving back to the stretched position
  to take load off the trigger before release.
- `SafeRetracting`: safety path; trigger has released and the stretcher is
  slowly relaxing spring energy to home.
- `Fault`: stretcher cannot safely continue automatic operation.

## Trigger States

`TriggerState::Position` is command-inferred:

```text
Unknown
HoldingHigh
ReleasingLow
```

The current implementation assumes:

- `HoldingHigh`: bar is high and can hold the spring.
- `ReleasingLow`: motor pulls the wire and lowers the bar to release spring
  energy.
- `settled`: the commanded position has had enough time to settle.

Because there is no trigger sensor, these are not physical confirmations.

## Normal Cycle Details

### 1. Prearm in parallel

During `Homing` and `Searching`, `ensure_prearm_started()` checks trigger and
stretcher state. It commands the trigger high, waits until that command is
settled, then commands `spring_stretcher_.stretch()`.

The stretcher then runs:

```text
Home -> Stretching -> HoldingStretched -> Retracting -> ArmedAndClear
```

`ArmedAndClear` is the important ready state. It means the spring is latched by
the trigger and the stretcher motors are retracted clear of the track.

### 2. Pick a detected dart

Vision publishes `Detection`. Search stability logic picks a zone only when the
dart remains inside the same configured slot long enough. The matching zone
selects the trajectory to execute.

### 3. Load only when armed

When the trajectory reaches the special step:

```json
{"position": "loading", "gripper": "open"}
```

`move_to_loading_and_release()` first calls `wait_for_prearmed()`. If the
launcher is not `ArmedAndClear`, the arm refuses to load the dart.

Only after spring readiness is confirmed does the arm move to `loading`, open
the gripper, and set `dart_loaded_ = true`.

### 4. Fire after the arm clears

The rest of the trajectory returns the arm through the configured reset/search
poses. At the end of the trajectory, `fire_loaded_dart()` verifies:

```text
dart_loaded_ == true
spring_stretcher.phase == ArmedAndClear
trigger.position == HoldingHigh
arm state is readable and hw_ok
```

Then it commands:

```text
trigger.release()
wait TRIGGER_RELEASE_PULSE_S
trigger.hold()
spring_stretcher.idle()
```

The cycle then returns to search/prearm.

## Safety Recovery

The launcher should not use the trigger as an indefinite fault-hold mechanism.
If a spring-loaded failure prevents normal firing, `safe_deenergize()` runs:

```text
spring_stretcher.take_load()
wait until stretcher reports holding_load
trigger.release()
wait TRIGGER_SAFE_RELEASE_WAIT_S
spring_stretcher.safe_retract()
wait until spring stretcher returns Home
trigger.hold()
clear dart_loaded_
clear prearm_requested_
```

This releases spring energy through controlled stretcher motion instead of
leaving the spring latched.

If the stretcher is unavailable or cannot take load, the code logs an error and
does not command a trigger release automatically.

## Key Invariants

The arm may load a dart only when:

```text
spring_stretcher.armed_and_clear == true
trigger.position == HoldingHigh
trigger.settled == true
```

The trigger may fire only when:

```text
dart_loaded_ == true
spring_stretcher.armed_and_clear == true
trigger.position == HoldingHigh
arm state is readable and hw_ok
```

Safety de-energize may release the trigger only after:

```text
spring_stretcher.holding_load == true
```

## Hardware TODO

`TriggerModule::write_trigger_position()` is still a stub. Replace it with the
real trigger motor position command once the motor/controller interface is
chosen. Keep the public module behavior binary: `hold()` for bar high and
`release()` for bar low.
