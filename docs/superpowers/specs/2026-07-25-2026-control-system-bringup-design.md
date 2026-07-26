# 2024 Robot Code → 2026 Control System Bring-Up

## Context

This repository holds the 2024 Crescendo competition robot's code. The team is
bringing it up on the 2026 FRC control system for offseason demonstration and
coding practice — not for competing in the 2026 game. `build.gradle` and
`.wpilib/wpilib_preferences.json` are already updated to GradleRIO 2026.2.1 /
projectYear 2026, and `vendordeps/` has been partially bumped (Phoenix5,
Phoenix6, REVLib, WPILibNewCommands are on 2026 versions). Two vendordep
deletions are currently staged that would break compilation, and two vendor
libraries changed their Java API surface between the versions this code was
written against and the versions now vendored in.

## Goal

Get the existing robot code compiling and deployable against the 2026 vendor
libraries with the smallest possible set of changes. Preserve existing
behavior. Where code is semantically tied to the 2024 game (game piece names,
scoring positions, auto routine names) but not actually broken, leave it
functionally as-is and mark it with a `// TODO:` comment instead of rewriting
it — this robot's autonomous/game-specific logic is being kept live for now,
not redesigned, but it no longer makes sense outside the context of a 2024
match and should be revisited once the robot's new purpose (demo / practice)
is decided in more detail.

## Non-goals

- No functional/behavioral changes beyond what's required to compile.
- No new features, no refactors, no abstraction cleanup.
- No attempt to make autonomous paths/robot config physically accurate (see
  RobotConfig note below) — that requires real measurements and the
  PathPlanner GUI, out of scope here.

## Root causes

1. **Vendordep deletions in progress.** `vendordeps/PathplannerLib.json` and
   `vendordeps/PathplannerLib-2026.1.2.json` are both staged for deletion, but
   `RobotContainer.java` and `SwerveSubsystem.java` still actively import and
   call into PathPlannerLib. `vendordeps/Phoenix6.json` (old) and
   `vendordeps/libgrapplefrc2024.json` are also staged for deletion; both are
   safe to leave deleted — Phoenix6 is superseded by
   `Phoenix6-26.3.0.json`, and the only `libgrapplefrc` (LaserCan) usage, in
   `Intake.java`, is already commented out.

2. **REVLib removed `CANSparkMax`.** REV renamed it to `SparkMax` and moved
   configuration (inversion, idle mode, encoder conversion factors, following)
   from direct setter calls to a `SparkMaxConfig` object passed to
   `.configure(...)`. This affects `SwerveModule.java` (2 motors) and
   `Tilt.java` (2 motors).

3. **PathPlannerLib removed the 2024 AutoBuilder API.**
   `AutoBuilder.configureHolonomic(...)`, `HolonomicPathFollowerConfig`,
   `PIDConstants`, and `ReplanningConfig` (from `com.pathplanner.lib.util`)
   no longer exist. The current API is `AutoBuilder.configure(...)` taking a
   `PPHolonomicDriveController` and a `RobotConfig` (normally loaded via
   `RobotConfig.fromGUISettings()`, which reads
   `src/main/deploy/pathplanner/settings.json`). That settings file does not
   currently exist in this repo, so a reasonable placeholder `RobotConfig`
   will be constructed in code and flagged with a TODO — it won't reflect the
   robot's real mass/MOI/module layout until someone re-exports it from the
   PathPlanner GUI.

4. **Phoenix5 (`VictorSPX`) and Phoenix6 `TalonFX`/`Orchestra`/`Follower`
   usage already matches the current API** (confirmed the
   `Follower(int, MotorAlignmentValue)` constructor used in `Shooter.java` is
   the current 2026 signature, not a leftover) — verified by compiling, not
   assumed.

5. **WPILib-core and Phoenix6 sensor APIs also changed**, found only by
   compiling against the real jars rather than reading migration docs:
   - `Pigeon2.getAngle()` (used in `SwerveSubsystem.getGyroHeading()`) no
     longer exists. `CorePigeon2.getYaw()` — a `StatusSignal<Angle>`, the
     same pattern already used elsewhere in this file for `CANcoder` — is the
     direct replacement (`pidgy.getYaw().refresh().getValueAsDouble()`).
   - `DutyCycleEncoder.getDistance()` (used in `Tilt.getTiltAngle()`) no
     longer exists; `get()` is the replacement. Since this code never called
     the old `setDistancePerRotation()`, `getDistance()` was numerically
     identical to `get()` (rotations, unscaled) — so swapping the method name
     preserves behavior exactly.
   - `DutyCycleEncoder.reset()` (used in `Tilt.resetTiltEncoder()`) was
     removed with **no replacement** in the current API. Per the "leave
     comments on things that don't make sense out of context of a match"
     approach, this becomes a no-op with a TODO rather than an invented
     substitute — the "Reset Tilt Encoder" dashboard button currently does
     nothing.

## Design

### 1. Vendordeps
Restore `vendordeps/PathplannerLib-2026.1.2.json` (`git checkout` the staged
deletion). Leave `PathplannerLib.json` (duplicate/older), `Phoenix6.json`, and
`libgrapplefrc2024.json` deleted.

### 2. REVLib migration (`SwerveModule.java`, `Tilt.java`)
Mechanical rename `CANSparkMax` → `SparkMax`; replace direct setter calls
(`setInverted`, `setIdleMode`, `driveEncoder.setPositionConversionFactor`,
etc.) with an equivalent `SparkMaxConfig`, applied via
`motor.configure(config, ResetMode.kResetSafeParameters,
PersistMode.kPersistParameters)`. Use the top-level `com.revrobotics.ResetMode`
/ `com.revrobotics.PersistMode` enums, not the `SparkBase`-nested ones — both
exist and both compile, but the nested ones are deprecated-for-removal.
Following is also config-only now: `rightConfig.follow(leftMotor, true)`
replaces the old direct `rightMotor.follow(leftMotor, true)` instance call.
No behavioral change — same idle modes, same inversions, same conversion
factors, same PID values, same follower-invert relationship.

### 3. PathPlannerLib migration (`SwerveSubsystem.java`)
Replace `AutoBuilder.configureHolonomic(...)` with `AutoBuilder.configure(...)`
using `new PPHolonomicDriveController(translationPID, rotationPID)` (same PID
constants as today: 1.7/0/0 translation, 0.3/0/0 rotation) and a `RobotConfig`
built with a placeholder mass/MOI/module config, marked with a TODO to
re-export from the PathPlanner GUI. `RobotContainer.java`'s
`NamedCommands`/`PathPlannerAuto` usage is unaffected by this API change and
needs no code changes — only marked with TODOs where the names are
2024-game-specific (e.g. `"Shoot Speaker"`, `"Tilt To Amp"`).

### 4. Sensor/gyro API fixes (`SwerveSubsystem.java`, `Tilt.java`)
`pidgy.getAngle()` → `pidgy.getYaw().refresh().getValueAsDouble()` (same sign,
same continuous-degrees value, matching the `StatusSignal` pattern already
used for `CANcoder` in this file). `tiltEncoder.getDistance()` → `get()`
(numerically identical here, since distance-per-rotation was never
configured). `tiltEncoder.reset()` has no replacement — turned into a no-op
with a TODO rather than an invented substitute.

`Intake.java` (Phoenix5) and `Shooter.java` (Phoenix6 `TalonFX`/`Follower`)
needed no changes — confirmed by building, not assumed.

### 5. TODO-comment pass
After a clean build, one pass through the changed/reviewed files adding
`// TODO:` comments (no logic changes) on code that is meaningful only in the
context of a 2024 match: named auto commands tied to Crescendo scoring
(`RobotContainer.registerNamedCommands`), `TiltAimToSpeaker`, amp/speaker
setpoints in `Tilt.java` and `RobotContainer`'s copilot bindings, and the
placeholder `RobotConfig` from step 3.

## Verification

`./gradlew compileJava` was run after each subsystem's migration (network
access to CTRE/REV/PathPlanner Maven repos is available from this
environment), confirming each step against the real 2026 vendor JARs rather
than docs alone — this is how the sensor API breaks in root cause 5 were
actually found, after web searches and doc fetches gave inconsistent or
incomplete answers. `./gradlew build` (full build, including the `test` task)
passes with **BUILD SUCCESSFUL** as of this writing.

## Testing

No unit tests exist for hardware-dependent subsystems (expected for FRC robot
code — these classes talk directly to CAN devices). Verification here is
compilation only; on-robot behavioral testing is out of scope for this task
and would happen separately once deployed to real hardware.
