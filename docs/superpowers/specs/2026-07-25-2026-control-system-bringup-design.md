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

4. **Phoenix5 (`VictorSPX`) and Phoenix6 (`TalonFX`, `Pigeon2`, `CANcoder`,
   `Orchestra`, `Follower`) usage appears to already match the current API**
   (confirmed the `Follower(int, MotorAlignmentValue)` constructor used in
   `Shooter.java` is the current 2026 signature, not a leftover). These will
   be verified by compiling rather than assumed fixed.

## Design

### 1. Vendordeps
Restore `vendordeps/PathplannerLib-2026.1.2.json` (`git checkout` the staged
deletion). Leave `PathplannerLib.json` (duplicate/older), `Phoenix6.json`, and
`libgrapplefrc2024.json` deleted.

### 2. REVLib migration (`SwerveModule.java`, `Tilt.java`)
Mechanical rename `CANSparkMax` → `SparkMax`; replace direct setter calls
(`setInverted`, `setIdleMode`, `driveEncoder.setPositionConversionFactor`,
etc.) with an equivalent `SparkMaxConfig`, applied once via
`motor.configure(config, ResetMode.kResetSafeParameters,
PersistMode.kPersistParameters)`. `follow()` remains a direct instance call.
No behavioral change — same idle modes, same inversions, same conversion
factors, same PID values.

### 3. PathPlannerLib migration (`SwerveSubsystem.java`)
Replace `AutoBuilder.configureHolonomic(...)` with `AutoBuilder.configure(...)`
using `new PPHolonomicDriveController(translationPID, rotationPID)` (same PID
constants as today: 1.7/0/0 translation, 0.3/0/0 rotation) and a `RobotConfig`
built with a placeholder mass/MOI/module config, marked with a TODO to
re-export from the PathPlanner GUI. `RobotContainer.java`'s
`NamedCommands`/`PathPlannerAuto` usage is unaffected by this API change and
needs no code changes — only marked with TODOs where the names are
2024-game-specific (e.g. `"Shoot Speaker"`, `"Tilt To Amp"`).

### 4. Compile-and-verify the rest
`Intake.java` (Phoenix5) and `Shooter.java` (Phoenix6) are expected to need no
changes. Confirmed by building, not assumed — if `./gradlew compileJava`
surfaces errors here, fix only what the compiler flags.

### 5. TODO-comment pass
After a clean build, one pass through the changed/reviewed files adding
`// TODO:` comments (no logic changes) on code that is meaningful only in the
context of a 2024 match: named auto commands tied to Crescendo scoring
(`RobotContainer.registerNamedCommands`), `TiltAimToSpeaker`, amp/speaker
setpoints in `Tilt.java` and `RobotContainer`'s copilot bindings, and the
placeholder `RobotConfig` from step 3.

## Verification

`./gradlew compileJava` (network access to CTRE/REV/PathPlanner Maven repos is
available from this environment) run after each subsystem's migration, so
each step is confirmed against the real 2026 vendor JARs rather than docs
alone. A final `./gradlew build` confirms the whole project, including tests.

## Testing

No unit tests exist for hardware-dependent subsystems (expected for FRC robot
code — these classes talk directly to CAN devices). Verification here is
compilation only; on-robot behavioral testing is out of scope for this task
and would happen separately once deployed to real hardware.
