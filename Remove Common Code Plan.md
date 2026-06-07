# Plan: Strip frc5010 common from rebuilt/ and stand on plain WPILib

## Context

The robot project at `Rebuilt2026` has ~237 frc5010 source files embedded directly under [src/main/java/org/frc5010/](src/main/java/org/frc5010/) and 29 files under [src/main/java/frc/robot/rebuilt/](src/main/java/frc/robot/rebuilt/) that import from them. The user wants those 29 files to stop depending on `org.frc5010.*` and run on stock WPILib (plus the vendordeps already present: PathPlanner, Phoenix6, REVLib, ThriftyLib, AdvantageKit, PhotonVision). The end state is: `grep -r "org.frc5010" src/main/java/frc/` returns zero matches, the project compiles, and the robot is deployable.

Per the user's choices:
- **Drivetrain:** stub it for now (no real swerve), defer replacement to a later effort.
- **Init:** plain Java constants + constructors. Drop JSON / `RobotsParser` config loading.
- **frc5010 tree:** leave it in place as dead code. Just stop importing it.
- **Delivery:** phased multi-PR migration; the build stays green at each phase boundary.

The hard parts: `Rebuilt extends GenericRobot`, five subsystems extend `GenericSubsystem`, and `RobotContainer` uses `RobotsParser` to dynamically instantiate the robot from JSON. Those are the loadbearing seams.

## Phase 1 — Local utility shims (low-risk, high-volume)

Goal: replace the leaf-level frc5010 helpers used scattered throughout `rebuilt/` with local `frc.robot.rebuilt.util.*` equivalents wrapping WPILib / vendor APIs. After this phase, no `rebuilt/` file imports an `org.frc5010.utils.*`, `org.frc5010.sensors.Controller`, or `org.frc5010.subsystems.LEDStrip`.

**New files (under `src/main/java/frc/robot/rebuilt/util/`):**
- `AllianceFlipUtil.java` — port the static API used at [Rebuilt.java:56](src/main/java/frc/robot/rebuilt/Rebuilt.java#L56), [ShotCalculator.java](src/main/java/frc/robot/rebuilt/subsystems/Launcher/ShotCalculator.java), [FieldRegions.java](src/main/java/frc/robot/rebuilt/subsystems/Launcher/FieldRegions.java), [LauncherCommands.java](src/main/java/frc/robot/rebuilt/commands/LauncherCommands.java), [LauncherIOReal.java](src/main/java/frc/robot/rebuilt/subsystems/Launcher/LauncherIOReal.java), [ShotCalibrationCommand.java](src/main/java/frc/robot/rebuilt/commands/ShotCalibrationCommand.java). Internally just uses `DriverStation.getAlliance()` + `Pose2d`/`Translation2d` flips against `FIELD_LENGTH`/`FIELD_WIDTH`.
- `GeomUtil.java` — only used by [ShotCalculator.java](src/main/java/frc/robot/rebuilt/subsystems/Launcher/ShotCalculator.java); copy just the method(s) actually called.
- `Controller.java` — joystick wrapper used in 16 places. Wrap `CommandXboxController` and expose only the `createXButton()` / `getRightYAxis()` / `setRightYAxis()` style methods the calling code actually uses (audit by grep).
- `LedStrip.java` — wrap WPILib `AddressableLED` + `AddressableLEDBuffer`. Match the 2-3 static methods called from [Rebuilt.java:89-96](src/main/java/frc/robot/rebuilt/Rebuilt.java#L89-L96), [LauncherCommands.java](src/main/java/frc/robot/rebuilt/commands/LauncherCommands.java), [IndexerCommands.java](src/main/java/frc/robot/rebuilt/commands/IndexerCommands.java), [LauncherIOReal.java](src/main/java/frc/robot/rebuilt/subsystems/Launcher/LauncherIOReal.java).
- `OrchestraManager.java` — used in [Rebuilt.java:80,129](src/main/java/frc/robot/rebuilt/Rebuilt.java#L80) only. Thin wrapper around CTRE `com.ctre.phoenix6.Orchestra` (or no-op stub if no Talons need to play music — confirm during execution).
- `AprilTagsHelper.java` — replace `org.frc5010.common.vision.AprilTags` calls in [FieldConstants.java](src/main/java/frc/robot/rebuilt/FieldConstants.java), [LauncherIOReal.java](src/main/java/frc/robot/rebuilt/subsystems/Launcher/LauncherIOReal.java), [LauncherCommands.java](src/main/java/frc/robot/rebuilt/commands/LauncherCommands.java) with direct `AprilTagFieldLayout.loadField(AprilTagFields.k20XXReefscape)` lookups. May not need a wrapper class at all — just inline.
- `StateMachine.java` — used by [ClimbCommands.java](src/main/java/frc/robot/rebuilt/commands/ClimbCommands.java), [IntakeCommands.java](src/main/java/frc/robot/rebuilt/commands/IntakeCommands.java), [LauncherCommands.java](src/main/java/frc/robot/rebuilt/commands/LauncherCommands.java), [Indexer.java](src/main/java/frc/robot/rebuilt/subsystems/Indexer/Indexer.java). Read the frc5010 implementation at `src/main/java/org/frc5010/common/...StateMachine.java` and reproduce the minimum API the 4 callers actually use. (Likely just `addState` + `transition` — check before reimplementing.)
- A `ConfigConstants` replacement: just inline its constants where used (it appears to be a string-key registry — replace `ConfigConstants.DRIVETRAIN` with a literal or local constant).

**Edits:** for each call site listed above, swap the import and adjust call signatures where the new API differs. No behavior change intended — purely a mechanical rename pass.

**Exit criteria for Phase 1:** `grep -r "org.frc5010.utils\|org.frc5010.sensors.Controller\|org.frc5010.subsystems.LEDStrip\|org.frc5010.common.vision.AprilTags" src/main/java/frc/` returns zero. Robot still extends GenericRobot, subsystems still extend GenericSubsystem — that's Phase 3. Build passes.

## Phase 2 — Drivetrain stub

Goal: kill the `GenericDrivetrain` / `GenericSwerveDrivetrain` / `DrivePoseEstimator` references in `rebuilt/`. Replace with a local stub subsystem so the rest of the code compiles and the robot is *runnable* (won't drive — that's expected per user direction).

**New file:** `src/main/java/frc/robot/rebuilt/subsystems/drive/StubDrivetrain.java extends SubsystemBase`. Audit every call to `drivetrain.*` in [Rebuilt.java](src/main/java/frc/robot/rebuilt/Rebuilt.java), [AutoCommands.java](src/main/java/frc/robot/rebuilt/commands/AutoCommands.java), [LauncherCommands.java](src/main/java/frc/robot/rebuilt/commands/LauncherCommands.java), [LauncherIOReal.java](src/main/java/frc/robot/rebuilt/subsystems/Launcher/LauncherIOReal.java), [IntakeIOReal.java](src/main/java/frc/robot/rebuilt/subsystems/intake/IntakeIOReal.java), [IntakeIOSim.java](src/main/java/frc/robot/rebuilt/subsystems/intake/IntakeIOSim.java), [ShotCalibrationCommand.java](src/main/java/frc/robot/rebuilt/commands/ShotCalibrationCommand.java) and provide a no-op or sensible-default for each: `getPose()` returns `Pose2d.kZero`, `toggleFieldOrientedDrive()` is a no-op, `createDefaultCommand(driver)` returns `Commands.none()`, `addAutoCommands(chooser)` is a no-op, `setAutoBuilder()` is a no-op, `generateAutoCommand(cmd)` returns `cmd`, `configureButtonBindings(...)` is a no-op, `getGyro()` returns a fake gyro or null-object. Add a `// TODO real swerve` marker on the class.

**Edits:** in [Rebuilt.java:38,62](src/main/java/frc/robot/rebuilt/Rebuilt.java#L38) change the field type from `GenericDrivetrain` to `StubDrivetrain`, construct it with `new StubDrivetrain()` instead of pulling from `subsystems.get(...)`. Repeat the type change in every command file that holds a drivetrain reference.

**Exit criteria for Phase 2:** zero `org.frc5010.common.drive.*` and zero `org.frc5010.common.sensors.gyro.*` imports in `rebuilt/`. Robot disables/enables cleanly in sim (no drive). Build passes.

## Phase 3 — Subsystem base class

Goal: switch `Intake`, `Indexer`, `Launcher`, `Climb`, `HubStatus` from `extends GenericSubsystem` to `extends SubsystemBase`. Drop the JSON device-loading machinery in favor of constructor args / constants.

**Edits per subsystem (template — apply to all 5):**
- Change `extends GenericSubsystem` → `extends SubsystemBase`.
- Replace `super("intake.json")` (or equivalent) with an empty default constructor.
- The IO-Real / IO-Sim subclasses currently take a `devices` map (the frc5010 device registry). Replace with explicit motor/encoder construction inline using vendor SDKs and constants from [Constants.java](src/main/java/frc/robot/rebuilt/Constants.java) (extend that file as needed). Example for [IntakeIOReal.java](src/main/java/frc/robot/rebuilt/subsystems/intake/IntakeIOReal.java): instead of pulling a `PercentControlMotor` out of `devices`, just `new TalonFX(Constants.Intake.SPINTAKE_CAN_ID)` and configure it inline.
- Drop the `subsystems` map plumbing from [Launcher.java](src/main/java/frc/robot/rebuilt/subsystems/Launcher/Launcher.java) (currently `new Launcher(subsystems)`) and from the command containers (`new TestCommands(subsystems)` etc.). They were using it to look up the drivetrain — now that's just a direct field reference on Rebuilt.

**Where the constants come from:** the frc5010 JSON files under `src/main/deploy/...` (or wherever the rebuilt_robot config lives — verify path during execution) define the CAN IDs, gear ratios, PID gains. Read those JSONs once, transcribe their values into [Constants.java](src/main/java/frc/robot/rebuilt/Constants.java) as static finals. After that, the JSON files become dead config — leave them or delete, user's call at the end.

**SystemIdentification (sysid) routines:** [LauncherIOReal.java](src/main/java/frc/robot/rebuilt/subsystems/Launcher/LauncherIOReal.java) and [IntakeIOReal.java](src/main/java/frc/robot/rebuilt/subsystems/intake/IntakeIOReal.java) use frc5010's `SystemIdentification` wrapper. Replace with WPILib's `edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine` directly — same concept, slightly different API. Sysid commands are referenced in [TurretDynamicCommand](src/main/java/frc/robot/rebuilt/commands/TurretDynamicCommand.java), [TurretQuasistaticCommand](src/main/java/frc/robot/rebuilt/commands/TurretQuasistaticCommand.java), [TurretKsMapCommand](src/main/java/frc/robot/rebuilt/commands/TurretKsMapCommand.java) — those will need their parameter types updated.

**Exit criteria for Phase 3:** zero `org.frc5010.common.arch.GenericSubsystem`, zero `org.frc5010.common.motors.*`, zero `org.frc5010.common.config.*` (except possibly `ConfigConstants` if it lingers — kill that too). Five subsystems extend `SubsystemBase`. Build passes.

## Phase 4 — Robot entry point

Goal: Rebuilt no longer extends GenericRobot; RobotContainer no longer uses RobotsParser.

**Edits to [Rebuilt.java](src/main/java/frc/robot/rebuilt/Rebuilt.java):**
- Drop `extends GenericRobot`. Becomes a plain class.
- Drop the `Rebuilt(String directory)` constructor signature; new constructor takes no args (or takes the controllers).
- Methods that were `@Override` from `GenericRobot` (`disabledInit`, `disabledPeriodic`, `configureButtonBindings`, `configureAltButtonBindings`, `setupDefaultCommands`, `setupAltDefaultCommands`, `initAutoCommands`, `buildAutoCommands`, `generateAutoCommand`, `getAutonomousCommand`) become plain public methods. The `Controller driver, Controller operator` parameters stay (they're now your local Controller from Phase 1).
- Replace the `subsystems.get(ConfigConstants.DRIVETRAIN)` lookup with direct `new StubDrivetrain()`.
- Replace `selectableCommand` (a frc5010 chooser) with a local `SendableChooser<Command>` field.
- Drop the `operator.isPresent()` block at [Rebuilt.java:73](src/main/java/frc/robot/rebuilt/Rebuilt.java#L73) — wire the operator controller in directly via the new constructor.
- Drop the `chooseAllianceWpiColor()` super-call at [Rebuilt.java:96](src/main/java/frc/robot/rebuilt/Rebuilt.java#L96) — reimplement as a 3-line local helper using `DriverStation.getAlliance()`.

**Edits to [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java):**
- Drop `implements WpiHelperInterface` and the `WpiNetworkTableValuesHelper` call.
- Drop the `RobotsParser` field and the `Constants` import (or replace with `frc.robot.rebuilt.Constants`).
- Construct `robot = new Rebuilt(...)` directly instead of `robotsParser.getRobot()`.
- Construct `Controller driver` and `Controller operator` here (from Phase 1's Controller class) and pass into Rebuilt's lifecycle methods.

**Robot.java**: should already be vanilla `LoggedRobot`, no changes expected. Verify during execution.

**Exit criteria for Phase 4:** [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java) and [Rebuilt.java](src/main/java/frc/robot/rebuilt/Rebuilt.java) have zero `org.frc5010.*` imports. `grep -r "org.frc5010" src/main/java/frc/` returns zero across the whole `frc.*` tree. Build passes, robot connects in sim, joystick buttons fire.

## Phase 5 — Verify and tidy

Goal: verify zero leakage, confirm runtime behavior, remove now-dead local code.

- Run `grep -rn "org.frc5010" src/main/java/frc/` — expect nothing.
- Run `./gradlew build` — expect green.
- Run sim (`./gradlew simulateJava`) — robot enables in teleop, all five subsystems instantiate, button bindings fire, no NPEs in disable→enable transitions. Manually toggle each driver/operator binding from the sim GUI.
- Inspect the unused JSON deploy configs (`rebuilt_robot/*.json`) — they're now dead since Phase 3. User decides whether to delete; not required for compilation.
- The `temp_yams/` folder appears unrelated/scratch — flag for user, don't touch.
- The `org.frc5010.*` tree under [src/main/java/org/frc5010/](src/main/java/org/frc5010/) stays as dead code per user direction. Optional: add a top-level `package-info.java` noting "deprecated, no longer used by frc.robot — do not re-import."

## Critical files to modify (summary)

Entry point:
- [src/main/java/frc/robot/RobotContainer.java](src/main/java/frc/robot/RobotContainer.java) — Phase 4
- [src/main/java/frc/robot/rebuilt/Rebuilt.java](src/main/java/frc/robot/rebuilt/Rebuilt.java) — Phase 1 (utils), Phase 2 (drivetrain field), Phase 4 (base class)

Subsystems (all five touched in Phase 3):
- [Launcher.java](src/main/java/frc/robot/rebuilt/subsystems/Launcher/Launcher.java) + [LauncherIO.java](src/main/java/frc/robot/rebuilt/subsystems/Launcher/LauncherIO.java) + [LauncherIOReal.java](src/main/java/frc/robot/rebuilt/subsystems/Launcher/LauncherIOReal.java) + [LauncherIOSim.java](src/main/java/frc/robot/rebuilt/subsystems/Launcher/LauncherIOSim.java) + [ShotCalculator.java](src/main/java/frc/robot/rebuilt/subsystems/Launcher/ShotCalculator.java) + [FieldRegions.java](src/main/java/frc/robot/rebuilt/subsystems/Launcher/FieldRegions.java)
- [Indexer.java](src/main/java/frc/robot/rebuilt/subsystems/Indexer/Indexer.java) + [IndexerIOReal.java](src/main/java/frc/robot/rebuilt/subsystems/Indexer/IndexerIOReal.java)
- [Intake.java](src/main/java/frc/robot/rebuilt/subsystems/intake/Intake.java) + [IntakeIO.java](src/main/java/frc/robot/rebuilt/subsystems/intake/IntakeIO.java) + [IntakeIOReal.java](src/main/java/frc/robot/rebuilt/subsystems/intake/IntakeIOReal.java) + [IntakeIOSim.java](src/main/java/frc/robot/rebuilt/subsystems/intake/IntakeIOSim.java)
- [Climb.java](src/main/java/frc/robot/rebuilt/subsystems/Climb/Climb.java)
- [HubStatus.java](src/main/java/frc/robot/rebuilt/subsystems/DriverDisplay/HubStatus.java)

Commands (all use Controller / GenericSubsystem / drivetrain — touched across all phases):
- [AutoCommands.java](src/main/java/frc/robot/rebuilt/commands/AutoCommands.java), [ClimbCommands.java](src/main/java/frc/robot/rebuilt/commands/ClimbCommands.java), [IndexerCommands.java](src/main/java/frc/robot/rebuilt/commands/IndexerCommands.java), [IntakeCommands.java](src/main/java/frc/robot/rebuilt/commands/IntakeCommands.java), [LauncherCommands.java](src/main/java/frc/robot/rebuilt/commands/LauncherCommands.java), [TestCommands.java](src/main/java/frc/robot/rebuilt/commands/TestCommands.java), [ShotCalibrationCommand.java](src/main/java/frc/robot/rebuilt/commands/ShotCalibrationCommand.java), [TurretDynamicCommand.java](src/main/java/frc/robot/rebuilt/commands/TurretDynamicCommand.java), [TurretQuasistaticCommand.java](src/main/java/frc/robot/rebuilt/commands/TurretQuasistaticCommand.java), [TurretKsMapCommand.java](src/main/java/frc/robot/rebuilt/commands/TurretKsMapCommand.java), [TurretSeekingTuneCommand.java](src/main/java/frc/robot/rebuilt/commands/TurretSeekingTuneCommand.java), [TurretTrackingTuneCommand.java](src/main/java/frc/robot/rebuilt/commands/TurretTrackingTuneCommand.java)

Util / constants:
- [src/main/java/frc/robot/rebuilt/FieldConstants.java](src/main/java/frc/robot/rebuilt/FieldConstants.java) — Phase 1 (AprilTags)
- [src/main/java/frc/robot/rebuilt/util/TorqueCurrentArmSupport.java](src/main/java/frc/robot/rebuilt/util/TorqueCurrentArmSupport.java) — Phase 3 (UnitsParser, YamsArmConfigurationJson)
- [src/main/java/frc/robot/rebuilt/Constants.java](src/main/java/frc/robot/rebuilt/Constants.java) — extended in Phase 3 to absorb all values formerly loaded from JSON

## Verification

Per-phase: `./gradlew build` must succeed at the end of each phase before opening the PR. `grep -c "org.frc5010" src/main/java/frc/**/*.java` should monotonically decrease, hitting zero at Phase 4.

End-to-end (after Phase 4):
1. `./gradlew build` — green.
2. `./gradlew simulateJava` — robot starts, no exceptions in logs.
3. In Driver Station sim: enable teleop → all five subsystems show in NetworkTables, no NPEs from `Rebuilt.disabledPeriodic` LED block, no NPEs from `setupDefaultCommands`.
4. Bind a button on the sim controller → confirm subsystem command runs (via AdvantageKit log or Glass).
5. Switch to autonomous → "Do Nothing" auto runs cleanly (the StubDrivetrain's auto-command pass-through returns whatever was selected).
6. Disable → re-enable → confirm Orchestra logic in `disabledInit`/`disabledPeriodic` doesn't crash (or is gracefully no-op'd).

Drivetrain motion is *not* expected to work after this migration — that's the explicit cost of the stub strategy and the trigger for a follow-on YAGSL-integration project.

## Out of scope (explicitly)

- Real swerve replacement (deferred — see follow-on YAGSL project).
- Deleting the `org.frc5010.*` source tree (left in place per user direction; can be deleted in a future cleanup once everyone's confident nothing reaches back into it).
- Migrating the existing JSON robot config files to a new format (they become dead artifacts after Phase 3; deletion is optional cleanup).
- Touching anything in [src/main/java/org/frc5010/](src/main/java/org/frc5010/).
