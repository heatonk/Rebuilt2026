package frc.robot.rebuilt.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.FieldConstants;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.commands.IntakeCommands.IntakeState;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import frc.robot.rebuilt.subsystems.Launcher.ShotCalculator;
import frc.robot.rebuilt.subsystems.Launcher.ShotCalculator.ShootingParameters;
import frc.robot.rebuilt.subsystems.drive.StubDrivetrain;
import frc.robot.rebuilt.subsystems.intake.Intake;
import frc.robot.rebuilt.util.AllianceFlipUtil;
import frc.robot.rebuilt.util.LedStrip;
import frc.robot.rebuilt.util.StateMachine;
import frc.robot.rebuilt.util.StateMachine.State;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.sensors.Controller;

/** defines commands and state launcher logic for the launcher */
public class LauncherCommands {

  private StateMachine stateMachine;
  private State idleState;
  private State lowState;
  private State prepState;
  private State presetState;
  private State hammerTimeState;
  private State autoHammerTimeState;
  private State escapeHammerTimeState;
  private static Launcher launcher;
  private static Intake intake;
  private static StubDrivetrain drivetrain;
  private Map<String, GenericSubsystem> subsystems;
  private static Translation2d hubTarget = FieldConstants.Hub.topCenterPoint.toTranslation2d();
  private static Translation2d allianceSideLeft = FieldConstants.Tower.leftUpright;
  private static Translation2d allianceSideRight = FieldConstants.Tower.rightUpright;
  private static Translation2d intakeToCenterTranslation =
      new Translation2d(Inches.of(25), Inches.of(0));
  private static Transform2d intakeToCenter =
      new Transform2d(intakeToCenterTranslation, Rotation2d.fromDegrees(180));
  private static Translation2d rearToCenterTranslation =
      new Translation2d(Inches.of(13.5), Inches.of(0));
  private static Transform2d rearToCenter =
      new Transform2d(rearToCenterTranslation, Rotation2d.fromDegrees(0));

  // Stored preset targets — written once when a preset command is activated
  private static Angle presetHoodAngle = Constants.Launcher.LOW_HOOD_ANGLE;
  private static Angle presetTurretAngle = Constants.Launcher.TURRET_FORWARD;
  private static AngularVelocity presetFlywheelSpeed = RPM.of(0);

  /**
   * When true, the indexer will only churn once the flywheel has reached its goal speed. Set to
   * false to allow churning at any time regardless of flywheel speed.
   */
  public static boolean requireFlywheelAtGoalForChurn = true;

  public static Translation2d getRobotToTarget(Translation2d target) {
    return target.minus(drivetrain.getPoseEstimator().getCurrentPose().getTranslation());
  }
  // public static Angle getHoodAngle(Distance toTarget) {} Placeholder for now
  /** declares possible states for the launcher */
  public static enum LauncherState {
    IDLE,
    LOW_SPEED,
    PREP,
    HAMMERTIME,
    AUTO_HAMMERTIME,
    ESCAPE_HAMMERTIME,
    PRESET;

    @Override
    public String toString() {
      return this.name();
    }
  }
  /** initializes the launcher state machine and adds states */
  public LauncherCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;
    launcher = Rebuilt.launcher;
    intake = Rebuilt.intake;
    launcher.setCurrentState(LauncherState.HAMMERTIME);
    launcher.setRequestedState(LauncherState.HAMMERTIME);

    drivetrain = Rebuilt.drivetrain;
    configureStateMachine();
  }
  /** sets the state machine as the default command of the launcher */
  public void setDefaultCommands() {
    if (launcher != null) {
      stateMachine.addRequirements(launcher);
      launcher.setDefaultCommand(stateMachine);
    }
  }

  public void configureStateMachine() {
    stateMachine = new StateMachine("LauncherStateMachine");
    presetState = stateMachine.addState("PRESET-SHOOT", presetStateCommand());
    idleState = stateMachine.addState("IDLE", idleStateCommand());
    lowState = stateMachine.addState("LOW-SPEED", lowStateCommand());
    prepState = stateMachine.addState("PREP-SHOOT", prepStateCommand());
    hammerTimeState = stateMachine.addState("HAMMER-TIME", hammerTimeStateCommand());
    autoHammerTimeState = stateMachine.addState("AUTO-HAMMER-TIME", autoHammerTimeStateCommand());
    escapeHammerTimeState =
        stateMachine.addState("ESCAPE-HAMMER-TIME", escapeHammerTimeStateCommand());
    stateMachine.setInitialState(hammerTimeState);
    idleState.switchTo(lowState).when(() -> launcher.isRequested(LauncherState.LOW_SPEED));
    idleState.switchTo(prepState).when(() -> launcher.isRequested(LauncherState.PREP));
    idleState.switchTo(presetState).when(() -> launcher.isRequested(LauncherState.PRESET));
    idleState.switchTo(hammerTimeState).when(() -> launcher.isRequested(LauncherState.HAMMERTIME));
    idleState
        .switchTo(autoHammerTimeState)
        .when(() -> launcher.isRequested(LauncherState.AUTO_HAMMERTIME));

    lowState.switchTo(idleState).when(() -> launcher.isRequested(LauncherState.IDLE));
    lowState.switchTo(prepState).when(() -> launcher.isRequested(LauncherState.PREP));
    lowState.switchTo(presetState).when(() -> launcher.isRequested(LauncherState.PRESET));
    lowState.switchTo(hammerTimeState).when(() -> launcher.isRequested(LauncherState.HAMMERTIME));
    lowState
        .switchTo(autoHammerTimeState)
        .when(() -> launcher.isRequested(LauncherState.AUTO_HAMMERTIME));

    prepState.switchTo(lowState).when(() -> launcher.isRequested(LauncherState.LOW_SPEED));
    prepState.switchTo(idleState).when(() -> launcher.isRequested(LauncherState.IDLE));
    prepState.switchTo(presetState).when(() -> launcher.isRequested(LauncherState.PRESET));
    prepState.switchTo(hammerTimeState).when(() -> launcher.isRequested(LauncherState.HAMMERTIME));
    prepState
        .switchTo(autoHammerTimeState)
        .when(() -> launcher.isRequested(LauncherState.AUTO_HAMMERTIME));

    presetState.switchTo(idleState).when(() -> launcher.isRequested(LauncherState.IDLE));
    presetState.switchTo(lowState).when(() -> launcher.isRequested(LauncherState.LOW_SPEED));
    presetState.switchTo(prepState).when(() -> launcher.isRequested(LauncherState.PREP));
    presetState
        .switchTo(hammerTimeState)
        .when(() -> launcher.isRequested(LauncherState.HAMMERTIME));
    presetState
        .switchTo(autoHammerTimeState)
        .when(() -> launcher.isRequested(LauncherState.AUTO_HAMMERTIME));

    autoHammerTimeState
        .switchTo(escapeHammerTimeState)
        .when(() -> launcher.isRequested(LauncherState.ESCAPE_HAMMERTIME));

    escapeHammerTimeState.switchTo(idleState).when(() -> launcher.isRequested(LauncherState.IDLE));
    escapeHammerTimeState
        .switchTo(lowState)
        .when(() -> launcher.isRequested(LauncherState.LOW_SPEED));
    escapeHammerTimeState.switchTo(prepState).when(() -> launcher.isRequested(LauncherState.PREP));

    // Hammer Time is a special case since it's a toggle state
    hammerTimeState.switchTo(lowState).when(() -> launcher.isRequested(LauncherState.LOW_SPEED));
    hammerTimeState.switchTo(prepState).when(() -> launcher.isRequested(LauncherState.PREP));
    hammerTimeState.switchTo(presetState).when(() -> launcher.isRequested(LauncherState.PRESET));

    Trigger readyToFireTrigger =
        new Trigger(() -> launcher.isCurrent(LauncherState.PREP) && launcher.isAtGoal());
    readyToFireTrigger.onTrue(IndexerCommands.shouldFeedCommand());
    Trigger churnWhileFiring =
        new Trigger(() -> launcher.isCurrent(LauncherState.PREP) && !launcher.isAtGoal());
    churnWhileFiring.onTrue(IndexerCommands.shouldChurnCommand());
    readyToFireTrigger
        .negate()
        .and(churnWhileFiring.negate())
        .onTrue(IndexerCommands.shouldIdleCommand());
  }

  /**
   * Returns true when it is safe to begin churning the indexer. When {@code
   * requireFlywheelAtGoalForChurn} is {@code true} (default), churning is only permitted once the
   * flywheel has reached its goal speed. Set the flag to {@code false} to allow churning at any
   * time regardless of flywheel speed.
   */
  public static boolean isFlywheelReadyForChurn() {
    return !requireFlywheelAtGoalForChurn
        || (launcher != null && launcher.isFlywheelAtOrAboveGoal());
  }

  public void configureButtonBindings(Controller driver, Controller operator) {

    // driver.createAButton().onTrue(shouldPrepCommand());
    driver.createBButton().whileTrue(shouldPrepCommand()).onFalse(shouldLowCommand());

    driver.createAButton().onTrue(shouldLowCommand()).onFalse(shouldLowCommand());

    operator
        .createLeftPovButton()
        .onTrue(Commands.runOnce(() -> ShotCalculator.incrementFlywheelMultiplier(-0.01)));
    operator
        .createRightPovButton()
        .onTrue(Commands.runOnce(() -> ShotCalculator.incrementFlywheelMultiplier(0.01)));

    operator.createAButton().whileTrue(towerPresetStateCommand()).onFalse(shouldLowCommand());

    operator
        .createBButton()
        .whileTrue(rightCornerPresetStateCommandr())
        .onFalse(shouldLowCommand());

    operator.createXButton().whileTrue(leftCornerPresetStateCommand()).onFalse(shouldLowCommand());
    operator
        .createYButton()
        .whileTrue(turretForwardPresetStateCommand())
        .onFalse(shouldLowCommand());

    operator.createBackButton().whileTrue(zeroHoodSequence());

    // This allowed auto-hammer time
    // Trigger isTrenchTrigger = new Trigger(() -> launcher.isNearTrench());
    // isTrenchTrigger.onTrue(shouldAutoHammerTimeCommand()).onFalse(shouldEscapeHammerTimeCommand());

    // When intake is retracting or retracted, force turret to HAMMERTIME (hopper arm interferes)
    Trigger intakeUpTrigger =
        new Trigger(
            () ->
                intake.isCurrent(IntakeState.RETRACTING)
                    || intake.isCurrent(IntakeState.RETRACTED));
    intakeUpTrigger.onTrue(shouldHammerTimeCommand());

    // When intake deploys away from turret, return to LOW_SPEED
    Trigger intakeDeployingTrigger =
        new Trigger(
            () ->
                ((intake.isCurrent(IntakeState.DEPLOYING)
                            && intake
                                .getHopperAngle()
                                .lt(
                                    Constants.Intake.HOPPER_RETRACTED_ANGLE.minus(
                                        Constants.Launcher.HOPPER_EXTENSION_BUFFER_BEFORE_AIM)))
                        || intake.isCurrent(IntakeState.INTAKING))
                    && launcher.isCurrent(LauncherState.HAMMERTIME));
    intakeDeployingTrigger.onTrue(shouldLowCommand());

    operator
        .createUpPovButton()
        .onTrue(
            Commands.runOnce(() -> ShotCalculator.incrementFlywheelMultiplier(0.01))
                .ignoringDisable(true));
    operator
        .createDownPovButton()
        .onTrue(
            Commands.runOnce(() -> ShotCalculator.incrementFlywheelMultiplier(-0.01))
                .ignoringDisable(true));
  }

  /** creates command behavior for the IDLE launcher state */
  private static Command idleStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              launcher.setCurrentState(LauncherState.IDLE);
            }),
        launcher.stopTrackingCommand());
  }
  /** creates command behavior for when the launcher is at low speed */
  private static Command lowStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              launcher.setCurrentState(LauncherState.LOW_SPEED);
              LedStrip.changeSegmentPattern(
                  LedStrip.ALL_LEDS, LedStrip.getSolidPattern(Color.kGreen));
            }),
        launcher.trackTargetLowCommand());
  }
  /** creates command behavior when the launcher is at prep state */
  private static Command prepStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              launcher.setCurrentState(LauncherState.PREP);
              LedStrip.changeSegmentPattern(
                  LedStrip.ALL_LEDS, LedStrip.getRainbowPattern(0));
            }),
        launcher.trackTargetCommand());
  }
  /** creates command behavior for when the launcher is at preset */
  private static Command presetStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              launcher.setCurrentState(LauncherState.PRESET);
            }),
        Commands.run(
            () -> launcher.usePresets(presetHoodAngle, presetTurretAngle, presetFlywheelSpeed)));
  }

  public static Command shouldIdleCommand() {
    return Commands.runOnce(() -> launcher.setRequestedState(LauncherState.IDLE));
  }

  public static Command shouldLowCommand() {
    return Commands.runOnce(() -> launcher.setRequestedState(LauncherState.LOW_SPEED));
  }

  public static Command shouldPrepCommand() {
    return Commands.runOnce(() -> launcher.setRequestedState(LauncherState.PREP));
  }

  public static Command shouldPresetCommand() {
    return Commands.runOnce(() -> launcher.setRequestedState(LauncherState.PRESET));
  }

  public static Command shouldAutoHammerTimeCommand() {
    return Commands.runOnce(() -> launcher.setRequestedState(LauncherState.AUTO_HAMMERTIME));
  }

  public static Command shouldEscapeHammerTimeCommand() {
    return Commands.runOnce(() -> launcher.setRequestedState(LauncherState.ESCAPE_HAMMERTIME));
  }

  public static Command shouldToggleHammerTimeCommand() {
    return Commands.runOnce(
        () -> {
          if (launcher.getCurrentState() == LauncherState.HAMMERTIME) {
            launcher.setRequestedState(LauncherState.LOW_SPEED);
          } else {
            launcher.setRequestedState(LauncherState.HAMMERTIME);
          }
        });
  }

  public static Command shouldHammerTimeCommand() {
    return Commands.runOnce(() -> launcher.setRequestedState(LauncherState.HAMMERTIME));
  }

  // Order is Hood Angle, Turret Angle, Flywheel Speed
  // Values are placeholders and need to be tuned
  public static Command leftCornerPresetStateCommand() {
    return shouldPresetCommand()
        .andThen(
            Commands.runOnce(
                () -> {
                  ShootingParameters params =
                      launcher.getShootingParameters(
                          () ->
                              AllianceFlipUtil.apply(
                                  new Pose2d(
                                      new Translation2d(
                                          Inches.of(
                                              FieldConstants.aprilTagFieldLayout
                                                      .getTagPose(31)
                                                      .get()
                                                      .getX()
                                                  + 25),
                                          FieldConstants.FIELD_WIDTH.minus(Inches.of(17.25))),
                                      new Rotation2d())),
                          () -> FieldConstants.Hub.topCenterPoint.toTranslation2d());
                  presetHoodAngle = Radians.of(params.hoodAngle());
                  presetTurretAngle = params.turretAngle().getMeasure();
                  presetFlywheelSpeed =
                      RPM.of(params.flywheelSpeed() * ShotCalculator.getFlywheelMultiplier());
                }));
  }

  public static Command rightCornerPresetStateCommandr() {
    return shouldPresetCommand()
        .andThen(
            Commands.runOnce(
                () -> {
                  ShootingParameters params =
                      launcher.getShootingParameters(
                          () ->
                              AllianceFlipUtil.apply(
                                  new Pose2d(
                                      new Translation2d(
                                          Inches.of(
                                              FieldConstants.aprilTagFieldLayout
                                                      .getTagPose(31)
                                                      .get()
                                                      .getX()
                                                  + 25),
                                          Inches.of(17.5)),
                                      new Rotation2d())),
                          () -> FieldConstants.Hub.topCenterPoint.toTranslation2d());
                  presetHoodAngle = Radians.of(params.hoodAngle());
                  presetTurretAngle = params.turretAngle().getMeasure();
                  presetFlywheelSpeed =
                      RPM.of(params.flywheelSpeed() * ShotCalculator.getFlywheelMultiplier());
                }));
  }

  public static Command towerPresetStateCommand() {
    return shouldPresetCommand()
        .andThen(
            Commands.runOnce(
                () -> {
                  ShootingParameters params =
                      launcher.getShootingParameters(
                          () ->
                              AllianceFlipUtil.apply(FieldConstants.Tower.face.plus(rearToCenter)),
                          () -> FieldConstants.Hub.topCenterPoint.toTranslation2d());
                  presetHoodAngle = Radians.of(params.hoodAngle());
                  presetTurretAngle = Constants.Launcher.TURRET_FORWARD;
                  presetFlywheelSpeed =
                      RPM.of(params.flywheelSpeed() * ShotCalculator.flywheelMultiplier);
                }));
  }

  public static Command turretForwardPresetStateCommand() {
    return shouldPresetCommand()
        .andThen(
            Commands.runOnce(
                () -> {
                  presetHoodAngle = Constants.Launcher.FWD_HOOD_ANGLE;
                  presetTurretAngle = Constants.Launcher.TURRET_FORWARD;
                  presetFlywheelSpeed = Constants.Launcher.FWD_FLYWHEEL_RPM;
                }));
  }

  public static Command hammerTimeStateCommand() {
    return Commands.parallel(
        Commands.run(
            () -> {
              launcher.setCurrentState(LauncherState.HAMMERTIME);
              launcher.usePresets(
                  Constants.Launcher.LOW_HOOD_ANGLE,
                  Degrees.of(0),
                  Constants.Launcher.LOW_FLYWHEEL_RPM);
            }));
  }

  public static Command autoHammerTimeStateCommand() {
    return Commands.parallel(
        Commands.run(
            () -> {
              launcher.setCurrentState(LauncherState.AUTO_HAMMERTIME);
              launcher.usePresets(
                  Constants.Launcher.LOW_HOOD_ANGLE,
                  Degrees.of(0),
                  Constants.Launcher.LOW_FLYWHEEL_RPM);
            }));
  }

  public static Command escapeHammerTimeStateCommand() {
    return Commands.runOnce(() -> launcher.setCurrentState(LauncherState.ESCAPE_HAMMERTIME))
        .andThen(
            Commands.runOnce(
                () -> {
                  launcher.setRequestedState(launcher.getPreTrenchState());
                }));
  }

  public static LauncherState getCurrentState() {
    return launcher.getCurrentState();
  }

  public static Command zeroHoodSequence() {
    return Commands.run(() -> launcher.runHoodDown())
        .withTimeout(Seconds.of(0.75))
        .andThen(Commands.runOnce(() -> launcher.stopHood()))
        .andThen(Commands.runOnce(() -> launcher.zeroHood()))
        .andThen(
            Commands.run(
                    () -> {
                      LedStrip.changeSegmentPattern(
                          LedStrip.ALL_LEDS, LedStrip.getRainbowPattern(50.0));
                    })
                .withTimeout(0.5)
                .ignoringDisable(true))
        .beforeStarting(() -> frc.robot.rebuilt.Rebuilt.isZeroingBurst = true)
        .finallyDo(
            () -> {
              frc.robot.rebuilt.Rebuilt.isZeroingBurst = false;
            });
  }
}
