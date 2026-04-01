// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.commands.LauncherCommands.LauncherState;
import java.util.Map;
import java.util.function.Supplier;
import org.frc5010.common.arch.GenericSubsystem;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;

public class Launcher extends GenericSubsystem {
  private final LauncherIO io;
  private final Arm hood;
  private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();
  public static Transform3d robotToTurret = new Transform3d();
  private Map<String, GenericSubsystem> subsystems;
  private SmartTurretController smartTurretController;

  private static final double PROFILE_PERIOD_SECONDS = 0.005; // 200 Hz

  /** Creates a new Launcher. */
  public Launcher(Map<String, GenericSubsystem> subsystems) {
    super("launcher.json");

    this.subsystems = subsystems;
    Pivot turret = (Pivot) devices.get("turret");
    hood = (Arm) devices.get("hood");
    robotToTurret =
        new Transform3d(
            turret.getPivotConfig().getMechanismPositionConfig().getRelativePosition().get(),
            new Rotation3d());
    /** Chooses the IO implimentation to be real or simulated */
    if (RobotBase.isSimulation()) {
      io = new LauncherIOSim(devices, subsystems);
    } else {
      io = new LauncherIOReal(devices, subsystems);
    }

    io.configureShotCalculator(ShotCalculator.getInstance());

    // Register the SmartTurretController's high-frequency stepping loop.
    // This runs at 200 Hz (5 ms) via a Notifier, evaluating state transitions and
    // sending control requests to the TalonFX.
    smartTurretController = io.getSmartTurretController();
    if (smartTurretController != null) {
      Notifier profileNotifier =
          new Notifier(() -> smartTurretController.step(PROFILE_PERIOD_SECONDS));
      profileNotifier.setName("SmartTurret");
      profileNotifier.startPeriodic(PROFILE_PERIOD_SECONDS);
    }
  }

  /**
   * Updates the inputs of the Launcher subsystem from the physical devices.
   *
   * <p>This method is called periodically by the GenericSubsystem class.
   */
  @Override
  public void periodic() {
    super.periodic();

    io.updateInputs(inputs);
    Logger.processInputs("Launcher", inputs);
  }

  /**
   * Called every time the scheduler runs while the robot is in simulation mode. Used to update
   * simulation models.
   */
  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
    io.updateSimulation(this, Rebuilt.indexer);
  }

  /**
   * Run the shooter at the given speed. This is a convenience method which simply calls
   * setUpperSpeed with the given speed.
   *
   * @param speed the speed to set the upper shooter motor to, in units of RPM.
   */
  public void runShooter(double speed) {
    io.runShooter(speed);
  }

  public void setHoodAngle(Angle angle) {
    io.setHoodAngle(angle);
  }

  public void setTurretRotation(Angle angle) {
    io.setTurretRotation(angle);
  }

  public boolean isShooting() {
    return inputs.stateCurrent == LauncherState.PREP || inputs.stateCurrent == LauncherState.PRESET;
  }

  public Command getHoodSysIdCommand() {
    return io.getHoodSysIdCommand(this);
  }

  public Command getTurretSysIdCommand() {
    return io.getTurretSysIdCommand(this);
  }

  public Command getFlyWheelSysIdCommand() {
    return io.getFlyWheelSysIdCommand(this);
  }

  public Command getHoodCharacterizationCommand() {
    return io.getHoodCharacterizationCommand(this);
  }

  public Command getTurretCharacterizationCommand() {
    return io.getTurretCharacterizationCommand(this);
  }

  public Command getTurretFFCharacterizationCommand() {
    return io.getTurretFFCharacterizationCommand(this);
  }

  public Command getTurretKsMapCommand() {
    return io.getTurretKsMapCommand(this);
  }

  public Command getTurretTrackingTuneCommand() {
    return io.getTurretTrackingTuneCommand(this);
  }

  public Translation2d getRobotTarget() {
    return io.determineTarget().get();
  }

  @Override
  public Command getDefaultCommand() {
    return Commands.runOnce(
        () -> {
          io.stopAllMotors();
        },
        this);
  }

  public void stopAllMotors() {
    io.stopAllMotors();
  }
  /** Command that aims the launcher using hood turret and flywheel values from calculations */
  public Command trackTargetCommand() {
    return Commands.run(
        () -> {
          io.setHoodAngle(inputs.hoodAngleCalculated);
          io.setTurretRotationWithFeedforward(
              inputs.turretAngleCalculated,
              inputs.turretFeedforwardRadPerSec,
              inputs.turretFeedforwardAccelRadPerSecSq);
          io.setFlyWheelVelocity(inputs.flyWheelSpeedCalculated);
        });
  }
  /** Aims the launcher using the preset hood angle and calculates flywheel and turret values */
  public Command trackTargetLowCommand() {
    return Commands.run(
        () -> {
          io.setHoodAngleLow();
          io.setTurretRotationWithFeedforward(
              inputs.turretAngleCalculated,
              inputs.turretFeedforwardRadPerSec,
              inputs.turretFeedforwardAccelRadPerSecSq);
          io.setFlyWheelVelocity(inputs.flyWheelSpeedCalculated);
        });
  }
  /** Aims the turret and sets the flywheel to a given speed */
  public Command trackTargetCommand(double speed) {
    return Commands.run(
        () -> {
          io.setHoodAngle(hood.getMotorController().getConfig().getMechanismLowerLimit().get());
          io.setTurretRotationWithFeedforward(
              inputs.turretAngleCalculated,
              inputs.turretFeedforwardRadPerSec,
              inputs.turretFeedforwardAccelRadPerSecSq);
          io.setFlyWheelVelocity(RPM.of(speed));
        });
  }

  /**
   * A command which stops the tracking of a target and resets the turret rotation and hood angle to
   * 0 degrees.
   *
   * @return a command which stops tracking and resets the turret rotation and hood angle.
   */
  public Command stopTrackingCommand() {
    return Commands.runOnce(
        () -> {
          setTurretRotation(Degrees.of(0));
          setHoodAngle(Constants.Launcher.LOW_HOOD_ANGLE);
          io.setFlyWheelVelocity(RPM.of(0));
        });
  }

  /**
   * Checks if the robot is at the desired speed and angle. This method returns true if the robot
   * speed and angle are within the allowed tolerances of the desired values.
   *
   * @return true if the robot is at the desired speed and angle, false otherwise.
   */
  public boolean isAtGoal() {
    return inputs.flyWheelSpeedAtGoal
        && inputs.hoodAngleAtGoal
        && inputs.turretAngleAtGoal
        && inputs.isValidCalculation;
  }

  public boolean isOKToFire() {
    return inputs.isValidCalculation;
  }

  public boolean isRequested(LauncherState state) {
    return inputs.stateRequested == state;
  }
  /** Checks if the current launcher matches the given state */
  public boolean isCurrent(LauncherState state) {
    return inputs.stateCurrent == state;
  }
  /** Updates the launcher's current state */
  public void setCurrentState(LauncherState state) {
    inputs.stateCurrent = state;
  }
  /** Sets the launcher's requested state to transition to */
  public void setRequestedState(LauncherState state) {
    inputs.stateRequested = state;
  }

  public LauncherState getCurrentState() {
    return inputs.stateCurrent;
  }

  public LauncherState getPreTrenchState() {
    return inputs.preTrenchState;
  }

  public boolean isNearTrench() {
    boolean nearTrench = io.isNearTrench();
    if (nearTrench && (getCurrentState() != LauncherState.AUTO_HAMMERTIME)) {
      inputs.preTrenchState = getCurrentState();
      if (getCurrentState() == LauncherState.PRESET) {
        inputs.preTrenchState = LauncherState.LOW_SPEED;
      }
    }
    return nearTrench;
  }

  /** Applies the hood and turret angle, and the flywheel speed */
  public void usePresets(Angle hoodAngle, Angle turretAngle, AngularVelocity flywheelSpeed) {
    io.setHoodAngle(hoodAngle);
    io.setTurretRotation(turretAngle);
    io.setFlyWheelVelocity(flywheelSpeed);
  }

  public ShotCalculator.ShootingParameters getShootingParameters(
      Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d> targetPositionSupplier) {
    return io.getShootingParameters(robotPoseSupplier, targetPositionSupplier);
  }

  // ---- Actual (measured) value getters for tuning/telemetry ----

  /** Get the actual hood angle as measured by the encoder. */
  public Angle getHoodAngleActual() {
    return inputs.hoodAngleActual;
  }

  /** Get the actual turret angle as measured by the encoder. */
  public Angle getTurretAngleActual() {
    return inputs.turretAngleActual;
  }

  /** Get the actual flywheel speed as measured by the encoder. */
  public AngularVelocity getFlywheelSpeedActual() {
    return inputs.flyWheelSpeedActual;
  }

  // ---- Desired (setpoint) value getters for tuning/telemetry ----

  /** Get the desired hood angle setpoint. */
  public Angle getHoodAngleDesired() {
    return inputs.hoodAngleDesired;
  }

  /** Get the desired turret angle setpoint. */
  public Angle getTurretAngleDesired() {
    return inputs.turretAngleDesired;
  }

  /** Get the desired flywheel speed setpoint. */
  public AngularVelocity getFlywheelSpeedDesired() {
    return inputs.flyWheelSpeedDesired;
  }

  // ---- Error and at-goal getters for tuning/telemetry ----

  /** Get the flywheel speed error (actual - desired). */
  public AngularVelocity getFlywheelSpeedError() {
    return inputs.flyWheelSpeedError;
  }

  /** Get the hood angle error in degrees (actual - desired). */
  public double getHoodAngleError() {
    return inputs.hoodAngleError;
  }

  /** Get the turret angle error in degrees (actual - desired). */
  public double getTurretAngleError() {
    return inputs.turretAngleError;
  }

  /** Whether the flywheel speed is within tolerance of the setpoint. */
  public boolean isFlywheelAtGoal() {
    return inputs.flyWheelSpeedAtGoal;
  }

  public boolean isFlywheelAtOrAboveGoal() {
    return inputs.flyWheelSpeedAtGoal || inputs.flyWheelSpeedActual.gt(inputs.flyWheelSpeedDesired);
  }

  /** Whether the hood angle is within tolerance of the setpoint. */
  public boolean isHoodAtGoal() {
    return inputs.hoodAngleAtGoal;
  }

  /** Whether the turret angle is within tolerance of the setpoint. */
  public boolean isTurretAtGoal() {
    return inputs.turretAngleAtGoal;
  }

  public Command increaseHoodAngleCommand() {
    return Commands.runOnce(
        () -> {
          Angle newAngle = inputs.hoodAngleActual.plus(Degrees.of(0.5));
          if (newAngle.lt(Degrees.of(60))) {
            io.setHoodAngle(newAngle);
          }
        });
  }
  /** Decreases the hood angle by 0.5 degrees and ensures it does not go below 30 degrees */
  public Command decreaseHoodAngleCommand() {
    return Commands.runOnce(
        () -> {
          Angle newAngle = inputs.hoodAngleActual.minus(Degrees.of(0.5));
          if (newAngle.gt(Degrees.of(30))) {
            io.setHoodAngle(newAngle);
          }
        });
  }
  /** Decreases the flywheel speed by 10 RPM and ensures it does not go below 0 RPM */
  public Command decreaseFlywheelSpeedCommand() {
    return Commands.runOnce(
        () -> {
          AngularVelocity newSpeed = inputs.flyWheelSpeedActual.minus(RPM.of(10));
          if (newSpeed.gt(RPM.of(0))) {
            io.setFlyWheelVelocity(newSpeed);
          }
        });
  }
  /** Increases the flywheel speed by 10 RPM and ensures it does not go above 300 RPM */
  public Command increaseFlywheelSpeedCommand() {
    return Commands.runOnce(
        () -> {
          AngularVelocity newSpeed = inputs.flyWheelSpeedActual.plus(RPM.of(10));
          if (newSpeed.lt(RPM.of(300))) { // Assuming 300 RPM as the upper limit
            io.setFlyWheelVelocity(newSpeed);
          }
        });
  }
  /** Decreases the turret angle by 10 degrees and ensures it does not go below -90 */
  public Command decreaseTurretAngleCommand() {
    return Commands.runOnce(
        () -> {
          Angle newAngle = inputs.turretAngleActual.minus(Degrees.of(1));
          if (newAngle.gt(Degrees.of(-90))) { // Assuming -90 degrees as the left limit
            io.setTurretRotation(newAngle);
          }
        });
  }
  /** Increases the turret angle by 1 degree and ensures it does not go above 90 degrees */
  public Command increaseTurretAngleCommand() {
    return Commands.runOnce(
        () -> {
          Angle newAngle = inputs.turretAngleActual.plus(Degrees.of(1));
          if (newAngle.lt(Degrees.of(90))) { // Assuming 90 degrees as the right limit
            io.setTurretRotation(newAngle);
          }
        });
  }
}
