package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.rebuilt.commands.LauncherCommands;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Launcher subsystem. */
public interface LauncherIO {

  @AutoLog
  public static class LauncherIOInputs {
    public LauncherCommands.LauncherState stateRequested = LauncherCommands.LauncherState.IDLE;

    public LauncherCommands.LauncherState stateCurrent = LauncherCommands.LauncherState.IDLE;
    public LauncherCommands.LauncherState preTrenchState = LauncherCommands.LauncherState.IDLE;

    public boolean isValidCalculation = false;

    public Distance distanceToVirtualTarget = Meters.of(0.0);
    public AngularVelocity flyWheelSpeedDesired = RPM.of(0.0);

    public AngularVelocity flyWheelSpeedCalculated = RotationsPerSecond.of(0.0);

    public Angle hoodAngleCalculated = Degrees.of(0.0);
    public Angle turretAngleCalculated = Degrees.of(0.0);
    public Angle hoodAngleDesired = Degrees.of(0.0);
    public Angle turretAngleDesired = Degrees.of(0.0);

    public AngularVelocity flyWheelSpeedActual = RPM.of(0.0);

    public Angle hoodAngleActual = Degrees.of(0.0);
    public Angle turretAngleActual = Degrees.of(0.0);

    public boolean flyWheelSpeedAtGoal = false;

    public boolean hoodAngleAtGoal = false;
    public boolean turretAngleAtGoal = false;

    public AngularVelocity flyWheelSpeedError = RPM.of(0.0);

    public double hoodAngleError = 0.0;
    public double turretAngleError = 0.0;

    public double hoodVelocity = 0.0;

    public boolean hoodMoving = true;

    public double turretVelocity = 0.0;
    public double flyWheelMotorOutput = 0.0;

    /** Kinematic feedforward for the turret (rad/s), computed via numerical differentiation. */
    public double turretFeedforwardRadPerSec = 0.0;

    /**
     * Acceleration feedforward for the turret (rad/s^2), computed via numerical differentiation.
     */
    public double turretFeedforwardAccelRadPerSecSq = 0.0;

    public Translation2d robotToTarget = new Translation2d();

    public Distance targetDistance = Meters.of(0.0);

    public Angle uniqueCoverage = Degrees.of(0.0);
    public boolean coverageSatisfiesRange = false;
  }

  public default void updateInputs(LauncherIOInputs inputs) {}

  public void runShooter(double speed);

  public void setFlyWheelVelocity(AngularVelocity speed);

  public void setHoodAngle(Angle angle);

  public void setHoodAngleLow();

  public void setTurretRotation(Angle angle);

  public default void setTurretRotationWithFeedforward(
      Angle angle, double feedforwardRadPerSec, double accelerationRadPerSecSq) {
    setTurretRotation(angle);
  }

  public LinearVelocity getFlyWheelExitSpeed(AngularVelocity velocity);

  public Command getHoodCharacterizationCommand(SubsystemBase launcher);

  public Command getHoodSysIdCommand();

  public Command getHoodSysIdCommand(SubsystemBase launcher);

  public Command getTurretCharacterizationCommand(SubsystemBase launcher);

  public Command getTurretSysIdCommand();

  public Command getFlyWheelSysIdCommand(SubsystemBase launcher);

  public Command getFlyWheelSysIdCommand();

  public Command getTurretSysIdCommand(SubsystemBase launcher);

  public void runHoodDown();

  public void stopHood();

  public Boolean isHoodStalled();

  public void resetHoodAngle(Angle angle);

  public ShotCalculator.ShootingParameters getShootingParameters(
      Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d> targetPositionSupplier);

  public void stopAllMotors();

  public default void configureShotCalculator(ShotCalculator shotCalculator) {}

  public default SmartTurretController getSmartTurretController() {
    return null;
  }

  public default void updateSimulation(Launcher launcher, Indexer indexer) {}

  public boolean isNearTrench();

  public Optional<Translation2d> determineTarget();

  public default Command getTurretQuasistaticCommand(SubsystemBase launcher) {
    return Commands.none();
  }

  public default Command getTurretDynamicCommand(SubsystemBase launcher) {
    return Commands.none();
  }

  public default Command getTurretKsMapCommand(SubsystemBase launcher) {
    return Commands.none();
  }

  public default Command getTurretTrackingTuneCommand(SubsystemBase launcher) {
    return Commands.none();
  }

  public default Command getTurretSeekingTuneCommand(SubsystemBase launcher) {
    return Commands.none();
  }

  public void zeroTurret();

  public default boolean isTurretAtZero() {
    return false;
  }

  public default BooleanSupplier getTurretZeroButtonSupplier() {
    return () -> false;
  }
}
