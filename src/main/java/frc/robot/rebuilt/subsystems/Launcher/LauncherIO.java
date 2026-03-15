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
import frc.robot.rebuilt.commands.LauncherCommands;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import java.util.Optional;
import java.util.function.Supplier;
import org.frc5010.common.arch.GenericSubsystem;
import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Launcher subsystem. */
public interface LauncherIO {

  @AutoLog
  public static class LauncherIOInputs {
    /** Initializes the requested and current launcher states to idle */
    public LauncherCommands.LauncherState stateRequested = LauncherCommands.LauncherState.IDLE;

    public LauncherCommands.LauncherState stateCurrent = LauncherCommands.LauncherState.IDLE;
    public LauncherCommands.LauncherState preTrenchState = LauncherCommands.LauncherState.IDLE;
    /**
     * Intializes the distance to the virtual target and desired flywheel speed to start at 0 and
     * calculation validity to false
     */
    public boolean isValidCalculation = false;

    public Distance distanceToVirtualTarget = Meters.of(0.0);
    public AngularVelocity flyWheelSpeedDesired = RPM.of(0.0);
    /**
     * Intializes calculated and desired angles to 0 degrees and calculated flywheel speed to 0 RPS
     */
    public AngularVelocity flyWheelSpeedCalculated = RotationsPerSecond.of(0.0);

    public Angle hoodAngleCalculated = Degrees.of(0.0);
    public Angle turretAngleCalculated = Degrees.of(0.0);
    public Angle hoodAngleDesired = Degrees.of(0.0);
    public Angle turretAngleDesired = Degrees.of(0.0);
    /** Initializes actual flywheel speed to 0 RPM and actual hood and turret angles to 0 degrees */
    public AngularVelocity flyWheelSpeedActual = RPM.of(0.0);

    public Angle hoodAngleActual = Degrees.of(0.0);
    public Angle turretAngleActual = Degrees.of(0.0);
    /** Decides whether the flywheel speed and turret and hood angle have reached their goals */
    public boolean flyWheelSpeedAtGoal = false;

    public boolean hoodAngleAtGoal = false;
    public boolean turretAngleAtGoal = false;
    /** Initializes the hood and turret angle errors to 0 and the flywheel speed error to 0 RPM */
    public AngularVelocity flyWheelSpeedError = RPM.of(0.0);

    public double hoodAngleError = 0.0;
    public double turretAngleError = 0.0;
    /** Intiializes the hood and turret velocities to 0 and the flywheel motor output to 0 */
    public double hoodVelocity = 0.0;

    public double turretVelocity = 0.0;
    public double flyWheelMotorOutput = 0.0;

    /** Kinematic feedforward for the turret from the aiming solver (rad/s, field-relative). */
    public double turretFeedforwardRadPerSec = 0.0;

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

  /**
   * Sets the turret to the given angle while simultaneously applying a kinematic feedforward
   * velocity. The feedforward is passed directly to the underlying motor controller's closed-loop
   * request so the controller does not need to derive it from the position error alone.
   *
   * @param angle desired turret mechanism angle
   * @param feedforwardRadPerSec angular velocity feedforward in rad/s (mechanism units)
   */
  public default void setTurretRotationWithFeedforward(Angle angle, double feedforwardRadPerSec) {
    setTurretRotation(angle);
  }

  public LinearVelocity getFlyWheelExitSpeed(AngularVelocity velocity);

  public Command getHoodCharacterizationCommand(GenericSubsystem launcher);

  public Command getHoodSysIdCommand();

  public Command getHoodSysIdCommand(GenericSubsystem launcher);

  public Command getTurretCharacterizationCommand(GenericSubsystem launcher);

  public Command getTurretSysIdCommand();

  public Command getFlyWheelSysIdCommand(GenericSubsystem launcher);

  public Command getFlyWheelSysIdCommand();

  public Command getTurretSysIdCommand(GenericSubsystem launcher);

  public ShotCalculator.ShootingParameters getShootingParameters(
      Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d> targetPositionSupplier);

  public void stopAllMotors();

  public default void configureShotCalculator(ShotCalculator shotCalculator) {}

  public default void updateSimulation(Launcher launcher, Indexer indexer) {}

  public boolean isNearTrench();

  public Optional<Translation2d> determineTarget();
}
