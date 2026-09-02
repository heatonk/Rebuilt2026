package frc.robot.rebuilt.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import swervelib.simulation.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

/**
 * Real swerve drivetrain backed by the CTRE Tuner X generated {@link CommandSwerveDrivetrain}.
 *
 * <p>Provides the rebuilt-side API surface that the rest of the code (Rebuilt.java, ShotCalculator,
 * LauncherIO*, IntakeIOSim, etc.) already calls. The CTRE SwerveRequest pattern is hidden behind a
 * few simple methods.
 */
public class RebuiltDrivetrain extends CommandSwerveDrivetrain {

  private static final double MAX_LINEAR_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final Telemetry telemetry = new Telemetry(MAX_LINEAR_SPEED);
  private final PoseEstimator poseEstimator = new PoseEstimator();

  private boolean fieldOriented = true;

  private CommandXboxController driver = null;
  private double lastVx = 0;
  private double lastVy = 0;
  private double lastOmega = 0;

  private final SwerveRequest.FieldCentric fieldCentricRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(MAX_LINEAR_SPEED * 0.1)
          .withRotationalDeadband(MAX_ANGULAR_RATE * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.RobotCentric robotCentricRequest =
      new SwerveRequest.RobotCentric()
          .withDeadband(MAX_LINEAR_SPEED * 0.1)
          .withRotationalDeadband(MAX_ANGULAR_RATE * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeedsRequest =
      new SwerveRequest.ApplyRobotSpeeds();

  public RebuiltDrivetrain() {
    super(
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight);
    registerTelemetry(telemetry::telemeterize);
  }

  public PoseEstimator getPoseEstimator() {
    return poseEstimator;
  }

  public void drive(ChassisSpeeds speeds) {
    setControl(applyRobotSpeedsRequest.withSpeeds(speeds));
  }

  public void toggleFieldOrientedDrive() {
    fieldOriented = !fieldOriented;
  }

  public void configureButtonBindings(
      CommandXboxController driver, CommandXboxController operator) {
    // No-op: matches the original stub. Button bindings can be wired here later
    // (brake / seedFieldCentric / sysid chords) once the team picks the layout.
  }

  public Command createDefaultCommand(CommandXboxController driver) {
    this.driver = driver;
    return applyRequest(
        () -> {
          double vx = -driver.getLeftY() * MAX_LINEAR_SPEED;
          double vy = -driver.getLeftX() * MAX_LINEAR_SPEED;
          double omega = -driver.getRightX() * MAX_ANGULAR_RATE;
          lastVx = vx;
          lastVy = vy;
          lastOmega = omega;
          return fieldOriented
              ? fieldCentricRequest.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega)
              : robotCentricRequest.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega);
        });
  }

  public void setAutoBuilder() {
    // No-op for now. Wire PathPlanner AutoBuilder.configure(...) here when ready;
    // see the pattern at org.frc5010.common.drive.swerve.GenericSwerveDrivetrain.
  }

  public Command generateAutoCommand(Command autoCommand) {
    return autoCommand;
  }

  public void addAutoCommands(LoggedDashboardChooser<Command> selectableCommand) {
    // No-op: PathPlanner-generated autos populate via AutoBuilder.buildAutoChooser()
    // in Rebuilt.buildAutoCommands once setAutoBuilder() is wired.
  }

  @Override
  public void periodic() {
    super.periodic();
    var state = getState();
    Logger.recordOutput("Drivetrain/Pose", state.Pose);
    Logger.recordOutput("Drivetrain/Speeds", state.Speeds);
    Logger.recordOutput("Drivetrain/ModuleStates", state.ModuleStates);
    Logger.recordOutput("Drivetrain/ModuleTargets", state.ModuleTargets);
    Logger.recordOutput("Drivetrain/OdometryPeriod", state.OdometryPeriod);
    Logger.recordOutput("Drivetrain/FailedDaqs", (long) state.FailedDaqs);
    Logger.recordOutput("Drivetrain/FieldOriented", fieldOriented);
    if (driver != null) {
      Logger.recordOutput("Drivetrain/Input/StickLeftX", driver.getLeftX());
      Logger.recordOutput("Drivetrain/Input/StickLeftY", driver.getLeftY());
      Logger.recordOutput("Drivetrain/Input/StickRightX", driver.getRightX());
    }
    Logger.recordOutput("Drivetrain/Input/CommandedVx", lastVx);
    Logger.recordOutput("Drivetrain/Input/CommandedVy", lastVy);
    Logger.recordOutput("Drivetrain/Input/CommandedOmega", lastOmega);
  }

  public ChassisSpeeds getFieldVelocity() {
    var state = getState();
    return ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
  }

  public ChassisSpeeds getFieldAcceleration() {
    // CTRE SwerveDrivetrain does not expose acceleration directly. Returning zero
    // matches stub behavior; estimate via finite differences if downstream math needs it.
    return new ChassisSpeeds();
  }

  public Field2d getField2d() {
    return telemetry.getField2d();
  }

  public static Optional<AbstractDriveTrainSimulation> getMapleSimDrive() {
    // CTRE has its own sim path via updateSimState; no maple-sim integration.
    return Optional.empty();
  }

  /** Source-compatible pose accessor for callers that used the stub's inner class. */
  public class PoseEstimator {
    public Pose2d getCurrentPose() {
      return getState().Pose;
    }

    public Pose3d getCurrentPose3d() {
      return new Pose3d(getState().Pose);
    }
  }
}
