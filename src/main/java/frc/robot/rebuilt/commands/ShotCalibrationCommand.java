package frc.robot.rebuilt.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.FieldConstants;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import frc.robot.rebuilt.subsystems.Launcher.ShotCalculator;
import frc.robot.rebuilt.util.AllianceFlipUtil;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.drive.swerve.GenericSwerveDrivetrain;

/**
 * A streamlined command to rapidly calibrate the shooter at various distances.
 *
 * <p>State Machine:
 *
 * <ol>
 *   <li><b>ALIGN_AND_DRIVE:</b> Automatically drives the robot to face the hub at
 *       `currentDistance`.
 *   <li><b>TUNE_AND_FIRE:</b> Populates dashboard with an initial guess, allows operator to tune
 *       RPM/Hood without bizarre scaling, and allows firing.
 *   <li><b>NEXT_DISTANCE:</b> Acknowledges operator confirmation, logs the tuned point, and backs
 *       up by an increment.
 * </ol>
 */
public class ShotCalibrationCommand extends Command {

  private enum CalibrationState {
    ALIGN_AND_DRIVE,
    TUNE_AND_FIRE,
    NEXT_DISTANCE
  }

  private static final String PREFIX = "ShotCal/";

  private final Launcher launcher;
  private final GenericSwerveDrivetrain drivetrain;
  private final ShotCalculator shotCalculator;

  private CalibrationState currentState = CalibrationState.ALIGN_AND_DRIVE;

  // Configuration
  private double currentDistanceMeters;
  private final double distanceStepMeters;

  // Controllers for physical robot alignment
  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  // Target coordinates
  private Translation2d hubTarget;
  private Pose2d targetPoseForDistance;

  // Tuning state block
  private boolean initialGuessPopulated = false;

  public ShotCalibrationCommand(
      Launcher launcher,
      GenericDrivetrain drivetrain,
      double initialDistance,
      double distanceStep) {
    this.launcher = launcher;
    this.drivetrain = (GenericSwerveDrivetrain) drivetrain;
    this.shotCalculator = ShotCalculator.getInstance();

    this.currentDistanceMeters = initialDistance;
    this.distanceStepMeters = distanceStep;

    // Initialize alignment controllers based on standard DriveToPosition constants
    xController = new ProfiledPIDController(2.0, 0, 0, new TrapezoidProfile.Constraints(0.2, 0.5));
    yController = new ProfiledPIDController(2.0, 0, 0, new TrapezoidProfile.Constraints(0.2, 0.5));
    thetaController =
        new ProfiledPIDController(3.0, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));

    xController.setTolerance(0.05); // 5cm
    yController.setTolerance(0.05); // 5cm
    thetaController.setTolerance(0.035); // ~2 degrees
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(launcher, this.drivetrain);
  }

  @Override
  public void initialize() {
    currentState = CalibrationState.ALIGN_AND_DRIVE;
    initialGuessPopulated = false;

    // Determine the hub position
    hubTarget = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

    // Setup dashboard fields
    SmartDashboard.putNumber(PREFIX + "Distance Step (m)", distanceStepMeters);
    SmartDashboard.putNumber(
        PREFIX + "Test Hood Angle", Constants.Launcher.offsetLegacyHoodAngleDegrees(35.0));
    SmartDashboard.putNumber(PREFIX + "Test Flywheel RPM", 1800.0);
    SmartDashboard.putNumber(PREFIX + "Flywheel Multiplier", 1.0);
    SmartDashboard.putBoolean(PREFIX + "Force Firing", false);
    SmartDashboard.putBoolean(PREFIX + "Confirm & Next", false);
    SmartDashboard.putBoolean(PREFIX + "Apply Guess", false);

    System.out.println("[ShotCalibration] Starting interactive calibration.");
  }

  @Override
  public void execute() {
    SmartDashboard.putString(PREFIX + "State", currentState.name());
    SmartDashboard.putNumber(PREFIX + "Current Target Dist", currentDistanceMeters);
    SmartDashboard.putNumber(PREFIX + "Actual Target Dist", getActualDistance());

    switch (currentState) {
      case ALIGN_AND_DRIVE:
        handleAlignAndDrive();
        break;

      case TUNE_AND_FIRE:
        handleTuneAndFire();
        break;

      case NEXT_DISTANCE:
        handleNextDistance();
        break;
    }
  }

  private void handleAlignAndDrive() {
    Pose2d currentPose = drivetrain.getPoseEstimator().getCurrentPose();

    // We want to be `currentDistanceMeters` away from the hub.
    // The easiest generic way to do this is to stand on a line drawn from the hub through our
    // current position.
    Translation2d hubToRobot = currentPose.getTranslation().minus(hubTarget);
    Rotation2d angleFromHub = hubToRobot.getAngle();

    // The target translation is the hub, plus a vector pointing towards the robot with length =
    // currentDistance
    Translation2d targetTranslation = hubTarget.plus(new Translation2d(currentDistanceMeters, 0.0));

    // We want the robot to face the hub. (Angle from hub + 180 deg)
    Rotation2d targetRotation = angleFromHub.plus(Rotation2d.fromDegrees(180));
    targetPoseForDistance = new Pose2d(targetTranslation, targetRotation);

    // Drive using controllers
    double xSpeed = xController.calculate(currentPose.getX(), targetPoseForDistance.getX());
    double ySpeed = yController.calculate(currentPose.getY(), targetPoseForDistance.getY());
    double thetaSpeed =
        thetaController.calculate(
            currentPose.getRotation().getRadians(),
            targetPoseForDistance.getRotation().getRadians());

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, thetaSpeed, currentPose.getRotation());
    drivetrain.drive(speeds);

    SmartDashboard.putNumber(PREFIX + "X Error", xController.getPositionError());
    SmartDashboard.putNumber(PREFIX + "Y Error", yController.getPositionError());
    SmartDashboard.putNumber(PREFIX + "Theta Error", thetaController.getPositionError());
    SmartDashboard.putBoolean(PREFIX + "X At Goal", xController.atGoal());
    SmartDashboard.putBoolean(PREFIX + "Y At Goal", yController.atGoal());
    SmartDashboard.putBoolean(PREFIX + "Theta At Goal", thetaController.atGoal());

    // Check transition
    if (xController.atGoal() && yController.atGoal() && thetaController.atGoal()) {
      drivetrain.drive(new ChassisSpeeds()); // stop
      currentState = CalibrationState.TUNE_AND_FIRE;
      initialGuessPopulated = false;
    }
  }

  private void handleTuneAndFire() {
    boolean applyGuess = SmartDashboard.getBoolean(PREFIX + "Apply Guess", false);

    // 1. Give an initial guess if we just arrived or if the operator requested it
    if (!initialGuessPopulated || applyGuess) {
      if (applyGuess) {
        SmartDashboard.putBoolean(PREFIX + "Apply Guess", false); // reset
      }

      double multiplier = SmartDashboard.getNumber(PREFIX + "Flywheel Multiplier", 1.0);
      double[] guess = shotCalculator.getBallisticGuess(currentDistanceMeters);

      if (false && guess != null) {
        SmartDashboard.putNumber(PREFIX + "Test Hood Angle", guess[0]);
        SmartDashboard.putNumber(PREFIX + "Test Flywheel RPM", guess[1] * multiplier);
      } else {
        // Fallback interpolation from lookup if ballistic isn't set up
        SmartDashboard.putNumber(
            PREFIX + "Test Hood Angle",
            shotCalculator.getLookupHoodAngleDegrees(currentDistanceMeters));
        SmartDashboard.putNumber(
            PREFIX + "Test Flywheel RPM",
            shotCalculator.getLookupFlywheelSpeed(currentDistanceMeters) * multiplier);
      }
      initialGuessPopulated = true;
    }

    // 2. Read explicit tuning values from operator. No funny business.
    double hoodSetpoint =
        SmartDashboard.getNumber(
            PREFIX + "Test Hood Angle", Constants.Launcher.offsetLegacyHoodAngleDegrees(35.0));
    double rpmSetpoint = SmartDashboard.getNumber(PREFIX + "Test Flywheel RPM", 1800.0);
    boolean fireRequested = SmartDashboard.getBoolean(PREFIX + "Force Firing", false);
    boolean nextRequested = SmartDashboard.getBoolean(PREFIX + "Confirm & Next", false);

    // 3. Apply literal assignments to hardware
    // Always zero turret for straightforward distances
    launcher.usePresets(Degrees.of(hoodSetpoint), Degrees.of(0), RPM.of(rpmSetpoint));

    // Telemetry feedback
    SmartDashboard.putNumber(PREFIX + "Actual Hood", launcher.getHoodAngleActual().in(Degrees));
    SmartDashboard.putNumber(PREFIX + "Actual RPM", launcher.getFlywheelSpeedActual().in(RPM));
    SmartDashboard.putBoolean(PREFIX + "Is At Goal", launcher.isAtGoal());

    // 4. Handle "Force Firing" integration -> we can't fully control the indexer natively inside a
    // standard command without parallel racing,
    // so we assume the operator maps a secondary button to IndexerCommands.shouldFeedCommand() to
    // actually shoot the pre-spun ball.
    // The "Force Firing" boolean here is mostly visual if they prefer button boards.

    // 5. Transition
    if (nextRequested) {
      SmartDashboard.putBoolean(PREFIX + "Confirm & Next", false); // reset

      // Log it!
      System.out.println(
          String.format(
              "[ShotCalibration] POINT LOGGED -> DISTANCE: %.2fm | HOOD: %.1f° | RPM: %.1f",
              getActualDistance(), hoodSetpoint, rpmSetpoint));

      // Save it into live memory to test immediately
      shotCalculator.addDataPoint(
          getActualDistance(), hoodSetpoint, rpmSetpoint, getActualDistance() / 15.0);

      currentState = CalibrationState.NEXT_DISTANCE;
    }
  }

  private void handleNextDistance() {
    // Increment the tracking parameter
    double step = SmartDashboard.getNumber(PREFIX + "Distance Step (m)", distanceStepMeters);
    currentDistanceMeters += step;

    // Loop back to driving state
    currentState = CalibrationState.ALIGN_AND_DRIVE;
  }

  private double getActualDistance() {
    Pose2d currentPose = drivetrain.getPoseEstimator().getCurrentPose();
    return currentPose.getTranslation().getDistance(hubTarget);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("[ShotCalibration] command ended. Interrupted: " + interrupted);
    drivetrain.drive(new ChassisSpeeds());
    launcher.stopAllMotors();
  }

  @Override
  public boolean isFinished() {
    return false; // Manually cancelled by operator when they're done with all distances
  }

  /**
   * Wrap the calibration command with an indexer force-feed so the operator can fire shots during
   * tuning.
   */
  public static Command createWithFeed(
      Launcher launcher,
      GenericDrivetrain drivetrain,
      double initialDistance,
      double distanceStep) {
    return Commands.parallel(
        new ShotCalibrationCommand(launcher, drivetrain, initialDistance, distanceStep),
        Commands.either(
            IndexerCommands.shouldForceCommand(),
            IndexerCommands.shouldChurnCommand(),
            () -> SmartDashboard.getBoolean(PREFIX + "Force Firing", false)));
  }
}
