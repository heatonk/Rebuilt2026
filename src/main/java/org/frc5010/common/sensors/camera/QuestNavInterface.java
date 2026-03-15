package org.frc5010.common.sensors.camera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.drive.pose.PoseProvider;

/** Add your docs here. */
public class QuestNavInterface implements PoseProvider {

  private String networkTableRoot = "questnav";
  private Supplier<ChassisSpeeds> robotVelocity = null;
  private Transform3d robotToQuest;
  private QuestNav questNav;
  private boolean initializedPosition = false;
  private PoseFrame latestPoseFrame = null;

  private ChassisSpeeds velocity;
  private Pose3d previousPose;
  private double previousTime;
  private static boolean hasHardReset = false;
  private static boolean initialReset = false;

  // Soft reset state — applied in code without sending a command to the headset
  private Transform3d softResetTransform = new Transform3d();

  // I guess this works
  private static Pose2d latestPoseState = null;

  private Translation2d _calculatedOffsetToRobotCenter = new Translation2d();
  private int _calculatedOffsetToRobotCenterCount = 0;

  public QuestNavInterface(Transform3d robotToQuest) {
    super();
    this.robotToQuest = robotToQuest;
    this.questNav = new QuestNav();
  }

  public QuestNavInterface(Transform3d robotToQuest, String networkTableRoot) {
    super();
    this.robotToQuest = robotToQuest;
    this.networkTableRoot = networkTableRoot;
    this.questNav = new QuestNav();
  }

  private Pose3d getRobotPoseFromQuestPose(Pose3d questPose) {
    return questPose.transformBy(robotToQuest.inverse());
  }

  private Pose3d getQuestPoseFromRobotPose(Pose3d robotPose) {
    return robotPose.transformBy(robotToQuest);
  }

  public void withRobotSpeedSupplier(Supplier<ChassisSpeeds> robotSpeed) {
    robotVelocity = robotSpeed;
  }

  /**
   * Returns the robot pose as computed directly from the QuestNav headset plus the robotToQuest
   * transform, with no soft-reset offset applied. This is the "hard-reset" pose layer.
   */
  private Pose3d getProcessedRobotPose() {
    if (latestPoseFrame == null) {
      return new Pose3d();
    }
    return getRobotPoseFromQuestPose(latestPoseFrame.questPose3d());
  }

  public Optional<Pose3d> getRobotPose() {
    if (RobotBase.isReal()) {
      if (latestPoseFrame == null) {
        return Optional.empty();
      }
      return Optional.of(getProcessedRobotPose().transformBy(softResetTransform));
    } else {
      return Optional.empty();
    }
  }

  public Rotation3d getRotation() {
    if (latestPoseFrame == null) {
      return new Rotation3d();
    }
    return getProcessedRobotPose().transformBy(softResetTransform).getRotation();
  }

  public Translation3d getPosition() {
    if (latestPoseFrame == null) {
      return new Translation3d();
    }
    return getProcessedRobotPose().transformBy(softResetTransform).getTranslation();
  }

  private void updateObservations() {
    PoseFrame[] unreadQuestFrames = questNav.getAllUnreadPoseFrames();
    if (unreadQuestFrames.length > 0) {
      latestPoseFrame = unreadQuestFrames[unreadQuestFrames.length - 1];
    }

    List<PoseObservation> observations = new ArrayList<>();

    if (initializedPosition) {
      for (PoseFrame frame : unreadQuestFrames) {
        Pose3d robotPose =
            getRobotPoseFromQuestPose(frame.questPose3d()).transformBy(softResetTransform);
        double captureTime = frame.dataTimestamp();
        observations.add(
            new PoseObservation(
                captureTime,
                robotPose,
                0,
                0,
                0,
                PoseObservationType.ENVIRONMENT_BASED,
                ProviderType.ENVIRONMENT_BASED));
      }
    }
    input.connected = isActive();
    // Save pose observations to inputs object
    input.poseObservations = new PoseObservation[observations.size()];
    for (int i = 0; i < observations.size(); i++) {
      input.poseObservations[i] = observations.get(i);
    }
  }

  @Override
  public Matrix<N3, N1> getStdDeviations(PoseObservation observation) {
    double calib = getConfidence();
    if (DriverStation.isDisabled()) {
      calib = 10000;
    }
    if (null != robotVelocity) {
      Translation2d questVelVector =
          new Translation2d(getVelocity().vxMetersPerSecond, getVelocity().vyMetersPerSecond);
      Translation2d robotVelVector =
          new Translation2d(
              robotVelocity.get().vxMetersPerSecond, robotVelocity.get().vyMetersPerSecond);
      if (Math.abs(questVelVector.getNorm() - robotVelVector.getNorm()) > 1.0) {
        calib = 10;
      }
    }
    return VecBuilder.fill(calib, calib, calib * 0.2);
  }

  public double getConfidence() {
    if (RobotBase.isReal()) {
      return 0.05;
    } else {
      return Double.MAX_VALUE;
    }
  }

  @Override
  public boolean isConnected() {
    return questNav.isConnected();
  }

  public boolean isActive() {
    boolean simulation = RobotBase.isSimulation();
    // boolean disabled = DriverStation.isDisabled();

    boolean isActive =
        questNav.isConnected()
            && (questNav.getFrameCount().orElse(0) > 0 && !simulation)
            && questNav.isTracking();

    return isActive && initializedPosition;
  }

  /**
   * Performs a soft reset: recalculates the pose offset in code without sending any command to the
   * Quest headset. This is instantaneous and avoids the delay of a hard reset command. The
   * soft-reset transform is stored and applied on top of the raw headset pose in {@link
   * #getRobotPose()}, {@link #getPosition()}, and {@link #getRotation()}.
   */
  public void softReset(Pose3d pose) {
    // Transform3d(from, to) gives exactly the relative transform needed:
    // processed.transformBy(softResetTransform) == pose
    softResetTransform = new Transform3d(getProcessedRobotPose(), pose);
    initializedPosition = true;
  }

  /**
   * Performs a hard reset: sends {@code questNav.setPose()} to the headset so its internal
   * coordinate origin is relocated. Intended to be called only once at the beginning of a match.
   * Subsequent resets during the match should use {@link #softReset(Pose3d)}.
   */
  public void hardReset(Pose3d pose) {
    if (isConnected()) {
      Pose3d questPose = getQuestPoseFromRobotPose(pose);
      questNav.setPose(questPose);
      // Clear any accumulated soft-reset so the hard reset is authoritative
      softResetTransform = new Transform3d();
      initializedPosition = true;
      hasHardReset = initialReset;
      initialReset = true;
    }
  }

  public static Trigger hasHardReset() {
    return new Trigger(() -> hasHardReset);
  }

  @Override
  public void resetPose(Pose3d pose) {
    if (isConnected()) {
      SmartDashboard.putBoolean(networkTableRoot + "/Reset Pose", true);
      softReset(pose);
    }
  }

  @Override
  public ProviderType getType() {
    return ProviderType.ENVIRONMENT_BASED;
  }

  public int fiducialId() {
    return 0;
  }

  private void updateVelocity() {
    return; // Implement if needed.
  }

  public ChassisSpeeds getVelocity() {
    if (null != velocity) {
      return velocity;
    }
    return new ChassisSpeeds();
  }

  @Override
  public void update() {
    if (RobotBase.isReal()) {
      questNav.commandPeriodic();
      updateVelocity();
      updateObservations();
      SmartDashboard.putBoolean(networkTableRoot + "/Reset Pose", false);
      SmartDashboard.putBoolean("QUEST Connected", isConnected());
      SmartDashboard.putBoolean("QUEST Active", isActive());

      Pose2d currPose = getRobotPose().orElse(new Pose3d()).toPose2d();
      latestPoseState = currPose;
      SmartDashboard.putNumberArray(
          networkTableRoot + "/Quest POSE Update",
          new double[] {currPose.getX(), currPose.getY(), currPose.getRotation().getDegrees()});

      ChassisSpeeds velocity = getVelocity();
      SmartDashboard.putNumberArray(
          networkTableRoot + "/Velocity",
          new double[] {
            velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, velocity.omegaRadiansPerSecond
          });
    }
    logInput(networkTableRoot);
  }

  private Translation2d calculateOffsetToRobotCenter() {
    if (null == latestPoseState) {
      return new Translation2d();
    }

    Pose2d currentPose2d = latestPoseState;

    Rotation2d angle = currentPose2d.getRotation();
    Translation2d displacement = currentPose2d.getTranslation();

    double x =
        ((angle.getCos() - 1) * displacement.getX() + angle.getSin() * displacement.getY())
            / (2 * (1 - angle.getCos()));
    double y =
        ((-1 * angle.getSin()) * displacement.getX() + (angle.getCos() - 1) * displacement.getY())
            / (2 * (1 - angle.getCos()));

    return new Translation2d(x, y);
  }

  public Command determineOffsetToRobotCenter(GenericDrivetrain drivetrain) {
    return Commands.repeatingSequence(
            Commands.run(
                    () -> {
                      SmartDashboard.putNumber("QUEST POSE", getPosition().getX());
                      drivetrain.drive(new ChassisSpeeds(0, 0, 0.314));
                    },
                    drivetrain)
                .withTimeout(3.0),
            Commands.runOnce(
                () -> {
                  // Update current offset
                  Translation2d offset = calculateOffsetToRobotCenter();

                  _calculatedOffsetToRobotCenter =
                      _calculatedOffsetToRobotCenter
                          .times(
                              (double) _calculatedOffsetToRobotCenterCount
                                  / (_calculatedOffsetToRobotCenterCount + 1))
                          .plus(offset.div(_calculatedOffsetToRobotCenterCount + 1));
                  _calculatedOffsetToRobotCenterCount++;

                  SmartDashboard.putNumberArray(
                      networkTableRoot + "/Quest Calculated Offset to Robot Center",
                      new double[] {
                        _calculatedOffsetToRobotCenter.getX(), _calculatedOffsetToRobotCenter.getY()
                      });
                }))
        .beforeStarting(
            Commands.runOnce(() -> resetPose(new Pose3d())).andThen(Commands.waitSeconds(1)));
  }
}
