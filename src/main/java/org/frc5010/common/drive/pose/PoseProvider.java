// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.pose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Arrays;
import java.util.List;
import org.frc5010.common.vision.VisionConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public interface PoseProvider {

  /**
   * Returns this provider's own {@link VisionIOInputsAutoLogged} instance.
   *
   * <p>Each implementing class must declare and return its own instance. The old pattern of a
   * single {@code public VisionIOInputsAutoLogged input} field on the interface was implicitly
   * {@code static final} in Java — meaning all cameras shared one object and each camera's {@code
   * updateCameraInfo()} overwrote the previous camera's observations.
   *
   * @return the per-instance inputs object
   */
  public VisionIOInputsAutoLogged getInput();

  public int getCameraIndex();

  public enum ProviderType {
    ALL,
    NONE,
    FIELD_BASED,
    ENVIRONMENT_BASED,
    RELATIVE
  }

  public enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION,
    ENVIRONMENT_BASED,
  }

  public enum PhotonPoseMethod {
    NONE,
    MULTITAG,
    TRIG
  }

  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public boolean hasTarget = false;
    public TargetRotation latestTargetRotation = new TargetRotation(new Rotation3d());
    public Pose3d latestTargetPose = new Pose3d();
    public double captureTime;
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
    /** Total summed distance to all visible tags (meters) */
    public double totalTagDistance = 0.0;
    /** Pose ambiguity of the best visible target */
    public double poseAmbiguity = 0.0;
    /** Latest estimated robot pose from this camera */
    public Pose3d estimatedRobotPose = new Pose3d();
    /** Count of unread PhotonVision frames returned this cycle */
    public int unreadResultCount = 0;
    /** Count of PhotonVision frames processed after capping */
    public int processedResultCount = 0;
    /** Count of PhotonVision frames dropped by the per-cycle cap */
    public int droppedResultCount = 0;
    /** Per-frame PhotonVision diagnostics for offline pose/filter tuning */
    public PhotonFrameObservation[] photonFrameObservations = new PhotonFrameObservation[0];
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetRotation(Rotation3d rotation) {}

  /** Logged raw pose-relevant data for a single tracked PhotonVision target. */
  public static record PhotonTargetObservation(
      int fiducialId,
      double yaw,
      double pitch,
      double area,
      double skew,
      double ambiguity,
      Transform3d bestCameraToTarget,
      Transform3d altCameraToTarget) {
    public static final PhotonTargetObservation EMPTY =
        new PhotonTargetObservation(
            -1, 0.0, 0.0, 0.0, 0.0, 0.0, new Transform3d(), new Transform3d());
  }

  /** Logged data for one pose-solve method from a PhotonVision frame. */
  public static record PhotonPoseEstimate(
      boolean present,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      int[] tagIds) {
    public static final PhotonPoseEstimate EMPTY =
        new PhotonPoseEstimate(false, new Pose3d(), 0.0, 0, 0.0, new int[0]);
  }

  /** Logged raw and solved data for a single PhotonVision frame. */
  public static record PhotonFrameObservation(
      double timestamp,
      double latencyMillis,
      long sequenceId,
      long publishTimestampMicros,
      int targetCount,
      double totalTagDistance,
      int[] multitagFiducialIds,
      Transform3d rawMultitagBestTransform,
      double rawMultitagAmbiguity,
      PhotonPoseMethod selectedMethod,
      PhotonPoseEstimate multiTagEstimate,
      PhotonPoseEstimate trigEstimate,
      PhotonTargetObservation[] targets) {
    public static final PhotonFrameObservation EMPTY =
        new PhotonFrameObservation(
            0.0,
            0.0,
            0L,
            0L,
            0,
            0.0,
            new int[0],
            new Transform3d(),
            0.0,
            PhotonPoseMethod.NONE,
            PhotonPoseEstimate.EMPTY,
            PhotonPoseEstimate.EMPTY,
            new PhotonTargetObservation[0]);
  }

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type,
      ProviderType provider) {}

  /*
   * Returns the current observations of the robot.
   *
   * @return The current observations of the robot.
   */
  public default List<PoseObservation> getObservations() {
    return Arrays.asList(getInput().poseObservations);
  }

  /**
   * Returns the raw observations array without wrapping — zero allocation per call.
   *
   * @return The current observations of the robot as a raw array.
   */
  public default PoseObservation[] getObservationsArray() {
    return getInput().poseObservations;
  }

  /*
   * Returns whether the pose provider is currently active. A camera could be
   * inactive if it is not currently tracking a target, for example.
   *
   * @return Whether the pose provider is currently active.
   */
  public default boolean isConnected() {
    return getInput().connected;
  }

  public default double getCaptureTime() {
    return getInput().captureTime;
  }

  public void update();

  public default void resetPose(Pose3d initPose) {}

  public ProviderType getType();

  public default void logInput(String tableName) {
    Logger.processInputs(VisionConstants.SBTabVisionDisplay + "/Camera " + tableName, getInput());
  }

  public default Matrix<N3, N1> getStdDeviations(PoseObservation observation) {
    // Calculate standard deviations
    double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
    double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;
    double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;
    if (observation.type() == PoseObservationType.MEGATAG_2) {
      linearStdDev *= VisionConstants.linearStdDevMegatag2Factor;
      angularStdDev *= VisionConstants.angularStdDevMegatag2Factor;
    }
    if (getCameraIndex() < VisionConstants.cameraStdDevFactors.length) {
      linearStdDev *= VisionConstants.cameraStdDevFactors[getCameraIndex()];
      angularStdDev *= VisionConstants.cameraStdDevFactors[getCameraIndex()];
    }

    return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
  }
}
