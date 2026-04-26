// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.frc5010.common.vision.VisionConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

/** A camera using the PhotonVision library. */
public class PhotonVisionPoseCamera extends PhotonVisionCamera implements FiducialTargetCamera {
  /** The pose estimator */
  protected PhotonPoseEstimator poseEstimator;
  /** The pose supplier */
  protected Supplier<Pose2d> poseSupplier;
  /** The current list of fiducial IDs */
  protected List<Integer> fiducialIds = new ArrayList<>();
  /** Pre-allocated list reused each cycle to avoid per-cycle GC pressure */
  private final List<PoseObservation> observations = new ArrayList<>();
  /** Pre-allocated set reused each cycle to avoid per-cycle GC pressure */
  private final Set<Short> tagIds = new HashSet<>();
  /**
   * Pre-allocated output arrays — grown on demand but never shrunk to avoid per-cycle allocation
   */
  private PoseObservation[] poseObsArray = new PoseObservation[0];

  private int[] tagIdArray = new int[0];

  /** Sentinel empty Pose3d — returned when no estimate is available; never mutated */
  private static final Pose3d EMPTY_POSE3D = new Pose3d();
  /** Pre-allocated stdDev matrix — mutated in getStdDeviations() to avoid per-call allocation */
  private final Matrix<N3, N1> stdDevMatrix = VecBuilder.fill(0.0, 0.0, 0.0);

  /**
   * Constructor
   *
   * @param name - the name of the camera
   * @param colIndex - the column index for the dashboard
   * @param fieldLayout - the field layout
   * @param cameraToRobot - the camera-to-robot transform
   * @param poseSupplier - the pose supplier
   */
  public PhotonVisionPoseCamera(
      String name,
      int colIndex,
      AprilTagFieldLayout fieldLayout,
      Transform3d cameraToRobot,
      Supplier<Pose2d> poseSupplier) {
    super(name, colIndex, cameraToRobot);
    this.poseSupplier = poseSupplier;
    this.fieldLayout = fieldLayout;
    poseEstimator = new PhotonPoseEstimator(fieldLayout, cameraToRobot);
  }

  public PhotonVisionPoseCamera(
      String name,
      int colIndex,
      AprilTagFieldLayout fieldLayout,
      Transform3d cameraToRobot,
      Supplier<Pose2d> poseSupplier,
      List<Integer> fiducialIds) {
    super(name, colIndex, cameraToRobot);
    this.poseSupplier = poseSupplier;
    this.fieldLayout = fieldLayout;

    this.fiducialIds = fiducialIds;
    visionLayout.addDouble("Observations", () -> input.poseObservations.length);
    List<AprilTag> filteredTags =
        fieldLayout.getTags().stream().filter(tag -> fiducialIds.contains(tag.ID)).toList();
    AprilTagFieldLayout filteredLayout =
        new AprilTagFieldLayout(
            filteredTags, fieldLayout.getFieldLength(), fieldLayout.getFieldWidth());
    poseEstimator = new PhotonPoseEstimator(filteredLayout, cameraToRobot);
  }

  /** Update the camera and target with the latest result */
  @Override
  public void updateCameraInfo() {
    poseEstimator.addHeadingData(Timer.getFPGATimestamp(), poseSupplier.get().getRotation());

    observations.clear();
    tagIds.clear();

    super.updateCameraInfo();

    double totalTagDistanceAccum = 0.0;
    double latestAmbiguity = 0.0;
    Pose3d latestRobotPose = null;

    for (PhotonPipelineResult iCamResult : camResults) {
      Optional<EstimatedRobotPose> estimate = poseEstimator.estimateCoprocMultiTagPose(iCamResult);

      if (estimate.isEmpty() && !DriverStation.isDisabled()) {
        estimate = poseEstimator.estimatePnpDistanceTrigSolvePose(iCamResult);
      }

      if (estimate.isPresent()) {
        EstimatedRobotPose estimatedRobotPose = estimate.get();
        Pose3d robotPose = estimatedRobotPose.estimatedPose;

        double totalTagDistance = 0.0;
        for (var iTarget : iCamResult.targets) {
          totalTagDistance += iTarget.bestCameraToTarget.getTranslation().getNorm();
        }
        // Compute the average tag distance
        int tagCount = estimatedRobotPose.targetsUsed.size();
        double averageDistance = 0.0;
        if (!iCamResult.targets.isEmpty()) {
          averageDistance = totalTagDistance / iCamResult.targets.size();
        }

        // Add tag IDs — use ifPresent to avoid lambda allocation from .map()
        if (iCamResult.multitagResult.isPresent()) {
          tagIds.addAll(iCamResult.multitagResult.get().fiducialIDsUsed);
        }

        // Accumulate telemetry for AK logging
        totalTagDistanceAccum += totalTagDistance;
        latestAmbiguity = iCamResult.getBestTarget().poseAmbiguity;
        latestRobotPose = robotPose;

        observations.add(
            new PoseObservation(
                iCamResult.getTimestampSeconds(), // Timestamp
                // 3D pose estimate
                robotPose,
                iCamResult.getBestTarget().poseAmbiguity,
                tagCount,
                averageDistance,
                PoseObservationType.PHOTONVISION,
                ProviderType.FIELD_BASED));
      }
    }

    // Save pose observations to inputs object (once, outside the loop).
    // Grow the pre-allocated array only when the size increases; never shrink it.
    int obsSize = observations.size();
    if (poseObsArray.length != obsSize) {
      poseObsArray = new PoseObservation[obsSize];
    }
    for (int i = 0; i < obsSize; i++) {
      poseObsArray[i] = observations.get(i);
    }
    input.poseObservations = poseObsArray;

    // Save tag IDs to inputs object (once, outside the loop).
    int tagCount2 = tagIds.size();
    if (tagIdArray.length != tagCount2) {
      tagIdArray = new int[tagCount2];
    }
    int i = 0;
    for (int id : tagIds) {
      tagIdArray[i++] = id;
    }
    input.tagIds = tagIdArray;
    // Log telemetry fields via AdvantageKit inputs
    input.totalTagDistance = totalTagDistanceAccum;
    input.poseAmbiguity = latestAmbiguity;
    input.estimatedRobotPose = latestRobotPose != null ? latestRobotPose : EMPTY_POSE3D;
  }

  @Override
  public Matrix<N3, N1> getStdDeviations(PoseObservation observation) {
    double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
    double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;

    double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;
    if (camResult.multitagResult.isEmpty()) {
      angularStdDev = 1.0;
    }
    // double rotStdDev = 0.3;

    // If really close, disregard angle measurement
    if (observation.averageTagDistance() < 0.3
        || (observation.averageTagDistance() > 2 && RobotState.isEnabled())) {
      angularStdDev = 1000.0;
    }

    if ((observation.averageTagDistance() > 2.5
        && RobotState.isEnabled()
        && observation.tagCount() < 2)) {
      linearStdDev = 100.0;
    }
    stdDevMatrix.set(0, 0, linearStdDev);
    stdDevMatrix.set(1, 0, linearStdDev);
    stdDevMatrix.set(2, 0, angularStdDev);
    return stdDevMatrix;
  }

  /**
   * Gets the current list of fiducial IDs for this camera.
   *
   * @return the current list of fiducial IDs
   */
  public List<Integer> getFiducialIds() {
    return fiducialIds;
  }

  /**
   * Sets the list of fiducial IDs for this camera. The camera will only consider targets with IDs
   * in this list when locating targets. This does not change the list spefified at construction
   * time to the pose camera so that the camera can be both a pose and target camera.
   *
   * @param fiducialIds the list of fiducial IDs to consider
   */
  public void setFiducialIds(List<Integer> fiducialIds) {
    this.fiducialIds = fiducialIds;
  }
}
