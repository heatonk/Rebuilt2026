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
import org.photonvision.targeting.PhotonTrackedTarget;

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
  /** Pre-allocated list reused each cycle to avoid per-cycle GC pressure */
  private final List<PhotonFrameObservation> photonFrameObservations = new ArrayList<>();
  /**
   * Pre-allocated output arrays — grown on demand but never shrunk to avoid per-cycle allocation
   */
  private PoseObservation[] poseObsArray = new PoseObservation[0];

  private PhotonFrameObservation[] photonFrameObsArray = new PhotonFrameObservation[0];

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
    photonFrameObservations.clear();

    super.updateCameraInfo();

    double totalTagDistanceAccum = 0.0;
    double latestAmbiguity = 0.0;
    Pose3d latestRobotPose = null;

    for (PhotonPipelineResult iCamResult : camResults) {
      Optional<EstimatedRobotPose> multiTagEstimate =
          poseEstimator.estimateCoprocMultiTagPose(iCamResult);
      Optional<EstimatedRobotPose> trigEstimate =
          poseEstimator.estimatePnpDistanceTrigSolvePose(iCamResult);

      Optional<EstimatedRobotPose> estimate = Optional.empty();
      PhotonPoseMethod selectedMethod = PhotonPoseMethod.NONE;
      if (multiTagEstimate.isPresent()) {
        estimate = multiTagEstimate;
        selectedMethod = PhotonPoseMethod.MULTITAG;
      } else if (trigEstimate.isPresent() && !DriverStation.isDisabled()) {
        estimate = trigEstimate;
        selectedMethod = PhotonPoseMethod.TRIG;
      }

      double totalTagDistance = 0.0;
      for (var iTarget : iCamResult.targets) {
        totalTagDistance += iTarget.bestCameraToTarget.getTranslation().getNorm();
      }
      double averageDistance =
          iCamResult.targets.isEmpty() ? 0.0 : totalTagDistance / iCamResult.targets.size();

      photonFrameObservations.add(
          new PhotonFrameObservation(
              iCamResult.getTimestampSeconds(),
              iCamResult.metadata.getLatencyMillis(),
              iCamResult.metadata.sequenceID,
              iCamResult.metadata.publishTimestampMicros,
              iCamResult.targets.size(),
              totalTagDistance,
              toMultitagIdArray(iCamResult),
              getRawMultitagBestTransform(iCamResult),
              getRawMultitagAmbiguity(iCamResult),
              selectedMethod,
              toPhotonPoseEstimate(multiTagEstimate, iCamResult, totalTagDistance, averageDistance),
              toPhotonPoseEstimate(trigEstimate, iCamResult, totalTagDistance, averageDistance),
              toPhotonTargetObservations(iCamResult.targets)));

      if (estimate.isPresent()) {
        EstimatedRobotPose estimatedRobotPose = estimate.get();
        Pose3d robotPose = estimatedRobotPose.estimatedPose;
        // Compute the average tag distance
        int tagCount = estimatedRobotPose.targetsUsed.size();

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

    int frameObsSize = photonFrameObservations.size();
    if (photonFrameObsArray.length != frameObsSize) {
      photonFrameObsArray = new PhotonFrameObservation[frameObsSize];
    }
    for (int i = 0; i < frameObsSize; i++) {
      photonFrameObsArray[i] = photonFrameObservations.get(i);
    }
    input.photonFrameObservations = photonFrameObsArray;

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

  private PhotonTargetObservation[] toPhotonTargetObservations(List<PhotonTrackedTarget> targets) {
    PhotonTargetObservation[] observations = new PhotonTargetObservation[targets.size()];
    for (int i = 0; i < targets.size(); i++) {
      PhotonTrackedTarget target = targets.get(i);
      observations[i] =
          new PhotonTargetObservation(
              target.fiducialId,
              target.yaw,
              target.pitch,
              target.area,
              target.skew,
              target.poseAmbiguity,
              target.bestCameraToTarget,
              target.altCameraToTarget);
    }
    return observations;
  }

  private PhotonPoseEstimate toPhotonPoseEstimate(
      Optional<EstimatedRobotPose> estimate,
      PhotonPipelineResult cameraResult,
      double totalTagDistance,
      double averageDistance) {
    if (estimate.isEmpty()) {
      return PhotonPoseEstimate.EMPTY;
    }

    EstimatedRobotPose estimatedRobotPose = estimate.get();
    return new PhotonPoseEstimate(
        true,
        estimatedRobotPose.estimatedPose,
        cameraResult.hasTargets() ? cameraResult.getBestTarget().poseAmbiguity : 0.0,
        estimatedRobotPose.targetsUsed.size(),
        averageDistance,
        toTargetIdArray(estimatedRobotPose.targetsUsed));
  }

  private int[] toMultitagIdArray(PhotonPipelineResult cameraResult) {
    if (cameraResult.multitagResult.isEmpty()) {
      return new int[0];
    }

    int[] ids = new int[cameraResult.multitagResult.get().fiducialIDsUsed.size()];
    for (int i = 0; i < ids.length; i++) {
      ids[i] = cameraResult.multitagResult.get().fiducialIDsUsed.get(i);
    }
    return ids;
  }

  private Transform3d getRawMultitagBestTransform(PhotonPipelineResult cameraResult) {
    if (cameraResult.multitagResult.isEmpty()) {
      return new Transform3d();
    }
    return cameraResult.multitagResult.get().estimatedPose.best;
  }

  private double getRawMultitagAmbiguity(PhotonPipelineResult cameraResult) {
    if (cameraResult.multitagResult.isEmpty()) {
      return 0.0;
    }
    return cameraResult.multitagResult.get().estimatedPose.ambiguity;
  }

  private int[] toTargetIdArray(List<PhotonTrackedTarget> targets) {
    int[] ids = new int[targets.size()];
    for (int i = 0; i < targets.size(); i++) {
      ids[i] = targets.get(i).fiducialId;
    }
    return ids;
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
