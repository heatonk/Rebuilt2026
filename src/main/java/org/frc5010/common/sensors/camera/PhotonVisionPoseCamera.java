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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    List<PoseObservation> observations = new ArrayList<>();
    SmartDashboard.putBoolean("Camera/" + name() + "/updating", true);

    super.updateCameraInfo();
    Set<Short> tagIds = new HashSet<>();

    for (PhotonPipelineResult iCamResult : camResults) {
      SmartDashboard.putBoolean("Camera/" + name() + "/resuls", iCamResult.hasTargets());
      Optional<EstimatedRobotPose> estimate = poseEstimator.estimateCoprocMultiTagPose(iCamResult);

      if (estimate.isEmpty() && !DriverStation.isDisabled()) {
        estimate = poseEstimator.estimatePnpDistanceTrigSolvePose(iCamResult);
      }

      if (estimate.isPresent()) {
        // if (!DriverStation.isDisabled()) {
        //   Optional<EstimatedRobotPose> finalEstimate =
        //       poseEstimator.estimatePnpDistanceTrigSolvePose(iCamResult);
        //   if (finalEstimate.isPresent()) {
        //     estimate = finalEstimate;
        //   }
        // }

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

        // Add tag IDs
        iCamResult.multitagResult.map(it -> tagIds.addAll(it.fiducialIDsUsed));

        SmartDashboard.putNumber(
            "Camera/" + name() + "/Total Distance To Tag " + name, totalTagDistance);
        SmartDashboard.putNumber(
            "Camera/" + name() + "/Photon Ambiguity " + name,
            iCamResult.getBestTarget().poseAmbiguity);
        SmartDashboard.putNumberArray(
            "Camera/" + name() + "/Photon Camera " + name + " POSE",
            new double[] {
              robotPose.getX(),
              robotPose.getY(),
              robotPose.getRotation().toRotation2d().getDegrees()
            });
        SmartDashboard.putNumberArray(
            "Camera/" + name() + "/Photon Camera " + name + " Robot Offset",
            new double[] {
              robotToCamera.getX(),
              robotToCamera.getY(),
              robotToCamera.getRotation().toRotation2d().getDegrees()
            });

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

      // Save pose observations to inputs object
      input.poseObservations = new PoseObservation[observations.size()];
      for (int i = 0; i < observations.size(); i++) {
        input.poseObservations[i] = observations.get(i);
      }
      // Save tag IDs to inputs objects
      input.tagIds = new int[tagIds.size()];
      int i = 0;
      for (int id : tagIds) {
        input.tagIds[i++] = id;
      }
    }
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
    return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
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
