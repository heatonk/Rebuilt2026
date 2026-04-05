// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** A camera using the PhotonVision library. */
public class PhotonVisionCamera extends GenericCamera {
  /** The camera */
  protected PhotonCamera camera;
  /** The field layout */
  protected AprilTagFieldLayout fieldLayout;
  /** The target, if any */
  protected Optional<PhotonTrackedTarget> target = Optional.empty();
  /** The latest camera result */
  protected PhotonPipelineResult camResult;
  /** The latest camera results */
  protected List<PhotonPipelineResult> camResults;

  /**
   * Constructor
   *
   * @param name - the name of the camera
   * @param colIndex - the column index for the dashboard
   * @param cameraToRobot - the camera-to-robot transform
   */
  public PhotonVisionCamera(String name, int colIndex, Transform3d cameraToRobot) {
    super(name, colIndex, cameraToRobot);
    this.robotToCamera = cameraToRobot;
    camera = new PhotonCamera(name);
  }

  /** The empty pipeline result sentinel — avoids allocating a new one each cycle */
  private static final PhotonPipelineResult EMPTY_RESULT = new PhotonPipelineResult();

  /** Update the camera and target with the latest result */
  @Override
  public void updateCameraInfo() {
    camResults = camera.getAllUnreadResults();
    // Avoid stream().findFirst() allocation — use direct index access
    camResult = camResults.isEmpty() ? EMPTY_RESULT : camResults.get(0);
    input.connected = camera.isConnected();
    input.captureTime = camResult.getTimestampSeconds();
  }

  /**
   * Get the target area
   *
   * @return the target area
   */
  @Override
  public double getTargetArea() {
    return target.map(t -> t.getArea()).orElse(Double.MAX_VALUE);
  }

  @Override
  public ProviderType getType() {
    return ProviderType.FIELD_BASED;
  }
}
