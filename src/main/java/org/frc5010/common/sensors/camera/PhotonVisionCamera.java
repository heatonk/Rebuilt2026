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

  /**
   * Maximum number of camera frames to process per 20ms loop cycle.
   *
   * <p>PhotonVision buffers every frame received since the NT connection was established. On the
   * very first call to {@code getAllUnreadResults()} (typically ~16 s after boot), that buffer may
   * contain hundreds of stale frames (16 s × 30 fps = ~480 frames per camera). Processing all of
   * them in one cycle causes a 115–400 ms first-periodic overrun. By capping the list to the
   * most-recent MAX_FRAMES_PER_CYCLE entries we limit latency while still receiving all frames that
   * arrive during a single 20 ms window (≤ 2 at 30 fps).
   */
  private static final int MAX_FRAMES_PER_CYCLE = 10;

  /** Update the camera and target with the latest result */
  @Override
  public void updateCameraInfo() {
    List<PhotonPipelineResult> allResults = camera.getAllUnreadResults();
    input.unreadResultCount = allResults.size();
    // Drain stale frames: keep only the most recent MAX_FRAMES_PER_CYCLE results.
    // This prevents a multi-hundred-millisecond stall on the first periodic() call when
    // the NT subscriber queue has accumulated hundreds of buffered frames since boot.
    int size = allResults.size();
    if (size > MAX_FRAMES_PER_CYCLE) {
      camResults = allResults.subList(size - MAX_FRAMES_PER_CYCLE, size);
    } else {
      camResults = allResults;
    }
    input.processedResultCount = camResults.size();
    input.droppedResultCount = Math.max(0, size - camResults.size());
    // Avoid stream().findFirst() allocation — use direct index access
    camResult = camResults.isEmpty() ? EMPTY_RESULT : camResults.get(camResults.size() - 1);
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
