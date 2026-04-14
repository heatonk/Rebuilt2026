// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import java.util.stream.Collectors;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.constants.Constants;
import org.frc5010.common.sensors.camera.GenericCamera;
import org.frc5010.common.sensors.camera.SimulatedCamera;
import org.frc5010.common.telemetry.DisplayBoolean;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionTargetSim;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/**
 * Abstract base class for vision-based camera subsystems.
 *
 * <p>This class provides the framework for implementing camera subsystems that process vision data
 * from one or more cameras. Subclasses implement specific behavior based on what the camera is
 * tracking (AprilTags, targets, game pieces, etc.).
 *
 * <p>Key responsibilities:
 *
 * <ul>
 *   <li>Periodically update camera data ({@link #periodic()})
 *   <li>Detect and track targets ({@link #hasValidTarget()})
 *   <li>Calculate distance and orientation to targets ({@link #getDistanceToTarget()})
 *   <li>Simulate vision targets in simulation mode ({@link #simulationPeriodic()})
 * </ul>
 *
 * <p>This is an abstract class; concrete implementations include:
 *
 * <ul>
 *   <li>{@link FiducialTargetSystem} - For AprilTag tracking
 *   <li>{@link VisibleTargetSystem} - For visual target tracking
 *   <li>{@link AprilTagPoseSystem} - For pose estimation
 * </ul>
 *
 * @see GenericCamera
 * @see GenericSubsystem
 */
public abstract class CameraSystem extends GenericSubsystem {
  /** The camera instance providing the vision data */
  protected GenericCamera camera;

  /** Telemetry display for camera target validity state */
  protected DisplayBoolean HAS_VALID_TARGET;
  /** Target model used for vision simulation (default: 12-inch diameter circle at ~0.3556m) */
  protected TargetModel targetModel = new TargetModel(0.3556);
  /** Whether to view game pieces in simulation */
  protected boolean viewGamePieces = true;
  /** Cached game piece A list to detect changes between cycles */
  private List<Pose3d> cachedGpas = List.of();
  /** Cached game piece B list to detect changes between cycles */
  private List<Pose3d> cachedGpbs = List.of();

  /**
   * Creates a new CameraSystem with the specified camera.
   *
   * <p>Initializes the camera subsystem and sets up telemetry displays for vision system status and
   * target information.
   *
   * @param camera the {@link GenericCamera} instance to use for vision processing
   * @throws NullPointerException if camera is null
   */
  public CameraSystem(GenericCamera camera) {
    this.camera = camera;
    HAS_VALID_TARGET = DashBoard.makeDisplayBoolean("Has Valid Target");
  }

  /**
   * Periodic update method called by the WPILib scheduler.
   *
   * <p>This method is invoked once per scheduler cycle (~20ms on FRC robots). It updates camera
   * data and processes vision information.
   *
   * @see #updateCameraInfo()
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateCameraInfo();
  }

  @Override
  public void simulationPeriodic() {
    if (!camera.canViewGamePieces()) {
      return;
    }
    // Update game piece A targets only when the list has changed
    List<Pose3d> gpas =
        SimulatedArena.getInstance().getGamePiecesByType(Constants.Simulation.gamePieceA).stream()
            .map(it -> it.getPose3d())
            .collect(Collectors.toList());
    if (!gpas.equals(cachedGpas)) {
      cachedGpas = gpas;
      SimulatedCamera.visionSim.removeVisionTargets("GPA");
      for (Pose3d gpa : gpas) {
        VisionTargetSim simTarget = new VisionTargetSim(gpa, targetModel);
        SimulatedCamera.visionSim.addVisionTargets("GPA", simTarget);
      }
    }
    // Update game piece B targets only when the list has changed
    List<Pose3d> gpbs =
        SimulatedArena.getInstance().getGamePiecesByType(Constants.Simulation.gamePieceB).stream()
            .map(it -> it.getPose3d())
            .collect(Collectors.toList());
    if (!gpbs.equals(cachedGpbs)) {
      cachedGpbs = gpbs;
      SimulatedCamera.visionSim.removeVisionTargets("GPB");
      for (Pose3d gpb : gpbs) {
        VisionTargetSim simTarget = new VisionTargetSim(gpb, targetModel);
        SimulatedCamera.visionSim.addVisionTargets("GPB", simTarget);
      }
    }
  }

  /**
   * Enables or disables the camera from viewing game pieces in simulation. When enabled, the camera
   * will simulate vision targets for game pieces in the arena. This can be useful for testing
   * vision code without a real camera. When disabled, the camera will only process vision
   * information for real camera images.
   *
   * @param viewGamePieces whether to enable or disable viewing game pieces in simulation
   */
  public void setViewGamePieces(boolean viewGamePieces) {
    this.viewGamePieces = viewGamePieces;
  }

  /**
   * Updates camera information during each periodic cycle.
   *
   * <p>This method is called from {@link #periodic()} and is responsible for refreshing vision data
   * from the camera. Subclasses may override this to perform additional processing beyond basic
   * camera updates.
   *
   * @see GenericCamera#update()
   */
  public void updateCameraInfo() {
    camera.update();
  }

  /**
   * Gets the distance from the camera to the detected target.
   *
   * <p>This method must be implemented by subclasses to calculate the distance based on the
   * specific type of target being tracked. The meaning of "distance" depends on the implementation:
   *
   * <ul>
   *   <li>For target-based systems: horizontal distance to the target
   *   <li>For AprilTag systems: 3D distance to the tag center
   *   <li>For game piece systems: distance to the piece
   * </ul>
   *
   * @return the distance to the target in meters, or a negative value if no valid target
   */
  public abstract double getDistanceToTarget();

  /**
   * Checks if the camera currently has a valid target in view.
   *
   * <p>This abstract method must be implemented by subclasses to determine the validity of the
   * current target based on confidence, ambiguity, or other criteria appropriate to the specific
   * vision task.
   *
   * @return {@code true} if a valid target is detected, {@code false} otherwise
   */
  protected abstract boolean hasValidTarget();

  /**
   * Creates a Trigger that activates when a valid target is detected.
   *
   * <p>This can be used in command scheduling to automatically trigger actions when a target
   * becomes available. For example:
   *
   * <pre>
   *   cameraSystem.hasAValidTarget().onTrue(alignCommand);
   * </pre>
   *
   * @return a {@link Trigger} that is active when {@link #hasValidTarget()} returns true
   */
  public Trigger hasAValidTarget() {
    return new Trigger(this::hasValidTarget);
  }
}
