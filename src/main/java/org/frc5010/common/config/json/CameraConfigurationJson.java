// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.List;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.drive.swerve.GenericSwerveDrivetrain;
import org.frc5010.common.sensors.camera.FiducialTargetCamera;
import org.frc5010.common.sensors.camera.GenericCamera;
import org.frc5010.common.sensors.camera.LimeLightCamera;
import org.frc5010.common.sensors.camera.PhotonVisionCamera;
import org.frc5010.common.sensors.camera.PhotonVisionFiducialTargetCamera;
import org.frc5010.common.sensors.camera.PhotonVisionPoseCamera;
import org.frc5010.common.sensors.camera.PhotonVisionVisualTargetCamera;
import org.frc5010.common.sensors.camera.QuestNavInterface;
import org.frc5010.common.sensors.camera.SimulatedCamera;
import org.frc5010.common.sensors.camera.SimulatedFiducialTargetCamera;
import org.frc5010.common.sensors.camera.SimulatedVisualTargetCamera;
import org.frc5010.common.subsystems.FiducialTargetSystem;
import org.frc5010.common.subsystems.VisibleTargetSystem;
import org.frc5010.common.vision.AprilTags;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * Configuration data class for camera systems in an FRC robot.
 *
 * <p>This class represents the JSON configuration for a single camera, including its type
 * (Limelight, PhotonVision, AprilTag), physical pose relative to the robot center, and calibration
 * parameters. The configuration supports both real hardware and simulation modes.
 *
 * <p>Configuration is typically loaded from JSON files and used to instantiate the appropriate
 * camera implementation ({@link LimeLightCamera}, {@link PhotonVisionCamera}, etc.) via the {@link
 * #configureCamera(GenericRobot)} method.
 */
public class CameraConfigurationJson {
  /** Constant identifier for Limelight camera type */
  public static String LIMELIGHT = "limelight";
  /** Constant identifier for PhotonVision camera type */
  public static String PHOTON_VISION = "photonvision";
  /** Constant identifier for AprilTag detection mode */
  public static String APRIL_TAG = "apriltag";
  /** Constant identifier for target tracking mode */
  public static String TARGET = "target";

  /** Unique name of the camera used as a subsystem identifier */
  public String name;
  /**
   * The use case for this camera. Valid values include:
   *
   * <ul>
   *   <li>"target" - for target tracking systems
   *   <li>"apriltag" - for AprilTag-based pose estimation
   *   <li>"quest" - for QuestNav visual odometry
   * </ul>
   */
  public String use;
  /**
   * The camera vendor/type. Valid values include:
   *
   * <ul>
   *   <li>"limelight" - Limelight camera
   *   <li>"photonvision" - PhotonVision camera
   * </ul>
   *
   * Defaults to "none" if not specified.
   */
  public String type = "none";
  /**
   * The pose estimation strategy for PhotonVision cameras (e.g., "AVERAGE_BEST_TARGETS",
   * "LOWEST_AMBIGUITY"). Defaults to "none" if not using multi-target pose estimation.
   */
  public String strategy = "none";
  /** The SmartDashboard column index for this camera's telemetry display */
  public int column = 0;
  /** Camera X position offset from robot center in meters (forward/backward in robot frame) */
  public double x = 0;
  /** Camera Y position offset from robot center in meters (left/right in robot frame) */
  public double y = 0;
  /** Camera Z position offset from robot center in meters (up/down in robot frame) */
  public double z = 0;
  /** Camera roll rotation in degrees around the X-axis in the robot's reference frame */
  public double roll = 0;
  /** Camera pitch rotation in degrees around the Y-axis in the robot's reference frame */
  public double pitch = 0;
  /** Camera yaw rotation in degrees around the Z-axis in the robot's reference frame */
  public double yaw = 0;
  /** Horizontal resolution of the camera in pixels */
  public int width = 800;
  /** Vertical resolution of the camera in pixels */
  public int height = 600;
  /** Camera field of view (FOV) in degrees */
  public double fov = 70;
  /** Optional height of the target in meters (used for target tracking mode) */
  public double targetHeight = 0;
  /** Whether to view game pieces in simulation */
  public boolean viewGamePieces = true;
  /**
   * Optional array of AprilTag fiducial IDs to track. If empty, all AprilTags may be detected
   * depending on the configuration strategy.
   */
  public int[] targetFiducialIds = new int[0];

  /**
   * Sets whether to view game pieces in simulation. If true, the camera system will simulate vision
   * targets for game pieces in the arena. This can be useful for testing vision code without a real
   * camera. If false, the camera system will only process vision information for real camera
   * images.
   *
   * @param viewGamePieces whether to enable or disable viewing game pieces in simulation
   */
  public void setViewGamePieces(boolean viewGamePieces) {
    this.viewGamePieces = viewGamePieces;
  }

  /**
   * Returns whether the camera system can view game pieces in simulation mode.
   *
   * <p>This property is set by the {@link #setViewGamePieces(boolean)} method and controls whether
   * the camera system will simulate vision targets for game pieces in the arena. If true, the
   * camera system will simulate game pieces in simulation; otherwise, it will only process vision
   * information for real camera images.
   *
   * @return whether the camera system can view game pieces in simulation mode
   */
  public boolean canViewGamePieces() {
    return viewGamePieces;
  }

  /**
   * Configures the camera system based on the provided robot and current configuration.
   *
   * <p>This method performs the following operations:
   *
   * <ol>
   *   <li>Creates the appropriate camera instance based on the {@link #type} field (real hardware
   *       or simulation)
   *   <li>Initializes camera parameters (resolution, FOV, pose relative to robot)
   *   <li>Sets up the camera according to its configured {@link #use} case:
   *       <ul>
   *         <li>"target" - Creates a target tracking system
   *         <li>"apriltag" - Registers camera with pose estimator for localization
   *         <li>"quest" - Initializes QuestNav visual odometry system
   *       </ul>
   *   <li>Registers the camera/vision system with the robot
   * </ol>
   *
   * <p>The method handles both real and simulated robot modes ({@link RobotBase#isReal()}). In
   * simulation, cameras are created with physics-based simulation capabilities.
   *
   * @param robot the {@link GenericRobot} instance to configure the camera for. The robot must have
   *     a drivetrain subsystem with a pose estimator if using pose-based vision modes
   * @throws IllegalArgumentException if the camera type or use case is not recognized
   * @throws NullPointerException if required subsystems (e.g., drivetrain) are not available for
   *     the specified use case
   */
  public void configureCamera(GenericRobot robot) {
    GenericCamera camera = null;
    GenericDrivetrain drivetrain = (GenericDrivetrain) robot.getSubsystem("drivetrain");
    Transform3d robotToCamera =
        new Transform3d(
            new Translation3d(x, y, z),
            new Rotation3d(Degrees.of(roll), Degrees.of(pitch), Degrees.of(yaw)));

    if (RobotBase.isReal()) {
      switch (type) {
        case "limelight":
          {
            camera = new LimeLightCamera(name, column, robotToCamera);
            ((LimeLightCamera) camera)
                .setAngleSupplier(
                    ((GenericDrivetrain) robot.getSubsystem("drivetrain"))
                            .getPoseEstimator()
                            .getCurrentPose()
                        ::getRotation)
                .setPoseEstimationChooser(() -> false);
            ((LimeLightCamera) camera).setIMUMode(4);
            break;
          }
        case "photonvision":
          {
            if (!"none".equalsIgnoreCase(strategy)) {
              if (targetFiducialIds.length > 0) {
                List<Integer> targetFiducialIdList = new ArrayList<>();
                for (int targetFiducialId : targetFiducialIds) {
                  targetFiducialIdList.add(targetFiducialId);
                }
                camera =
                    new PhotonVisionPoseCamera(
                        name,
                        column,
                        AprilTags.aprilTagFieldLayout,
                        robotToCamera,
                        robot.getPoseSupplier(),
                        targetFiducialIdList);
              } else {
                camera =
                    new PhotonVisionPoseCamera(
                        name,
                        column,
                        AprilTags.aprilTagFieldLayout,
                        robotToCamera,
                        robot.getPoseSupplier());
              }
            } else if (targetFiducialIds.length > 0) {
              List<Integer> targetFiducialIdList = new ArrayList<>();
              for (int targetFiducialId : targetFiducialIds) {
                targetFiducialIdList.add(targetFiducialId);
              }
              camera =
                  new PhotonVisionFiducialTargetCamera(
                      name,
                      column,
                      AprilTags.aprilTagFieldLayout,
                      robotToCamera,
                      robot.getPoseSupplier(),
                      targetFiducialIdList);
            } else if (targetHeight > 0) {
              camera = new PhotonVisionVisualTargetCamera(name, column, robotToCamera);
            } else {
              camera = new PhotonVisionCamera(name, column, robotToCamera);
            }
            break;
          }
        default:
      }
    } else {
      if (!"none".equalsIgnoreCase(strategy)) {
        if (targetFiducialIds.length > 0) {
          List<Integer> targetFiducialIdList = new ArrayList<>();
          for (int targetFiducialId : targetFiducialIds) {
            targetFiducialIdList.add(targetFiducialId);
          }
          camera =
              new SimulatedCamera(
                  name,
                  column,
                  AprilTags.aprilTagFieldLayout,
                  PoseStrategy.valueOf(strategy),
                  robotToCamera,
                  robot.getPoseSupplier(),
                  targetFiducialIdList,
                  width,
                  height,
                  fov);
        } else {
          camera =
              new SimulatedCamera(
                  name,
                  column,
                  AprilTags.aprilTagFieldLayout,
                  robotToCamera,
                  robot.getSimulatedPoseSupplier(),
                  width,
                  height,
                  fov);
        }
      } else if (targetFiducialIds.length > 0) {
        List<Integer> targetFiducialIdList = new ArrayList<>();
        for (int targetFiducialId : targetFiducialIds) {
          targetFiducialIdList.add(targetFiducialId);
        }
        camera =
            new SimulatedFiducialTargetCamera(
                name,
                column,
                AprilTags.aprilTagFieldLayout,
                robotToCamera,
                robot.getSimulatedPoseSupplier(),
                targetFiducialIdList,
                width,
                height,
                fov);
      } else if (targetHeight > 0) {
        camera =
            new SimulatedVisualTargetCamera(
                name,
                column,
                AprilTags.aprilTagFieldLayout,
                robotToCamera,
                robot.getSimulatedPoseSupplier(),
                width,
                height,
                fov);
      } else if (!"quest".equalsIgnoreCase(use)) {
        camera =
            new SimulatedCamera(
                name,
                column,
                AprilTags.aprilTagFieldLayout,
                robotToCamera,
                robot.getSimulatedPoseSupplier(),
                width,
                height,
                fov);
      }
    }
    if (null != camera) {
      camera.setCanViewGamePieces(viewGamePieces);
    }
    switch (use) {
      case "target":
        {
          if (targetFiducialIds.length > 0) {
            robot.addSubsystem(name, new FiducialTargetSystem((FiducialTargetCamera) camera));
          } else {
            robot.addSubsystem(name, new VisibleTargetSystem(camera, targetHeight));
          }
          break;
        }
      case "apriltag":
        {
          if (drivetrain != null) {
            drivetrain.getPoseEstimator().registerPoseProvider(camera);
          }
          // if (targetFiducialIds.length > 0) {
          // robot.addSubsystem(name, new VisibleTargetSystem(camera, targetHeight));
          // }
          // atSystem.addCamera(camera);
          break;
        }
      case "quest":
        {
          QuestNavInterface questNav = new QuestNavInterface(robotToCamera);

          if (drivetrain != null) {
            // FIX: Undo this
            questNav.withRobotSpeedSupplier(
                ((GenericSwerveDrivetrain) drivetrain)::getFieldVelocity);
            drivetrain.getPoseEstimator().registerPoseProvider(questNav);
          }
          break;
        }
    }
  }
}
