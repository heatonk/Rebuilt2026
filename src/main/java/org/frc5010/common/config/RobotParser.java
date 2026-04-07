// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.util.Map;
import java.util.Optional;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.json.AKitSwerveDrivetrainJson;
import org.frc5010.common.config.json.CameraConfigurationJson;
import org.frc5010.common.config.json.DriveteamControllersJson;
import org.frc5010.common.config.json.DrivetrainPropertiesJson;
import org.frc5010.common.config.json.RobotJson;
import org.frc5010.common.config.json.VisionPropertiesJson;
import org.frc5010.common.config.json.YAGSLDrivetrainJson;
import org.frc5010.common.config.json.devices.LEDStripParser;
import org.frc5010.common.config.json.devices.OrchestraParser;

/**
 * Parses JSON configuration files to initialize and build a robot's subsystems.
 *
 * <p>This class is responsible for reading and processing JSON configuration files from the robot's
 * deploy directory. It orchestrates the loading and configuration of:
 *
 * <ul>
 *   <li>Robot base configuration (basic properties)
 *   <li>Driveteam controller bindings
 *   <li>Vision system cameras
 *   <li>Drivetrain (swerve or other type)
 *   <li>LED strips
 * </ul>
 *
 * <p>The typical usage pattern is:
 *
 * <pre>
 *   RobotParser parser = new RobotParser("myRobotDir", robot);
 *   parser.createRobot(robot);
 * </pre>
 *
 * <p>Configuration files are expected to be located in the robot's deploy directory and must follow
 * the standard FRC5010 JSON schema.
 *
 * @see GenericRobot
 * @see DriveteamControllerConfiguration
 * @see CameraConfigurationJson
 */
public class RobotParser {
  /** JSON classes for the Driveteam controllers */
  private static DriveteamControllersJson controllersJson;
  /** Map of Driveteam controller configurations */
  private static Map<String, DriveteamControllerConfiguration> controllersMap;
  /** JSON classes for the cameras */
  private static VisionPropertiesJson visionJson;
  /** Map of camera configurations */
  private static Map<String, CameraConfigurationJson> camerasMap;
  /** JSON class for the drivetrain */
  private static Optional<DrivetrainPropertiesJson> driveTrainJson = Optional.empty();

  /**
   * Creates a new RobotParser and initializes robot configuration from JSON files.
   *
   * <p>This constructor performs the following steps:
   *
   * <ol>
   *   <li>Locates the robot configuration directory
   *   <li>Loads and parses robot.json for base robot configuration
   *   <li>Loads and parses controllers.json for driveteam controller configuration
   *   <li>Loads and parses cameras.json for vision system configuration
   *   <li>Parses LED strip configurations
   *   <li>Loads drivetrain configuration (YAGSL swerve or AdvantageKit swerve)
   * </ol>
   *
   * <p>The configuration is not fully applied to the robot until {@link #createRobot(GenericRobot)}
   * is called.
   *
   * @param robotDirectory the name of the directory (relative to the deploy directory) containing
   *     the robot configuration JSON files
   * @param robot the {@link GenericRobot} instance that will be configured with the loaded
   *     configuration
   * @throws IOException if any configuration file cannot be read or parsed
   * @throws AssertionError if required configuration files (robot.json) are missing
   */
  public RobotParser(String robotDirectory, GenericRobot robot) throws IOException {
    File directory = new File(Filesystem.getDeployDirectory(), robotDirectory);
    checkDirectory(directory);

    // Read in the robot configuration
    RobotJson robotJson =
        new ObjectMapper().readValue(new File(directory, "robot.json"), RobotJson.class);
    robotJson.configureRobot(robot, directory);

    // Read in the controllers
    controllersJson =
        new ObjectMapper()
            .readValue(new File(directory, "controllers.json"), DriveteamControllersJson.class);
    controllersMap = controllersJson.readControllers(directory);

    // Read in the cameras
    visionJson =
        new ObjectMapper()
            .readValue(new File(directory, "cameras.json"), VisionPropertiesJson.class);
    camerasMap = visionJson.readCameraSystem(directory);

    // Parse LED strips
    LEDStripParser.parse(robotDirectory);

    // Parse the orchestra configuration (if it exists)
    OrchestraParser.parse(robotDirectory);

    // Read in the drivetrain
    switch (robotJson.driveType) {
      case "YAGSL_SWERVE_DRIVE":
        {
          YAGSLDrivetrainJson yagslDriveTrainJson =
              new ObjectMapper()
                  .readValue(
                      new File(directory, "yagsl_drivetrain.json"), YAGSLDrivetrainJson.class);
          yagslDriveTrainJson.readDrivetrainConfiguration(robot, directory);
          driveTrainJson = Optional.of(yagslDriveTrainJson);
          break;
        }
      case "AKIT_SWERVE_DRIVE":
        {
          AKitSwerveDrivetrainJson akitDriveTrainJson =
              new ObjectMapper()
                  .readValue(
                      new File(directory, "akit_swerve_drivetrain.json"),
                      AKitSwerveDrivetrainJson.class);
          akitDriveTrainJson.readDrivetrainConfiguration(robot, directory);
          driveTrainJson = Optional.of(akitDriveTrainJson);
          break;
        }
      default:
        break;
    }
  }

  /**
   * Verifies that required JSON configuration files exist in the directory.
   *
   * <p>This method ensures that critical configuration files (at minimum, robot.json) are present
   * in the configuration directory before proceeding with parsing.
   *
   * @param directory the directory to check for JSON configuration files
   * @throws AssertionError if required configuration files do not exist
   */
  private void checkDirectory(File directory) {
    assert new File(directory, "robot.json").exists();
  }

  /**
   * Applies all loaded configurations to create and initialize the robot's subsystems.
   *
   * <p>This method instantiates and configures all robot subsystems based on the JSON configuration
   * files that were loaded during construction. It performs:
   *
   * <ul>
   *   <li>Creation of driveteam controller bindings
   *   <li>Creation of the drivetrain subsystem (if configured)
   *   <li>Creation of vision/camera subsystems
   * </ul>
   *
   * <p>This method should be called after the robot instance is fully initialized and ready to
   * receive subsystems.
   *
   * @param robot the {@link GenericRobot} instance to apply configurations to
   * @throws NullPointerException if required configuration data was not loaded successfully
   * @see #RobotParser(String, GenericRobot) for the loading phase
   */
  public void createRobot(GenericRobot robot) {
    controllersJson.createControllers(robot, controllersMap);
    driveTrainJson.ifPresent(it -> it.createDriveTrain(robot));
    visionJson.createCameraSystem(robot, camerasMap);
  }
}
