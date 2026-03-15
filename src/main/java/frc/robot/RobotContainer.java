// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.arch.WpiHelperInterface;
import org.frc5010.common.arch.WpiNetworkTableValuesHelper;
import org.frc5010.common.config.RobotsParser;
import org.frc5010.common.constants.Constants;

public class RobotContainer implements WpiHelperInterface {
  private static final RobotsParser robotsParser = new RobotsParser();
  public static Constants constants;
  private GenericRobot robot;

  public RobotContainer() {
    constants = new Constants();

    robot = robotsParser.getRobot();

    initAutoCommands();
    WpiNetworkTableValuesHelper.loadRegisteredToNetworkTables();
  }

  /**
   * Configures the button bindings for the robot. This should be called from the robot periodic
   * methods (i.e. robotPeriodic) to update the button bindings.
   */
  public void configureButtonBindings() {
    robot.configureButtonBindings();
  }

  public void disabledInit() {
    robot.disabledInit();
  }

  public void configureAltButtonBindings() {
    robot.configureAltButtonBindings();
  }

  // Just sets up defalt commands (setUpDeftCom)
  public void setupDefaults() {
    robot.determineAlliance();
    robot.setupDefaultCommands();
  }

  public void setupAltDefaultCommands() {
    robot.setupAltDefaultCommands();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return robot.getAutonomousCommand();
  }

  public void initAutoCommands() {
    robot.buildAutoCommands();
  }

  public void disabledPeriodic() {
    robot.disabledPeriodic();
  }
}
