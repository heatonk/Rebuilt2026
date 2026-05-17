// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.util.Controller;

public class RobotContainer {
  private final Rebuilt robot;

  public RobotContainer() {
    Controller driver = new Controller(0, "driver");
    Controller operator = new Controller(1, "operator");

    robot = new Rebuilt(driver, operator);

    initAutoCommands();
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
