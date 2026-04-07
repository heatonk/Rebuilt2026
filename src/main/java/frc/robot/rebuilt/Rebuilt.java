// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.commands.AutoCommands;
import frc.robot.rebuilt.commands.ClimbCommands;
import frc.robot.rebuilt.commands.IndexerCommands;
import frc.robot.rebuilt.commands.IntakeCommands;
import frc.robot.rebuilt.commands.LauncherCommands;
import frc.robot.rebuilt.commands.NamedCommandsReg;
import frc.robot.rebuilt.commands.TestCommands;
import frc.robot.rebuilt.subsystems.Climb.Climb;
import frc.robot.rebuilt.subsystems.DriverDisplay.HubStatus;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import frc.robot.rebuilt.subsystems.Launcher.FieldRegions;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import frc.robot.rebuilt.subsystems.intake.Intake;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.utils.OrchestraManager;
import org.frc5010.common.utils.geometry.AllianceFlipUtil;

/** This is an example robot class. */
/** Long's correction: Main robot class that initializes subsystems and commands */
public class Rebuilt extends GenericRobot {
  public static HubStatus hubStatus = new HubStatus();
  public static GenericDrivetrain drivetrain;
  public static Indexer indexer;
  public static Climb climb;
  public static Intake intake;
  public static Launcher launcher;
  public static LauncherCommands launcherCommands;
  public static AutoCommands autocommands;
  public static ClimbCommands climbCommands;
  public static IntakeCommands intakecommands;
  public static IndexerCommands indexerCommands;
  public static TestCommands testCommands;
  private boolean isButtonsConfigured = false;
  private boolean isAltButtonsConfigured = false;

  public Rebuilt(String directory) {
    super(directory);
    AllianceFlipUtil.configure(FieldConstants.FIELD_WIDTH, FieldConstants.FIELD_LENGTH);
    /** creating robot subsystems */
    indexer = new Indexer();
    // climb = new Climb();
    intake = new Intake();
    launcher = new Launcher(subsystems);
    drivetrain = (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);
    /** creates command containers */
    testCommands = new TestCommands(subsystems);
    climbCommands = new ClimbCommands(subsystems);
    launcherCommands = new LauncherCommands(subsystems);
    intakecommands = new IntakeCommands(subsystems);
    indexerCommands = new IndexerCommands(subsystems);
    autocommands = new AutoCommands(subsystems);
    OrchestraManager.loadMusic("sea2");
  }

  @Override
  public void disabledInit() {
    OrchestraManager.play();
  }

  @Override
  /** Configures buttons with commands */
  public void configureButtonBindings(Controller driver, Controller operator) {
    if (!isButtonsConfigured) {
      FieldRegions.setupFieldRegions();
      driver.createYButton().onTrue(Commands.runOnce(() -> drivetrain.toggleFieldOrientedDrive()));
      drivetrain.configureButtonBindings(driver, operator);
      climbCommands.configureButtonBindings(driver, operator);
      launcherCommands.configureButtonBindings(driver, operator);
      intakecommands.configureButtonBindings(driver, operator);
      indexerCommands.configureButtonBindings(driver, operator);
      isButtonsConfigured = true;
    }
  }

  @Override
  public void configureAltButtonBindings(Controller driver, Controller operator) {
    // Add test mode specific button bindings here
    if (!isAltButtonsConfigured) {
      testCommands.configureButtonBindings(driver);
      isAltButtonsConfigured = true;
    }
  }

  @Override
  /** Assigns default commands for each subsystem */
  public void setupDefaultCommands(Controller driver, Controller operator) {
    OrchestraManager.stop();
    // This is part of auto init, so a good place to run this
    FieldRegions.setupFieldRegions();
    drivetrain.setDefaultCommand(drivetrain.createDefaultCommand(driver));
    launcherCommands.setDefaultCommands();
    indexerCommands.setupDefaultCommands();
    intakecommands.setupDefaultCommands();
  }

  @Override
  public void initAutoCommands() {
    NamedCommandsReg.createNamedCommands();
    drivetrain.setAutoBuilder();
  }

  // @Override
  // public Command getAutonomousCommand() {
  //   if (DriverStation.isFMSAttached()) {
  //     intake.setHopperPosition(Constants.Intake.HOPPER_RETRACTED_ANGLE);
  //   }
  //   return super.getAutonomousCommand();
  // }

  @Override
  public Command generateAutoCommand(Command autoCommand) {
    return drivetrain.generateAutoCommand(autoCommand);
  }

  @Override
  /** Creates and registers available auto comands */
  public void buildAutoCommands() {
    super.buildAutoCommands();
    selectableCommand.addOption("Do Nothing", Commands.none());
    drivetrain.addAutoCommands(selectableCommand);
    autocommands.configureCharacterizationCommands(selectableCommand);
  }
}
