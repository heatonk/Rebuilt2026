// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.commands.AutoCommands;
import frc.robot.rebuilt.commands.IndexerCommands;
import frc.robot.rebuilt.commands.IntakeCommands;
import frc.robot.rebuilt.commands.LauncherCommands;
import frc.robot.rebuilt.commands.NamedCommandsReg;
import frc.robot.rebuilt.commands.TestCommands;
import frc.robot.rebuilt.subsystems.DriverDisplay.HubStatus;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import frc.robot.rebuilt.subsystems.Launcher.FieldRegions;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import frc.robot.rebuilt.subsystems.drive.StubDrivetrain;
import frc.robot.rebuilt.subsystems.intake.Intake;
import frc.robot.rebuilt.util.AllianceFlipUtil;
import frc.robot.rebuilt.util.LedStrip;
import frc.robot.rebuilt.util.OrchestraManager;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.sensors.Controller;

/** This is an example robot class. */
/** Long's correction: Main robot class that initializes subsystems and commands */
public class Rebuilt extends GenericRobot {
  public static String configDirectory = "rebuilt_robot";
  public static HubStatus hubStatus = new HubStatus();
  public static StubDrivetrain drivetrain;
  public static Indexer indexer;
  public static Intake intake;
  public static Launcher launcher;
  public static LauncherCommands launcherCommands;
  public static AutoCommands autocommands;
  public static IntakeCommands intakecommands;
  public static IndexerCommands indexerCommands;
  public static TestCommands testCommands;
  public static boolean isZeroingBurst = false;
  private boolean isButtonsConfigured = false;
  private boolean isAltButtonsConfigured = false;

  public Rebuilt(String directory) {
    super(directory);
    configDirectory = directory;
    AllianceFlipUtil.configure(FieldConstants.FIELD_WIDTH, FieldConstants.FIELD_LENGTH);
    /** creating robot subsystems */
    indexer = new Indexer();
    intake = new Intake();
    launcher = new Launcher(subsystems);
    drivetrain = new StubDrivetrain();
    /** creates command containers */
    testCommands = new TestCommands(subsystems);
    launcherCommands = new LauncherCommands(subsystems);
    intakecommands = new IntakeCommands(subsystems);
    indexerCommands = new IndexerCommands(subsystems);
    autocommands = new AutoCommands(subsystems);
    // OrchestraManager.loadMusic("raiders");
    RobotController.setBrownoutVoltage(Volts.of(4.6));

    if (operator.isPresent()) {
      operator.get().createStartButton().onTrue(launcher.zeroTurretCommand());
    }
  }

  @Override
  public void disabledInit() {
    OrchestraManager.play();
  }

  @Override
  public void disabledPeriodic() {
    super.disabledPeriodic();
    SmartDashboard.putBoolean("Orchestra Playing", OrchestraManager.isPlaying());
    if (launcher != null && !isZeroingBurst) {
      if (launcher.isTurretAtZero()) {
        LedStrip.changeSegmentPattern(
            LedStrip.ALL_LEDS,
            LedStrip.getSolidPattern(edu.wpi.first.wpilibj.util.Color.kGreen));
      } else {
        LedStrip.changeSegmentPattern(
            LedStrip.ALL_LEDS, LedStrip.getSolidPattern(chooseAllianceWpiColor()));
      }
    }
  }

  @Override
  /** Configures buttons with commands */
  public void configureButtonBindings(Controller driver, Controller operator) {
    if (!isButtonsConfigured) {
      FieldRegions.setupFieldRegions();
      driver.createYButton().onTrue(Commands.runOnce(() -> drivetrain.toggleFieldOrientedDrive()));
      drivetrain.configureButtonBindings(driver, operator);
      launcherCommands.configureButtonBindings(driver, operator);
      intakecommands.configureButtonBindings(driver, operator);
      indexerCommands.configureButtonBindings(driver, operator);
      hubStatus.configureButtonBindings(driver, operator);
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
