// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt;

import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
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
import frc.robot.rebuilt.util.ButtonBindingRegistry;
import frc.robot.rebuilt.util.Controller;
import frc.robot.rebuilt.util.LedStrip;
import frc.robot.rebuilt.util.OrchestraManager;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Main robot class that initializes subsystems and commands. */
public class Rebuilt {
  public static HubStatus hubStatus;
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
  private static boolean everEnabled = false;

  private final Controller driver;
  private final Controller operator;
  private boolean isButtonsConfigured = false;
  private boolean isAltButtonsConfigured = false;
  private LoggedDashboardChooser<Command> selectableCommand;

  public Rebuilt(Controller driver, Controller operator) {
    this.driver = driver;
    this.operator = operator;

    // Open the teleop binding scope before any subsystem construction so any controller
    // bindings done during init (e.g. the operator START button below) land in this scope.
    ButtonBindingRegistry.beginScope("teleop");

    AllianceFlipUtil.configure(FieldConstants.FIELD_WIDTH, FieldConstants.FIELD_LENGTH);

    hubStatus = new HubStatus();
    indexer = new Indexer();
    intake = new Intake();
    launcher = new Launcher();
    drivetrain = new StubDrivetrain();

    testCommands = new TestCommands();
    launcherCommands = new LauncherCommands();
    intakecommands = new IntakeCommands();
    indexerCommands = new IndexerCommands();
    autocommands = new AutoCommands();

    RobotController.setBrownoutVoltage(Volts.of(4.6));

    operator.createStartButton().onTrue(launcher.zeroTurretCommand());
  }

  public void disabledInit() {
    OrchestraManager.play();
  }

  public void disabledPeriodic() {
    SmartDashboard.putBoolean("Orchestra Playing", OrchestraManager.isPlaying());
    if (launcher != null && !isZeroingBurst) {
      if (launcher.isTurretAtZero()) {
        LedStrip.changeSegmentPattern(LedStrip.ALL_LEDS, LedStrip.getSolidPattern(Color.kGreen));
      } else {
        LedStrip.changeSegmentPattern(
            LedStrip.ALL_LEDS, LedStrip.getSolidPattern(chooseAllianceWpiColor()));
      }
    }
    if (selectableCommand != null) {
      selectableCommand.periodic();
    }
  }

  /** Configures buttons with commands. */
  public void configureButtonBindings() {
    if (!isButtonsConfigured) {
      ButtonBindingRegistry.beginScope("teleop");
      FieldRegions.setupFieldRegions();
      driver.createYButton().onTrue(Commands.runOnce(() -> drivetrain.toggleFieldOrientedDrive()));
      drivetrain.configureButtonBindings(driver, operator);
      launcherCommands.configureButtonBindings(driver, operator);
      intakecommands.configureButtonBindings(driver, operator);
      indexerCommands.configureButtonBindings(driver, operator);
      hubStatus.configureButtonBindings(driver, operator);
      ButtonBindingRegistry.assertAndPublish("teleop");
      isButtonsConfigured = true;
    }
  }

  public void configureAltButtonBindings() {
    if (!isAltButtonsConfigured) {
      ButtonBindingRegistry.beginScope("test");
      testCommands.configureButtonBindings(driver);
      ButtonBindingRegistry.assertAndPublish("test");
      isAltButtonsConfigured = true;
    }
  }

  /** Assigns default commands for each subsystem. */
  public void setupDefaultCommands() {
    OrchestraManager.stop();
    FieldRegions.setupFieldRegions();
    drivetrain.setDefaultCommand(drivetrain.createDefaultCommand(driver));
    launcherCommands.setDefaultCommands();
    indexerCommands.setupDefaultCommands();
    intakecommands.setupDefaultCommands();
  }

  public void setupAltDefaultCommands() {
    // No alt-mode default commands.
  }

  public String determineAlliance() {
    return DriverStation.getAlliance().map(Enum::name).orElse("N/A");
  }

  /** Builds the auto chooser and registers options. */
  public void buildAutoCommands() {
    NamedCommandsReg.createNamedCommands();
    drivetrain.setAutoBuilder();
    if (AutoBuilder.isConfigured()) {
      selectableCommand =
          new LoggedDashboardChooser<>("Auto Modes", AutoBuilder.buildAutoChooser());
      SmartDashboard.putData("Auto Modes", selectableCommand.getSendableChooser());
    } else {
      selectableCommand = new LoggedDashboardChooser<>("Auto Modes");
      SmartDashboard.putData("Auto Modes", selectableCommand.getSendableChooser());
    }
    selectableCommand.addOption("Do Nothing", Commands.none());
    drivetrain.addAutoCommands(selectableCommand);
    autocommands.configureCharacterizationCommands(selectableCommand);
  }

  public Command getAutonomousCommand() {
    everEnabled = true;
    if (selectableCommand == null || selectableCommand.get() == null) {
      return Commands.none();
    }
    return drivetrain.generateAutoCommand(selectableCommand.get().asProxy());
  }

  /** Returns true once {@link #getAutonomousCommand()} has fired at least once this session. */
  public static boolean hasEverEnabled() {
    return everEnabled;
  }

  /**
   * Returns the WPI color associated with the current alliance, or orange when no alliance is
   * known.
   */
  public static Color chooseAllianceWpiColor() {
    return DriverStation.getAlliance()
        .map(a -> a == Alliance.Red ? Color.kRed : Color.kBlue)
        .orElse(Color.kOrange);
  }
}
