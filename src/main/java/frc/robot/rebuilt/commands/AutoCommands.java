package frc.robot.rebuilt.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.commands.testCommands.ShotCalibrationCommand;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoCommands {

  public AutoCommands() {}

  public void configureNamedCommands() {}

  public void configureCharacterizationCommands(LoggedDashboardChooser<Command> selectableCommand) {
    selectableCommand.addOption(
        "PRO: Intake Hopper Characterization", Rebuilt.intake.getHopperCharacterizationCommand());
    selectableCommand.addOption(
        "PRO: Launcher Hood Characterization", Rebuilt.launcher.getHoodCharacterizationCommand());
    selectableCommand.addOption(
        "PRO: Launcher Turret Characterization",
        Rebuilt.launcher.getTurretCharacterizationCommand());
    selectableCommand.addOption(
        "TUNE: Shot Lookup Table Tuning",
        ShotCalibrationCommand.createWithFeed(Rebuilt.launcher, Rebuilt.drivetrain, 2.0, 0.5));
  }

  public void configureBasicAutoCommands(LoggedDashboardChooser<Command> selectableCommand) {
    selectableCommand.addOption(
        "Shoot Preload Only",
        Commands.sequence(
            IntakeCommands.shouldIntaking(),
            Commands.waitSeconds(2),
            LauncherCommands.shouldPrepCommand(),
            Commands.waitSeconds(2),
            IndexerCommands.shouldForceCommand()));
  }
}
