package frc.robot.rebuilt.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Rebuilt;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoCommands {

  public AutoCommands() {}

  public void configureNamedCommands() {}

  public void configureCharacterizationCommands(LoggedDashboardChooser<Command> selectableCommand) {
    selectableCommand.addOption(
        "PRO: Intake Hopper Characterization",
        Rebuilt.intake.getHopperCharacterizationCommand());
    selectableCommand.addOption(
        "PRO: Launcher Hood Characterization",
        Rebuilt.launcher.getHoodCharacterizationCommand());
    selectableCommand.addOption(
        "PRO: Launcher Turret Characterization",
        Rebuilt.launcher.getTurretCharacterizationCommand());
    selectableCommand.addOption(
        "TUNE: Shot Lookup Table Tuning",
        ShotCalibrationCommand.createWithFeed(
            Rebuilt.launcher,
            Rebuilt.drivetrain,
            2.0,
            0.5));
    selectableCommand.addOption(
        "PRO: Turret Quasistatic (kS, kV)",
        Rebuilt.launcher.getTurretQuasistaticCommand());
    selectableCommand.addOption(
        "PRO: Turret Dynamic (kA)",
        Rebuilt.launcher.getTurretDynamicCommand());
    selectableCommand.addOption(
        "TUNE: Turret kS Map Generation",
        Rebuilt.launcher.getTurretKsMapCommand());
    selectableCommand.addOption(
        "TUNE: Turret Tracking Sinusoidal Tuning",
        Rebuilt.launcher.getTurretTrackingTuneCommand());

    selectableCommand.addOption(
        "TUNE: Turret Seeking Tuning",
        Rebuilt.launcher.getTurretSeekingTuneCommand());
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
