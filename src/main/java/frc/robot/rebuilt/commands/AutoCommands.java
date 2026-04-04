package frc.robot.rebuilt.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import frc.robot.rebuilt.subsystems.intake.Intake;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.drive.GenericDrivetrain;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoCommands {

  private Map<String, GenericSubsystem> subsystems;

  public AutoCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;
  }

  public void configureNamedCommands() {}

  public void configureCharacterizationCommands(LoggedDashboardChooser<Command> selectableCommand) {
    selectableCommand.addOption(
        "PRO: Intake Hopper Characterization",
        ((Intake) subsystems.get(Constants.INTAKE)).getHopperCharacterizationCommand());
    selectableCommand.addOption(
        "PRO: Launcher Hood Characterization",
        ((Launcher) subsystems.get(Constants.LAUNCHER)).getHoodCharacterizationCommand());
    selectableCommand.addOption(
        "PRO: Launcher Turret Characterization",
        ((Launcher) subsystems.get(Constants.LAUNCHER)).getTurretCharacterizationCommand());
    selectableCommand.addOption(
        "TUNE: Shot Lookup Table Tuning",
        ShotCalibrationCommand.createWithFeed(
            (Launcher) subsystems.get(Constants.LAUNCHER),
            (GenericDrivetrain)
                subsystems.get(org.frc5010.common.config.ConfigConstants.DRIVETRAIN),
            2.0,
            0.5));
    selectableCommand.addOption(
        "PRO: Turret Quasistatic (kS, kV)",
        ((Launcher) subsystems.get(Constants.LAUNCHER)).getTurretQuasistaticCommand());
    selectableCommand.addOption(
        "PRO: Turret Dynamic (kA)",
        ((Launcher) subsystems.get(Constants.LAUNCHER)).getTurretDynamicCommand());
    selectableCommand.addOption(
        "TUNE: Turret kS Map Generation",
        ((Launcher) subsystems.get(Constants.LAUNCHER)).getTurretKsMapCommand());
    selectableCommand.addOption(
        "TUNE: Turret Tracking Sinusoidal Tuning",
        ((Launcher) subsystems.get(Constants.LAUNCHER)).getTurretTrackingTuneCommand());
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
