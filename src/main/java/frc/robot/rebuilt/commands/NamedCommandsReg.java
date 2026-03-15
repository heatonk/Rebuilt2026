package frc.robot.rebuilt.commands;

import com.pathplanner.lib.auto.NamedCommands;

public class NamedCommandsReg {

  public static void createNamedCommands() {
    // Launcer
    NamedCommands.registerCommand("launcherPrep", LauncherCommands.shouldPrepCommand());
    NamedCommands.registerCommand("launcherPreset", LauncherCommands.shouldPresetCommand());
    NamedCommands.registerCommand("launcherLow", LauncherCommands.shouldLowCommand());
    NamedCommands.registerCommand("launcherIdle", LauncherCommands.shouldIdleCommand());
    // intake
    NamedCommands.registerCommand("intakeIntake", IntakeCommands.shouldIntaking());
    NamedCommands.registerCommand("intakeOuttake", IntakeCommands.shouldOuttaking());
    NamedCommands.registerCommand("intakeRetracted", IntakeCommands.shouldRetracted());
    NamedCommands.registerCommand("intakeRetracting", IntakeCommands.shouldRetracting());
    // climb
    NamedCommands.registerCommand("climbDescend", ClimbCommands.shouldDescendCommand());
    NamedCommands.registerCommand("climbElevate", ClimbCommands.shouldElevateCommand());
    NamedCommands.registerCommand("climbEnable", ClimbCommands.shouldEnableCommand());
    NamedCommands.registerCommand("climbStop", ClimbCommands.shouldStopCommand());
    // indexer
    NamedCommands.registerCommand("indexerChurn", IndexerCommands.shouldChurnCommand());
    NamedCommands.registerCommand("indexerIdle", IndexerCommands.shouldIdleCommand());
    NamedCommands.registerCommand("indexerFeed", IndexerCommands.shouldFeedCommand());
    // preset
    NamedCommands.registerCommand("iForcePreset", IndexerCommands.shouldForceCommand());
    NamedCommands.registerCommand("hubPreset", LauncherCommands.leftCornerPresetStateCommand());
    NamedCommands.registerCommand("towerPreset", LauncherCommands.towerPresetStateCommand());
    NamedCommands.registerCommand(
        "towerForwardPreset", LauncherCommands.turretForwardPresetStateCommand());
  }
}
