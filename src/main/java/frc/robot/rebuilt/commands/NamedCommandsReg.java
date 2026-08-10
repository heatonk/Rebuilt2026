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
    NamedCommands.registerCommand("intakeRetracted", IntakeCommands.shouldRetracted());
    NamedCommands.registerCommand("intakeRetracting", IntakeCommands.shouldRetracting());
    // indexer
    NamedCommands.registerCommand("indexerChurn", IndexerCommands.shouldChurnCommand());
    NamedCommands.registerCommand("indexerIdle", IndexerCommands.shouldIdleCommand());
    NamedCommands.registerCommand("indexerFeed", IndexerCommands.shouldFeedCommand());
    // preset
    NamedCommands.registerCommand("iForcePreset", IndexerCommands.shouldForceCommand());
    // TODO: restore hub/tower preset auto commands — leftCornerPresetStateCommand and
    // towerPresetStateCommand were removed from LauncherCommands and need reinstating
    // (unrelated to YAMS removal — pre-existing gap).
    NamedCommands.registerCommand(
        "towerForwardPreset", LauncherCommands.turretForwardPresetStateCommand());
    NamedCommands.registerCommand("WaitUntilIntaking", IntakeCommands.waitUntilIntaking());
  }
}
