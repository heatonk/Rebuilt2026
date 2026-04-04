package frc.robot.rebuilt.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Constants;
// import frc.robot.rebuilt.subsystems.Climb.Climb;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import frc.robot.rebuilt.subsystems.intake.Intake;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.sensors.Controller;

public class TestCommands {

  private Map<String, GenericSubsystem> subsystems;

  Indexer indexer;
  // Climb climb;
  Intake intake;
  static Launcher launcher;

  public TestCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;
    indexer = (Indexer) subsystems.get(Constants.INDEXER);
    // climb = (Climb) subsystems.get(Constants.CLIMB);
    intake = (Intake) subsystems.get(Constants.INTAKE);
    launcher = (Launcher) subsystems.get(Constants.LAUNCHER);
  }

  public void configureButtonBindings(Controller controller) {
    controller.setRightYAxis(controller.createRightYAxis().negate().deadzone(0.07));
    controller.setLeftYAxis(controller.createLeftYAxis().negate().deadzone(0.07));
    launcher.setDefaultCommand(launcher.getDefaultCommand());
    intake.setDefaultCommand(
        Commands.run(
            () -> {
              intake.runHopper(controller.getRightYAxis());
              intake.runSpintake(controller.getLeftYAxis());
            },
            intake));

    indexer.configTestControls(controller);
    // intake.configTestController(controller);
    // climb.configTestControls(controller);
    controller
        .createBButton()
        .whileTrue(launcher.getTurretSysIdCommand().finallyDo(() -> launcher.stopAllMotors()));
    controller
        .createAButton()
        .whileTrue(launcher.getFlyWheelSysIdCommand().finallyDo(() -> launcher.stopAllMotors()));

    // SmartTurret tuning commands
    controller
        .createXButton()
        .whileTrue(
            launcher
                .getTurretQuasistaticCommand()
                .finallyDo(() -> launcher.stopAllMotors()));
    controller
        .createYButton()
        .whileTrue(launcher.getTurretKsMapCommand().finallyDo(() -> launcher.stopAllMotors()));
    controller
        .createLeftBumper()
        .whileTrue(
            launcher
                .getTurretDynamicCommand()
                .finallyDo(() -> launcher.stopAllMotors()));
    controller
        .createRightBumper()
        .whileTrue(
            launcher.getTurretTrackingTuneCommand().finallyDo(() -> launcher.stopAllMotors()));

    // // Turret PID tuning — hold right bumper to enter tuning mode
    // controller
    //     .createRightBumper()
    //     .whileTrue(new TurretTuningCommand(launcher).finallyDo(() -> launcher.stopAllMotors()));

    // // D-pad left/right cycles turret presets while tuning
    // controller
    //     .createRightPovButton()
    //     .onTrue(Commands.runOnce(() -> TurretTuningCommand.nextPreset()));
    // controller
    //     .createLeftPovButton()
    //     .onTrue(Commands.runOnce(() -> TurretTuningCommand.previousPreset()));
  }
}
