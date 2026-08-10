package frc.robot.rebuilt.commands.testCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import frc.robot.rebuilt.subsystems.intake.Intake;
import frc.robot.rebuilt.util.Controller;

public class TestCommands {

  Indexer indexer;
  Intake intake;
  static Launcher launcher;

  public TestCommands() {
    indexer = Rebuilt.indexer;
    intake = Rebuilt.intake;
    launcher = Rebuilt.launcher;
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
    controller
        .createBButton()
        .whileTrue(launcher.getTurretSysIdCommand().finallyDo(() -> launcher.stopAllMotors()));
    controller
        .createAButton()
        .whileTrue(launcher.getFlyWheelSysIdCommand().finallyDo(() -> launcher.stopAllMotors()));
  }
}
