package frc.robot.rebuilt.commands.testCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import frc.robot.rebuilt.subsystems.intake.Intake;

public class TestCommands {

  Indexer indexer;
  Intake intake;
  static Launcher launcher;

  public TestCommands() {
    indexer = Rebuilt.indexer;
    intake = Rebuilt.intake;
    launcher = Rebuilt.launcher;
  }

  public void configureButtonBindings(CommandXboxController controller) {
    launcher.setDefaultCommand(launcher.getDefaultCommand());
    intake.setDefaultCommand(
        Commands.run(
            () -> {
              intake.runHopper(MathUtil.applyDeadband(-controller.getRightY(), 0.07));
              intake.runSpintake(MathUtil.applyDeadband(-controller.getLeftY(), 0.07));
            },
            intake));

    indexer.configTestControls(controller);
    // intake.configTestController(controller);
    controller
        .b()
        .whileTrue(launcher.getTurretSysIdCommand().finallyDo(() -> launcher.stopAllMotors()));
    controller
        .a()
        .whileTrue(launcher.getFlyWheelSysIdCommand().finallyDo(() -> launcher.stopAllMotors()));
  }
}
