package frc.robot.rebuilt.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Constants;
// import frc.robot.rebuilt.subsystems.Climb.Climb;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import frc.robot.rebuilt.subsystems.intake.Intake;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.sensors.camera.QuestNavInterface;

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

    QuestNavInterface calibrationQuest = new QuestNavInterface(new Transform3d());
    controller
        .createYButton()
        .whileTrue(
            calibrationQuest.determineOffsetToRobotCenter(
                (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN)));

    controller
        .createXButton()
        .onTrue(
            Commands.run(
                () -> {
                  calibrationQuest.resetPose(new Pose3d());
                }));

    // Shot tuning command – hold Y button to enter tuning mode
    // controller
    //     .createYButton()
    //     .whileTrue(
    //         ShotCalibrationCommand.createWithFeed(
    //                 launcher, frc.robot.rebuilt.Rebuilt.drivetrain, 2.0, 0.5)
    //             .finallyDo(() -> launcher.stopAllMotors()));
  }
}
