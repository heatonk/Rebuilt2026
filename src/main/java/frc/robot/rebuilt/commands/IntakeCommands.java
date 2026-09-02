package frc.robot.rebuilt.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.subsystems.intake.Intake;

public class IntakeCommands {
  private static Intake intake;

  public IntakeCommands() {
    intake = Rebuilt.intake;
  }

  public void setupDefaultCommands() {}

  public void configureButtonBindings(
      CommandXboxController controller, CommandXboxController operator) {
    Trigger rightTrigger = controller.rightTrigger(Constants.Intake.INTAKE_DEADZONE);
    Trigger leftTrigger = controller.leftTrigger(Constants.Intake.INTAKE_DEADZONE);
    Trigger eitherTrigger = rightTrigger.or(leftTrigger);

    eitherTrigger
        .onTrue(Commands.runOnce(() -> intake.requestDeploy()))
        .whileTrue(
            Commands.run(
                () -> {
                  double right =
                      Math.min(
                          controller.getRightTriggerAxis(), Constants.Intake.INTAKE_MAX_IN);
                  double left =
                      Math.min(
                          controller.getLeftTriggerAxis(), Constants.Intake.INTAKE_MAX_IN);
                  double speed;
                  if (RobotState.isAutonomous()) {
                    speed = Constants.Intake.INTAKE_AUTO;
                  } else if (right > Constants.Intake.INTAKE_DEADZONE
                      || left > Constants.Intake.INTAKE_DEADZONE) {
                    speed = right - left;
                  } else {
                    speed = Constants.Intake.INTAKE_IN;
                  }
                  intake.setSpintakeSpeed(speed);
                }))
        .onFalse(Commands.runOnce(() -> intake.setSpintakeSpeed(0)));

    controller.rightBumper().onTrue(shouldRetracting());
  }

  // Named-command helpers — kept for PathPlanner / AutoCommands compatibility.

  public static Command shouldIntaking() {
    return Commands.runOnce(() -> intake.requestDeploy());
  }

  public static Command shouldRetracting() {
    return Commands.runOnce(() -> intake.requestRetract());
  }

  public static Command shouldRetracted() {
    return Commands.runOnce(() -> intake.requestRetract());
  }

  public static Command waitUntilIntaking() {
    return Commands.idle().until(() -> intake.isDeployed());
  }
}
