package frc.robot.rebuilt.commands;

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
      CommandXboxController driver, CommandXboxController operator) {
    Trigger leftTrigger = driver.leftTrigger(Constants.Intake.INTAKE_DEADZONE);
    Trigger rightTrigger = driver.rightTrigger(Constants.Intake.INTAKE_DEADZONE);

    leftTrigger.onTrue(Commands.runOnce(() -> intake.requestDeploy()));
    rightTrigger.onTrue(Commands.runOnce(() -> intake.requestRetract()));

    // Hold to manually zero the hopper after a failed first-deploy: drives toward the deploy hard
    // stop until a current spike, then zeros the encoder. Release before the spike aborts safely.
    operator.rightBumper().whileTrue(intake.manualZeroCommand());

    // driver.rightBumper().onTrue(shouldRetracting());
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
