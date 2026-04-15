package frc.robot.rebuilt.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.subsystems.intake.Intake;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.sensors.Controller;

public class IntakeCommands {
  static Intake intake;
  static boolean positionFound = false;
  Map<String, GenericSubsystem> subsystems;
  StateMachine intakeStateMachine = new StateMachine("IntakeStateMachine");

  DoubleSupplier intakeSpeedSupplier =
      () -> Constants.Intake.INTAKE_IN; // Default speed, can be overridden by triggers
  Supplier<DoubleSupplier> intakeSpeed = () -> intakeSpeedSupplier;

  public static enum IntakeState {
    UNKNOWN,
    RETRACTED,
    RETRACTING,
    DEPLOYING,
    INTAKING,
    DEPLOYED,
    ANGLED;
  }

  public IntakeCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;

    intake = (Intake) subsystems.get(Constants.INTAKE);

    setupTriggerStates();
  }

  public void setupDefaultCommands() {}

  private void setupTriggerStates() {
    // Map requested states to their commands and wire triggers in a compact loop.
    // CHURN is handled separately below so it can be gated on flywheel readiness.
    java.util.Map<IntakeState, Command> stateToCommand =
        java.util.Map.of(
            IntakeState.UNKNOWN, unknownStateCommand(),
            IntakeState.RETRACTED, retractedCommand(),
            IntakeState.DEPLOYED, deployedCommand());

    stateToCommand.forEach(
        (state, cmd) -> new Trigger(() -> intake.isRequested(state)).onTrue(cmd));

    // Not moving trigger senses if the hopper has hit the bumper hard stop for 0.5 sec
    Trigger hopperNotMovingAndLowish =
        new Trigger(() -> intake.isHopperStalling()).debounce(Constants.Intake.HOPPER_STALL_TIME);

    /** Trigger the deploying command */
    new Trigger(
            () ->
                intake.isRequested(IntakeState.INTAKING) && !intake.isCurrent(IntakeState.INTAKING))
        .onTrue(deployingCommand().until(() -> intake.isCurrent(IntakeState.INTAKING)));

    /** Trigger the intaking command */
    new Trigger(
            () ->
                intake.isRequested(IntakeState.INTAKING)
                    && intake.isCurrent(IntakeState.DEPLOYING)
                    && (intake.isDeployed() || hopperNotMovingAndLowish.getAsBoolean()))
        .onTrue(intakingCommand(intakeSpeed));

    /** Trigger the retracting command */
    new Trigger(
            () ->
                intake.isRequested(IntakeState.RETRACTING)
                    && !intake.isCurrent(IntakeState.RETRACTED))
        .onTrue(retractingCommand().until(() -> intake.isCurrent(IntakeState.RETRACTED)));

    /** Trigger the retracted command */
    new Trigger(() -> intake.isCurrent(IntakeState.RETRACTING) && (intake.isRetracted()))
        .onTrue(shouldRetracted());

    /** Trigger the angled command */
    new Trigger(
            () -> intake.isRequested(IntakeState.ANGLED) && !intake.isCurrent(IntakeState.ANGLED))
        .onTrue(angledCommand());
  }

  public void configureButtonBindings(Controller controller, Controller operator) {
    controller.setRightTrigger(
        controller.createRightTrigger().limit(Constants.Intake.INTAKE_MAX_IN));
    Trigger rightTrigger =
        new Trigger(() -> controller.getRightTrigger() > Constants.Intake.INTAKE_DEADZONE);
    controller.setLeftTrigger(
        controller
            .createLeftTrigger()
            .limit(Constants.Intake.INTAKE_MAX_IN)); // Axis are positive only hence IN
    Trigger leftTrigger =
        new Trigger(() -> controller.getLeftTrigger() > Constants.Intake.INTAKE_DEADZONE);

    rightTrigger.onTrue(shouldIntaking());
    leftTrigger.onTrue(shouldIntaking());

    controller.createRightBumper().onTrue(shouldRetracting());
    controller.createStartButton().onTrue(Commands.run(() -> intake.setHopperRetracted()));
    controller.createBackButton().onTrue(Commands.run(() -> intake.setHopperDeployed()));

    operator.createDownPovButton().onTrue(operatorHopperDownCommand());
    controller.createXButton().onTrue(operatorHopperDownCommand());
    operator.createRightBumper().onTrue(shouldAngled()).onFalse(shouldIntaking());

    intakeSpeedSupplier =
        () -> {
          double rightTriggerSpeed = controller.getRightTrigger();
          double leftTriggerSpeed = controller.getLeftTrigger();
          double speed = Constants.Intake.INTAKE_IN; // Default speed if neither trigger is pressed
          if (rightTriggerSpeed > Constants.Intake.INTAKE_DEADZONE
              || leftTriggerSpeed > Constants.Intake.INTAKE_DEADZONE) {
            speed =
                rightTriggerSpeed
                    - leftTriggerSpeed; // Positive for intaking, negative for outtaking
          }
          if (RobotState.isAutonomous()) {
            speed = Constants.Intake.INTAKE_AUTO; // Intake in speed in auto
          }
          return speed;
        };
  }

  public static Command intakingCommand(Supplier<DoubleSupplier> speed) {
    return Commands.runOnce(
            () -> {
              intake.setCurrentState(IntakeState.INTAKING);
              positionFound = true;
              intake.setHopperPosition(Degrees.of(0));
            },
            intake)
        .andThen(
            Commands.run(
                () -> {
                  double runSpeed = speed.get().getAsDouble();

                  if (intake.getHopperAngle().gt(Degrees.of(5))
                      || runSpeed > Constants.Intake.INTAKE_IN
                      || RobotState.isAutonomous()) {
                    intake.runHopper(-0.15);
                  } else {
                    intake.setDesiredHopperAngle(Constants.Intake.HOPPER_DEPLOYED_ANGLE);
                  }
                  intake.runSpintake(runSpeed);
                },
                intake));
  }

  public static Command waitUntilIntaking() {
    return Commands.idle().until(() -> intake.isCurrent(IntakeState.INTAKING));
  }

  public static Command deployingCommand() {
    return Commands.runOnce(
            () -> {
              intake.setCurrentState(IntakeState.DEPLOYING);
              intake.setHopperPosition(Constants.Intake.HOPPER_RETRACTED_ANGLE);
            },
            intake)
        .andThen(
            intake
                .setDesiredHopperAngle(Constants.Intake.HOPPER_DEPLOYED_ANGLE)
                .until(() -> intake.isHopperAtGoal())
                .andThen(
                    Commands.runOnce(
                        () -> {
                          intake.runHopper(-0.1);
                        })))
        .alongWith(
            Commands.run(
                () -> {
                  if (intake.getHopperAngle().lt(Degrees.of(60))) {
                    intake.runSpintake(Constants.Intake.INTAKE_IN);
                  }
                }));
  }

  public static Command deployedCommand() {
    return Commands.runOnce(
        () -> {
          intake.setCurrentState(IntakeState.DEPLOYED);
          intake.runHopper(0);
        },
        intake);
  }

  public static Command angledCommand() {
    return Commands.runOnce(
            () -> {
              intake.setCurrentState(IntakeState.ANGLED);
            },
            intake)
        .andThen(
            intake
                .setDesiredHopperAngle(Constants.Intake.HOPPER_ANGLED)
                .until(() -> intake.isHopperMoving())
                .andThen(
                    Commands.run(
                        () ->
                            intake.runSpintakes(
                                Constants.Intake.INTAKE_IN * 0.5, Constants.Intake.INTAKE_CHURN),
                        intake)));
  }

  public static Command retractingCommand() {
    return Commands.runOnce(
            () -> {
              intake.setCurrentState(IntakeState.RETRACTING);
            },
            intake)
        .andThen(() -> intake.runSpintake(0), intake)
        .andThen(
            intake
                .setDesiredHopperAngle(Constants.Intake.HOPPER_RETRACTED_ANGLE)
                .until(() -> intake.isHopperMoving()));
  }

  public static Command retractedCommand() {
    return Commands.runOnce(() -> intake.setCurrentState(IntakeState.RETRACTED))
        .andThen(() -> intake.runSpintake(0), intake)
        .andThen(() -> intake.runHopper(0), intake);
  }

  public static Command unknownStateCommand() {
    return Commands.runOnce(
            () -> {
              intake.setCurrentState(IntakeState.UNKNOWN);
            })
        .andThen(Commands.runOnce(() -> intake.runHopper(0), intake))
        .andThen(Commands.runOnce(() -> intake.runSpintake(0), intake));
  }

  public Command operatorHopperDownCommand() {
    return Commands.run(
            () -> {
              intake.runHopper(-.2);
            })
        .until(() -> intake.isHopperStalling())
        .andThen(intakingCommand(intakeSpeed));
  }

  public static Command shouldIntaking() {
    return Commands.runOnce(() -> intake.setRequestedState(IntakeState.INTAKING));
  }

  public static Command shouldRetracting() {
    return Commands.runOnce(() -> intake.setRequestedState(IntakeState.RETRACTING));
  }

  public static Command shouldRetracted() {
    return Commands.runOnce(() -> intake.setRequestedState(IntakeState.RETRACTED));
  }

  public static Command shouldAngled() {
    return Commands.runOnce(() -> intake.setRequestedState(IntakeState.ANGLED));
  }
}
