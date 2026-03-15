package frc.robot.rebuilt.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.subsystems.intake.Intake;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.arch.StateMachine.State;
import org.frc5010.common.sensors.Controller;

public class IntakeCommands {
  static Intake intake;
  Map<String, GenericSubsystem> subsystems;
  StateMachine intakeStateMachine = new StateMachine("IntakeStateMachine");
  /** Declaring states of the intake */
  State unknown = intakeStateMachine.addState("unknown", unknownStateCommand());

  State retracted = intakeStateMachine.addState("retracted", retractedCommand());

  State retracting = intakeStateMachine.addState("retracting", retractingCommand());
  State deploying = intakeStateMachine.addState("deploying", deployingCommand());
  State deployed =
      intakeStateMachine.addState(
          "deployed",
          Commands.run(
              () -> {
                intake.setCurrentState(IntakeState.DEPLOYED);
                intake.runHopper(0);
              }));
  State outtaking;
  State intaking;
  DoubleSupplier intakeSpeedSupplier = () -> 0.65;
  DoubleSupplier outtakeSpeedSupplier = () -> -0.5;

  public static enum IntakeState {
    UNKNOWN,
    RETRACTED,
    RETRACTING,
    DEPLOYING,
    INTAKING,
    OUTTAKING,
    DEPLOYED;
  }

  public IntakeCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;

    intake = (Intake) subsystems.get(Constants.INTAKE);

    setupStateMachine();
  }

  public void setupDefaultCommands() {
    intake.setDefaultCommand(intakeStateMachine);
  }

  private void setupStateMachine() {
    // For auto - these get replaced with versions that take controller input instead of constants
    outtaking = intakeStateMachine.addState("outtaking", outtakingCommand(outtakeSpeedSupplier));

    intaking = intakeStateMachine.addState("intaking", intakingCommand(intakeSpeedSupplier));

    // Not moving trigger senses if the hopper has hit the bumper hard stop for 0.5 sec
    Trigger hopperNotMoving =
        new Trigger(() -> intake.isHopperStalling())
            .debounce(0.5)
            .onTrue(Commands.runOnce(() -> intake.setHopperPosition(Degrees.of(0))));

    // If the hopper is not moving and we want to intake or outtake, set the hopper angle to 0 to
    // prevent jamming
    // hopperNotMoving
    //     .and(() -> intake.isRequested(IntakeState.INTAKING))
    //     .onTrue(Commands.runOnce(() -> intake.setHopperPosition(Degrees.of(0))));
    // hopperNotMoving
    //     .and(() -> intake.isRequested(IntakeState.OUTTAKING))
    //     .onTrue(Commands.runOnce(() -> intake.setHopperPosition(Degrees.of(0))));

    // Get out of the unknown state when we deploy so it hits the hardstop
    unknown.switchTo(deploying).when(() -> intake.isRequested(IntakeState.INTAKING));
    unknown.switchTo(deploying).when(() -> intake.isRequested(IntakeState.OUTTAKING));

    deployed.switchTo(retracting).when(() -> intake.isRequested(IntakeState.RETRACTING));
    deployed.switchTo(outtaking).when(() -> intake.isRequested(IntakeState.OUTTAKING));
    deployed.switchTo(intaking).when(() -> intake.isRequested(IntakeState.INTAKING));

    retracted.switchTo(deploying).when(() -> intake.isRequested(IntakeState.INTAKING));
    deploying
        .switchTo(intaking)
        .when(() -> hopperNotMoving.getAsBoolean() && intake.isRequested(IntakeState.INTAKING));

    retracting.switchTo(deploying).when(() -> intake.isRequested(IntakeState.INTAKING));
    retracting.switchTo(deploying).when(() -> intake.isRequested(IntakeState.OUTTAKING));

    intaking.switchTo(retracting).when(() -> intake.isRequested(IntakeState.RETRACTING));
    intaking.switchTo(outtaking).when(() -> intake.isRequested(IntakeState.OUTTAKING));

    outtaking.switchTo(retracting).when(() -> intake.isRequested(IntakeState.RETRACTING));
    outtaking.switchTo(intaking).when(() -> intake.isRequested(IntakeState.INTAKING));

    retracted.switchTo(deploying).when(() -> intake.isRequested(IntakeState.OUTTAKING));
    deploying
        .switchTo(outtaking)
        .when(() -> hopperNotMoving.getAsBoolean() && intake.isRequested(IntakeState.OUTTAKING));

    retracting.switchTo(retracted).when(() -> intake.isRetracted());
    deployed.switchTo(retracted).when(() -> intake.isRetracted());

    deploying.switchTo(deployed).when(() -> intake.isRequested(IntakeState.DEPLOYED));
    outtaking.switchTo(deployed).when(() -> intake.isRequested(IntakeState.DEPLOYED));
    intaking.switchTo(deployed).when(() -> intake.isRequested(IntakeState.DEPLOYED));
    retracted.switchTo(deployed).when(() -> intake.isRequested(IntakeState.DEPLOYED));
    retracting.switchTo(deployed).when(() -> intake.isRequested(IntakeState.DEPLOYED));

    intakeStateMachine.setInitialState(unknown);
    if (intake != null) {
      intakeStateMachine.addRequirements(intake);
    }
  }

  public void configureButtonBindings(Controller controller) {
    controller.setRightTrigger(
        controller.createRightTrigger().limit(Constants.Intake.INTAKE_MAX_IN));
    Trigger rightTrigger = new Trigger(() -> controller.getRightTrigger() > 0.25);
    controller.setLeftTrigger(
        controller
            .createLeftTrigger()
            .limit(Constants.Intake.INTAKE_MAX_IN)); // Axis are positive only hence IN
    Trigger leftTrigger = new Trigger(() -> controller.getLeftTrigger() > 0.25);

    rightTrigger.whileTrue(shouldIntaking());
    leftTrigger.whileTrue(shouldOuttaking());

    controller.createRightBumper().onTrue(shouldRetracting()).onFalse(shouldRetracted());

    controller.createStartButton().onTrue(Commands.run(() -> intake.setHopperRetracted()));
    controller.createBackButton().onTrue(Commands.run(() -> intake.setHopperDeployed()));

    outtakeSpeedSupplier = () -> controller.getLeftTrigger();

    intakeSpeedSupplier = () -> controller.getRightTrigger();
  }

  public static Command outtakingCommand(DoubleSupplier speed) {
    return Commands.runOnce(() -> intake.setCurrentState(IntakeState.OUTTAKING))
        .andThen(Commands.runOnce(() -> intake.runHopper(0)))
        // .andThen(() -> intake.setHopperAngle(Degrees.of(0.0)))
        .andThen(Commands.run(() -> intake.runSpintake(-speed.getAsDouble())));
    // Math.max(
    //     Constants.Intake.INTAKE_MAX_OUT,
    //     Math.min(-speed.getAsDouble(), Constants.Intake.INTAKE_OUT)))));
    // assuming outtaking is just intaking but goes the other way
  }

  public static Command intakingCommand(DoubleSupplier speed) {
    return Commands.runOnce(() -> intake.setCurrentState(IntakeState.INTAKING))
        .andThen(Commands.runOnce(() -> intake.runHopper(0)))
        // .andThen(() -> intake.setHopperAngle(Degrees.of(0.0)))
        .andThen(Commands.run(() -> intake.runSpintake(speed.getAsDouble())));
    // Math.min(
    //     Constants.Intake.INTAKE_MAX_IN,
    //     Math.max(speed.getAsDouble(), Constants.Intake.INTAKE_IN)))));
  }

  public static Command deployingCommand() {
    return Commands.runOnce(() -> intake.setCurrentState(IntakeState.DEPLOYING))
        .andThen(() -> intake.runHopper(Constants.Intake.HOPPER_GO_OUT));
  }

  public static Command retractingCommand() {
    return Commands.runOnce(
            () -> {
              intake.setCurrentState(IntakeState.RETRACTING);
              LauncherCommands.shouldIdleCommand();
            })
        // .andThen(() -> intake.setHopperAngle(Degrees.of(130)))
        .andThen(() -> intake.runHopper(Constants.Intake.HOPPER_GO_IN))
        .andThen(() -> intake.runSpintake(0));
  }

  public static Command retractedCommand() {
    return Commands.runOnce(() -> intake.setCurrentState(IntakeState.RETRACTED))
        .andThen(() -> intake.runHopper(0))
        // .andThen(() -> intake.setHopperAngle(Degrees.of(130.0)))
        .andThen(() -> intake.runSpintake(0));
  }

  public static Command unknownStateCommand() {
    return Commands.runOnce(() -> intake.setCurrentState(IntakeState.UNKNOWN))
        .andThen(() -> intake.runHopper(0))
        // .andThen(() -> intake.setHopperAngle(Degrees.of(130.0)))
        .andThen(() -> intake.runSpintake(0));
  }

  public static Command shouldOuttaking() {
    return Commands.runOnce(() -> intake.setRequestedState(IntakeState.OUTTAKING));
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
}
