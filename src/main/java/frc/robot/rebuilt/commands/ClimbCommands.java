package frc.robot.rebuilt.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.Constants.ClimbConstants;
import frc.robot.rebuilt.subsystems.Climb.Climb;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.arch.StateMachine.State;
import org.frc5010.common.sensors.Controller;

public class ClimbCommands {

  private Map<String, GenericSubsystem> subsystems;
  private StateMachine stateMachine;
  private State idleState;
  private State elevateState;
  private State descendState;
  private State liftedState;
  private State loweredState;
  private State disabledState;
  private State manualState;
  /** defines possible states for the climb */
  public static enum ClimbState {
    IDLE,
    ELEVATE,
    LIFTED,
    DESCEND,
    LOWERED,
    DISABLED,
    MANUAL
  }

  private static Climb climb;
  private DoubleSupplier getOpleftY;

  public ClimbCommands(Map<String, GenericSubsystem> systems) {
    this.subsystems = systems;

    // Create a simple state machine for climb and set it as the default command for the Climb
    climb = (Climb) subsystems.get(Constants.CLIMB);
    if (null == climb) {
      return;
    }
    stateMachine = new StateMachine("ClimbStateMachine");
    // a simple idle state; transitions will be added in configureButtonBindings
    idleState =
        stateMachine.addState(
            "idle",
            Commands.runOnce(() -> climb.runClimb(0))
                .alongWith(Commands.runOnce(() -> climb.setCurrentState(ClimbState.IDLE))));
    loweredState =
        stateMachine.addState(
            "lowered", Commands.runOnce(() -> climb.setCurrentState(ClimbState.LOWERED)));
    liftedState =
        stateMachine.addState(
            "lifted", Commands.runOnce(() -> climb.setCurrentState(ClimbState.LIFTED)));
    disabledState =
        stateMachine.addState(
            "disabled", Commands.runOnce(() -> climb.setCurrentState(ClimbState.DISABLED)));
    manualState =
        stateMachine.addState(
            "manual",
            Commands.runOnce(() -> climb.setCurrentState(ClimbState.MANUAL))
                .alongWith(
                    Commands.run(
                        () ->
                            climb.setDefaultCommand(
                                Commands.run(
                                    () -> {
                                      climb.runClimb(getOpleftY.getAsDouble());
                                    },
                                    climb)))));

    // states that actually run the climber
    if (climb != null) {
      elevateState =
          stateMachine.addState(
              "elevate",
              climb
                  .climberCommand(Meters.of(.5))
                  .alongWith(Commands.runOnce(() -> climb.setCurrentState(ClimbState.ELEVATE))));
      descendState =
          stateMachine.addState(
              "lower",
              climb
                  .climberCommand(Meters.of(0))
                  .alongWith(Commands.runOnce(() -> climb.setCurrentState(ClimbState.DESCEND))));
    } else {
      // fallback states if climb isn't available
      elevateState = stateMachine.addState("elevate", Commands.idle());
      descendState = stateMachine.addState("lower", Commands.idle());
    }

    // Set DISABLED as the default initial state
    stateMachine.setInitialState(disabledState);

    if (climb != null) {
      stateMachine.addRequirements(climb);
      climb.setDefaultCommand(stateMachine);
    }
  }

  public static Command shouldElevateCommand() {
    return Commands.runOnce(() -> climb.setRequestedState(ClimbState.ELEVATE));
  }

  public static Command shouldStopCommand() {
    return Commands.runOnce(() -> climb.setRequestedState(ClimbState.IDLE));
  }

  public static Command shouldDescendCommand() {
    return Commands.runOnce(() -> climb.setRequestedState(ClimbState.DESCEND));
  }

  // New: command to enable the climb (requests IDLE)
  public static Command shouldEnableCommand() {
    return Commands.runOnce(() -> climb.setRequestedState(ClimbState.IDLE));
  }

  public void configureButtonBindings(Controller driver, Controller operator) {
    if (null == climb) {
      return;
    }

    // Bind the "enable climb" button to transition DISABLED -> IDLE when pressed.
    // Change createStartButton() to whatever button you prefer on your controller.
    operator.createStartButton().onTrue(shouldEnableCommand());

    // Driver POV-Up enables the climb (requests IDLE)
    driver.createUpPovButton().onTrue(shouldEnableCommand());

    // disabled -> idle when the enable command is requested
    disabledState.switchTo(idleState).when(() -> climb.isRequested(ClimbState.IDLE));

    configCommonStates(operator);
  }

  public void configureAltButtonBindings(Controller driver, Controller operator) {
    stateMachine.setInitialState(idleState);

    operator.createXButton().onTrue(shouldElevateCommand()).onFalse(shouldStopCommand());
    operator.createYButton().onTrue(shouldDescendCommand()).onFalse(shouldStopCommand());
    // lowered -> elevate when requested

    stateMachine.setInitialState(idleState);

    configCommonStates(operator);
  }

  private void configCommonStates(Controller operator) {

    loweredState.switchTo(elevateState).when(() -> climb.isRequested(ClimbState.ELEVATE));
    // descend -> lowered when height is Zero
    descendState.switchTo(loweredState).when(() -> climb.getHeight().isEquivalent(Meters.of(0)));
    // elevate -> Lifted when height is =to Target
    elevateState
        .switchTo(liftedState)
        .when(() -> climb.getHeight().isEquivalent(ClimbConstants.MAX));
    // lifted -> descend when asked to descend
    liftedState.switchTo(descendState).when(() -> climb.isRequested(ClimbState.DESCEND));
    // elevate -> descend when asked to descend
    elevateState.switchTo(descendState).when(() -> climb.isRequested(ClimbState.DESCEND));
    // descend -> elevating when asked to elevate
    descendState.switchTo(elevateState).when(() -> climb.isRequested(ClimbState.ELEVATE));
    // elevate -> idle when stopped
    elevateState.switchTo(idleState).when(() -> climb.isRequested(ClimbState.IDLE));
    // descend -> idle when stopped
    descendState.switchTo(idleState).when(() -> climb.isRequested(ClimbState.IDLE));
    // idle -> elevate when requested
    idleState.switchTo(elevateState).when(() -> climb.isRequested(ClimbState.ELEVATE));
    // idle -> descend when requested
    idleState.switchTo(descendState).when(() -> climb.isRequested(ClimbState.DESCEND));
    // idle- > manual
    idleState.switchTo(manualState).when(() -> operator.getLeftYAxis() != 0);
    // manual to switch
    manualState.switchTo(idleState).when(() -> operator.getLeftYAxis() == 0);

    getOpleftY = () -> operator.getLeftYAxis();
  }
}
