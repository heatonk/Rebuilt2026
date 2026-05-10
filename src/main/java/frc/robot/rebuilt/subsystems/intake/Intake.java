// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.commands.IntakeCommands;
import frc.robot.rebuilt.commands.IntakeCommands.IntakeState;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.sensors.Controller;
import org.littletonrobotics.junction.Logger;

public class Intake extends GenericSubsystem {
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new Intake and selects the IO */
  public Intake() {
    super("intake.json");
    if (RobotBase.isSimulation()) {
      io = new IntakeIOSim(devices);
    } else {
      io = new IntakeIOReal(devices);
    }
  }

  public void runSpintake(double speed) {
    io.runSpintake(speed);
  }

  public void runSpintakes(double outerSpeed, double innerSpeed) {
    io.runSpintakes(innerSpeed, outerSpeed);
  }

  /** Creates a command that runs the spintake at the given speed and stops when done */
  public Command spintakeCommand(double speed) {
    return Commands.run(
            () -> {
              runSpintake(speed);
            })
        .finallyDo(
            () -> {
              runSpintake(0);
            });
  }

  public Command setDesiredHopperAngle(Angle angle) {
    return io.setHopperAngle(angle);
  }

  public Angle getHopperAngle() {
    return inputs.hopperAngleActual;
  }

  public boolean isRetracted() {
    return io.isRetracted();
  }

  public boolean isDeployed() {
    return io.isDeployed();
  }

  public void runHopper(double speed) {
    io.runHopper(speed);
  }
  /** Configures test controller bindings for the spintake, hopper control, and sysid */
  public void configTestController(Controller controller) {
    controller.createRightBumper().whileTrue(spintakeCommand(0.5));
    controller.createYButton().whileTrue(getHopperSysIdCommand());
    controller.setRightYAxis(controller.createRightYAxis());
    Trigger rightYAxis = new Trigger(() -> controller.getRightYAxis() > 0.01);
    rightYAxis.whileTrue(Commands.run(() -> runHopper(controller.getRightYAxis())));
  }
  /** Updates intake inputs from the io periodically and logs them each robot cycle */
  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);

    boolean autoRezeroTriggered = shouldAutoRezeroAtDeployHardStop() || shouldAutoRezeroPastStop();
    if (autoRezeroTriggered) {
      io.setHopperPosition(Degrees.of(0));
    }
    Logger.recordOutput("Intake/AutoRezeroTriggered", autoRezeroTriggered);

    Logger.processInputs("Intake", inputs);
  }

  private boolean shouldAutoRezeroAtDeployHardStop() {
    boolean deploySideState =
        inputs.stateRequested == IntakeState.INTAKING
            || inputs.stateRequested == IntakeState.DEPLOYED
            || inputs.stateCurrent == IntakeState.DEPLOYING
            || inputs.stateCurrent == IntakeState.INTAKING
            || inputs.stateCurrent == IntakeState.DEPLOYED;

    return inputs.hopperZeroed
        && deploySideState
        && inputs.hopperHardStopDetected
        && inputs.hopperAngleActual.in(Degrees)
            < Constants.Intake.HOPPER_DEPLOY_STOP_REZERO_MAX_ANGLE
        && inputs.hopperAngleActual.in(Degrees) > Constants.Intake.HOPPER_ANGLE_TOLERANCE;
  }

  private boolean shouldAutoRezeroPastStop() {
    return inputs.hopperZeroed
        && inputs.hopperAngleActual.in(Degrees) < Constants.Intake.HOPPER_AUTO_REZERO_THRESHOLD;
  }

  public boolean isRequested(IntakeState state) {
    return inputs.stateRequested == state;
  }

  public boolean isCurrent(IntakeState state) {
    return inputs.stateCurrent == state;
  }

  public boolean isNearTrench() {
    return isCurrent(IntakeState.DEPLOYING) && io.isNearTrench();
  }

  public void setCurrentState(IntakeState state) {
    inputs.stateCurrent = state;
  }

  public IntakeState getCurrentState() {
    return inputs.stateCurrent;
  }

  public boolean isHopperStalling() {
    return io.isHopperStalling();
  }

  public Command getHopperSysIdCommand() {
    return io.getHopperSysIdCommand();
  }

  public Command getHopperCharacterizationCommand() {
    return io.getHopperCharacterizationCommand(this);
  }

  public void setRequestedState(IntakeState state) {
    inputs.stateRequested = state;
  }

  public void setHopperDeployed() {
    io.setHopperPosition(Degrees.of(0));
    setRequestedState(IntakeCommands.IntakeState.DEPLOYED);
  }

  public void setHopperRetracted() {
    io.setHopperPosition(Degrees.of(120));
    setRequestedState(IntakeCommands.IntakeState.RETRACTED);
  }

  public boolean isHopperMoving() {
    return io.isHopperMoving();
  }

  public boolean isHopperHardStopDetected() {
    return inputs.hopperHardStopDetected;
  }

  public void setHopperPosition(Angle angle) {
    io.setHopperPosition(angle);
  }

  /** Zeroes the hopper encoder to 0° (the deployed/hard-stop position). */
  public void zeroHopper() {
    io.setHopperPosition(Degrees.of(0));
  }

  public boolean isHopperZeroed() {
    return inputs.hopperZeroed;
  }

  public void setHopperZeroed(boolean zeroed) {
    inputs.hopperZeroed = zeroed;
  }

  public boolean isHopperAtGoal() {
    return inputs.hopperAtGoal;
  }

  public boolean isHopperAtPosition(Angle angle) {
    return io.isHopperAtLocation(angle);
  }
}
