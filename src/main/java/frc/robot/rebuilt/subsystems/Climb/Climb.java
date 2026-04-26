// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Climb;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.commands.ClimbCommands.ClimbState;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.sensors.Controller;
import org.littletonrobotics.junction.Logger;

/** The class climb controlls the climb */
public class Climb extends GenericSubsystem {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  /** Creates the climb subsystem and chooses the IO */
  public Climb() {
    super("climb.json");
    if (RobotBase.isSimulation()) {
      io = new ClimbIOSim(devices);
    } else {
      io = new ClimbIOReal(devices);
    }
  }
  /** Sets a command that holds the climb at a given height */
  public Command climberCommand(Distance height) {
    return Commands.run(
            () -> {
              setClimbHeight(height);
            })
        .finallyDo(
            () -> {
              /** Resets to 0 when inactive */
              setClimbHeight(Meters.of(0));
            });
  }
  /** Sets the climber io to idle */
  public Command idleCommand() {
    return Commands.runOnce(
        () -> {
          io.idle();
        },
        this);
  }

  public void configTestControls(Controller controller) {
    controller.createBButton().whileTrue(climberCommand(Meters.of(.5)));
  }

  public void setClimbHeight(Distance height) {
    io.setHeight(height);
  }

  public Distance getHeight() {
    return inputs.climbHeight;
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
  }

  public void runClimb(double speed) {
    io.runClimb(speed);
  }

  public boolean isRequested(ClimbState state) {
    return inputs.stateRequested == state;
  }

  public boolean isCurrent(ClimbState state) {
    return inputs.stateCurrent == state;
  }

  public void setCurrentState(ClimbState state) {
    inputs.stateCurrent = state;
  }

  public void setRequestedState(ClimbState state) {
    inputs.stateRequested = state;
  }
}
