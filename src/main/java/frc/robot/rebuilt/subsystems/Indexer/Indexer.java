// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Indexer;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.rebuilt.commands.IndexerCommands.IndexerState;
import frc.robot.rebuilt.util.StateMachine;
import org.frc5010.common.sensors.Controller;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  /** Creates a new Index and selects the IO to real or simulated. */
  public Indexer() {
    if (RobotBase.isSimulation()) {
      io = new IndexerIOSim(this);
    } else {
      io = new IndexerIOReal(this);
    }
  }

  public void runSpindexer(double speed) {
    io.runSpindexer(speed);
  }

  public void runFeeder(double speed) {
    io.runTransferFront(speed);
  }

  // public void runTransferBack(double speed) {
  //   io.runTransferBack(speed);
  // }

  public void runTransferFront(double speed) {
    io.runTransferFront(speed);
  }

  public void configTestControls(Controller controller) {
    controller.createLeftBumper().whileTrue((spindexerCommand(.25)).alongWith(feederCommand(0.25)));
  }
  /** Command that runs the feeder at a given speed and stops when done */
  public Command feederCommand(double speed) {
    return Commands.run(
            () -> {
              runFeeder(0.25);
            },
            this)
        .finallyDo(
            () -> {
              runFeeder(0);
            });
  }
  /** returns a command that runs the spindexer at a set speed and stops when done */
  public Command spindexerCommand(double speed) {
    return Commands.run(
            () -> {
              runSpindexer(speed);
            })
        .finallyDo(
            () -> {
              runSpindexer(0);
            });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
  }

  public boolean isRequested(IndexerState state) {
    return inputs.stateRequested.compareTo(state) == 0;
  }

  public boolean isCurrent(IndexerState state) {
    return inputs.stateCurrent.compareTo(state) == 0;
  }

  public void setCurrentState(IndexerState state) {
    inputs.stateCurrent = state;
  }

  public void setRequestedState(IndexerState state) {
    inputs.stateRequested = state;
  }

  public void setDefaultCommands(StateMachine stateMachine) {
    inputs.stateRequested = IndexerState.IDLE;
  }
}
