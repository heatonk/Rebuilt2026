// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.rebuilt.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  public enum HopperState {
    UNKNOWN,
    DEPLOYING,
    DEPLOYED,
    RETRACTING,
    RETRACTED
  }

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private HopperState state = HopperState.UNKNOWN;
  private boolean hopperZeroed = false;
  private double requestedSpintakeSpeed = 0;

  private final Debouncer hardStopDebouncer =
      new Debouncer(Constants.Intake.HOPPER_STALL_TIME, Debouncer.DebounceType.kRising);

  // Used to bound the open-loop nudge phase when the hopper is already zeroed.
  private boolean deployNudgePhase = false;
  private final Timer deployNudgeTimer = new Timer();

  public Intake() {
    if (RobotBase.isSimulation()) {
      io = new IntakeIOSim(this);
    } else {
      io = new IntakeIOReal(this);
    }
  }

  // === Public API ===

  /** Request the hopper to deploy. No-op if already deploying or deployed. */
  public void requestDeploy() {
    if (state != HopperState.DEPLOYING && state != HopperState.DEPLOYED) {
      deployNudgePhase = false;
      state = HopperState.DEPLOYING;
    }
  }

  /** Request the hopper to retract. No-op if already retracting or retracted. */
  public void requestRetract() {
    if (state != HopperState.RETRACTING && state != HopperState.RETRACTED) {
      state = HopperState.RETRACTING;
    }
  }

  /**
   * Set the desired spintake speed. Only applied when the hopper is fully deployed; the subsystem
   * enforces zero otherwise. Call each loop from a trigger-bound command while active.
   */
  public void setSpintakeSpeed(double speed) {
    requestedSpintakeSpeed = speed;
  }

  public HopperState getState() {
    return state;
  }

  public boolean isCurrent(HopperState s) {
    return state == s;
  }

  public boolean isDeployed() {
    return state == HopperState.DEPLOYED;
  }

  public boolean isRetracted() {
    return state == HopperState.RETRACTED;
  }

  public Angle getHopperAngle() {
    return inputs.hopperAngleActual;
  }

  /** Direct hopper motor control. Bypasses the state machine — for test mode only. */
  public void runHopper(double speed) {
    io.runHopper(speed);
  }

  /** Direct spintake motor control. Bypasses the state machine — for test mode only. */
  public void runSpintake(double speed) {
    io.runSpintake(speed);
  }

  public Command getHopperSysIdCommand() {
    return io.getHopperSysIdCommand();
  }

  public Command getHopperCharacterizationCommand() {
    return io.getHopperCharacterizationCommand(this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    inputs.hopperZeroed = hopperZeroed;
    Logger.processInputs("Intake", inputs);

    boolean hardStop = hardStopDebouncer.calculate(inputs.hopperHardStopDetected);

    // Safety: if zeroed and the encoder drifts past the hard stop, re-zero.
    if (hopperZeroed && inputs.hopperAngleActual.in(Degrees) < Constants.Intake.HOPPER_AUTO_REZERO_THRESHOLD) {
      io.setHopperPosition(Degrees.of(0));
    }

    // Safety: if zeroed, deployed-side, and hard stop is detected but encoder is slightly off, correct it.
    if (hopperZeroed
        && (state == HopperState.DEPLOYING || state == HopperState.DEPLOYED)
        && hardStop
        && inputs.hopperAngleActual.in(Degrees) < Constants.Intake.HOPPER_DEPLOY_STOP_REZERO_MAX_ANGLE
        && inputs.hopperAngleActual.in(Degrees) > Constants.Intake.HOPPER_ANGLE_TOLERANCE) {
      io.setHopperPosition(Degrees.of(0));
    }

    switch (state) {
      case UNKNOWN:
        io.runSpintake(0);
        io.runHopper(0);
        break;

      case DEPLOYING:
        io.runSpintake(0);
        if (!hopperZeroed) {
          // Open-loop homing: drive toward the deployed hard stop to find the zero reference.
          io.runHopper(Constants.Intake.HOPPER_FIRST_DEPLOY_DUTY);
          if (hardStop) {
            markZeroed();
            state = HopperState.DEPLOYED;
          }
        } else if (!deployNudgePhase) {
          // PID to the deployed angle target.
          io.setHopperTarget(Constants.Intake.HOPPER_DEPLOYED_ANGLE);
          if (io.isHopperAtLocation(Constants.Intake.HOPPER_DEPLOYED_ANGLE)) {
            deployNudgePhase = true;
            deployNudgeTimer.restart();
          }
        } else {
          // Nudge open-loop to confirm contact with the mechanical hard stop.
          io.runHopper(Constants.Intake.HOPPER_DEPLOY_NUDGE_DUTY);
          if (hardStop || deployNudgeTimer.hasElapsed(1.5)) {
            io.runHopper(0);
            deployNudgePhase = false;
            state = HopperState.DEPLOYED;
          }
        }
        break;

      case DEPLOYED:
        // Hold hopper at deployed position.
        if (inputs.hopperAngleActual.gt(Degrees.of(5)) || RobotState.isAutonomous()) {
          io.runHopper(Constants.Intake.HOPPER_FIRST_DEPLOY_DUTY);
        } else {
          io.setHopperTarget(Constants.Intake.HOPPER_DEPLOYED_ANGLE);
        }
        io.runSpintake(requestedSpintakeSpeed);
        break;

      case RETRACTING:
        io.runSpintake(0);
        io.setHopperTarget(Constants.Intake.HOPPER_RETRACTED_ANGLE);
        if (io.isRetracted()) {
          state = HopperState.RETRACTED;
        }
        break;

      case RETRACTED:
        io.runSpintake(0);
        io.setHopperTarget(Constants.Intake.HOPPER_RETRACTED_ANGLE);
        break;
    }

    Logger.recordOutput("Intake/State", state.toString());
    Logger.recordOutput("Intake/HopperZeroed", hopperZeroed);
    Logger.recordOutput("Intake/HardStopDebounced", hardStop);
    Logger.recordOutput("Intake/RequestedSpintakeSpeed", requestedSpintakeSpeed);
  }

  private void markZeroed() {
    io.setHopperPosition(Degrees.of(0));
    hopperZeroed = true;
  }
}
