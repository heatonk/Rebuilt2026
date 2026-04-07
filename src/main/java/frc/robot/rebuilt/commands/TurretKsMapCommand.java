package frc.robot.rebuilt.commands;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rebuilt.subsystems.Launcher.SmartTurretController;
import org.frc5010.common.arch.GenericSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Generates a position-dependent kS map for the turret by probing at multiple positions.
 *
 * <p>At each test position, ramps TorqueCurrentFOC in both positive and negative directions until
 * movement is detected, recording the minimum current needed. The result is two {@link
 * InterpolatingDoubleTreeMap}s (positive and negative direction) that can be injected into the
 * SmartTurretController for dynamic kS compensation.
 */
public class TurretKsMapCommand extends Command {

  private enum State {
    MOVE_TO_POSITION,
    SETTLE,
    PROBE_POSITIVE,
    PROBE_NEGATIVE,
    RECORD_AND_ADVANCE,
    DONE
  }

  private final SmartTurretController controller;
  private final TalonFX talonFX;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final MotionMagicTorqueCurrentFOC moveRequest =
      new MotionMagicTorqueCurrentFOC(0).withSlot(0);
  private final TorqueCurrentFOC probeRequest = new TorqueCurrentFOC(0);
  private final Timer settleTimer = new Timer();

  private final double[] testPositions;
  private int currentIndex = 0;
  private State currentState = State.MOVE_TO_POSITION;

  private double probeCurrent = 0.0;
  private double ksPositive = 0.0;
  private double ksNegative = 0.0;

  private static final double PROBE_RAMP_RATE_AMPS_PER_CYCLE = 0.05; // Amps per 20ms cycle
  private static final double MOVEMENT_THRESHOLD_ROT_PER_SEC = 0.005;
  private static final double POSITION_TOLERANCE_ROT = 0.01;
  private static final double SETTLE_TIME_SECONDS = 0.5;
  private static final double MAX_PROBE_CURRENT_AMPS = 30.0;
  private static final int NUM_TEST_POSITIONS = 10;
  private static final String PREFIX = "TurretKsMap";

  private final InterpolatingDoubleTreeMap resultMapPositive = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap resultMapNegative = new InterpolatingDoubleTreeMap();

  public TurretKsMapCommand(
      SmartTurretController controller,
      Angle lowerLimit,
      Angle upperLimit,
      GenericSubsystem requirement) {
    this.controller = controller;
    this.talonFX = controller.getTalonFX();
    this.positionSignal = controller.getPositionSignal();
    this.velocitySignal = controller.getVelocitySignal();
    addRequirements(requirement);

    // Generate evenly-spaced test positions across the turret range.
    double lowerRot = lowerLimit.in(edu.wpi.first.units.Units.Rotations);
    double upperRot = upperLimit.in(edu.wpi.first.units.Units.Rotations);
    testPositions = new double[NUM_TEST_POSITIONS];
    for (int i = 0; i < NUM_TEST_POSITIONS; i++) {
      testPositions[i] = lowerRot + (upperRot - lowerRot) * i / (NUM_TEST_POSITIONS - 1);
    }
  }

  @Override
  public void initialize() {
    // Disable the SmartTurretController so the 200Hz Notifier stops sending competing commands.
    controller.stop();
    currentIndex = 0;
    currentState = State.MOVE_TO_POSITION;
    resultMapPositive.clear();
    resultMapNegative.clear();
    Logger.recordOutput(PREFIX + "Status", "Running");
  }

  @Override
  public void execute() {
    // Refresh signals for latest 250 Hz data.
    BaseStatusSignal.refreshAll(positionSignal, velocitySignal);
    double actualPos = positionSignal.getValueAsDouble();
    double actualVel = velocitySignal.getValueAsDouble();

    Logger.recordOutput(PREFIX + "State", currentState.name());
    Logger.recordOutput(PREFIX + "Position Index", currentIndex);
    Logger.recordOutput(PREFIX + "Probe Current", probeCurrent);

    switch (currentState) {
      case MOVE_TO_POSITION:
        talonFX.setControl(moveRequest.withPosition(testPositions[currentIndex]));
        if (Math.abs(actualPos - testPositions[currentIndex]) < POSITION_TOLERANCE_ROT
            && Math.abs(actualVel) < MOVEMENT_THRESHOLD_ROT_PER_SEC) {
          settleTimer.restart();
          currentState = State.SETTLE;
        }
        break;

      case SETTLE:
        talonFX.setControl(probeRequest.withOutput(0));
        if (settleTimer.hasElapsed(SETTLE_TIME_SECONDS)) {
          probeCurrent = 0;
          currentState = State.PROBE_POSITIVE;
        }
        break;

      case PROBE_POSITIVE:
        probeCurrent += PROBE_RAMP_RATE_AMPS_PER_CYCLE;
        talonFX.setControl(probeRequest.withOutput(probeCurrent));
        if (Math.abs(velocitySignal.getValueAsDouble()) > MOVEMENT_THRESHOLD_ROT_PER_SEC) {
          ksPositive = probeCurrent;
          probeCurrent = 0;
          settleTimer.restart();
          currentState = State.PROBE_NEGATIVE;
          // Return to position first.
          talonFX.setControl(probeRequest.withOutput(0));
        } else if (probeCurrent > MAX_PROBE_CURRENT_AMPS) {
          ksPositive = MAX_PROBE_CURRENT_AMPS;
          probeCurrent = 0;
          settleTimer.restart();
          currentState = State.PROBE_NEGATIVE;
          talonFX.setControl(probeRequest.withOutput(0));
        }
        break;

      case PROBE_NEGATIVE:
        if (!settleTimer.hasElapsed(SETTLE_TIME_SECONDS)) {
          talonFX.setControl(probeRequest.withOutput(0));
          return;
        }
        probeCurrent -= PROBE_RAMP_RATE_AMPS_PER_CYCLE;
        talonFX.setControl(probeRequest.withOutput(probeCurrent));
        if (Math.abs(velocitySignal.getValueAsDouble()) > MOVEMENT_THRESHOLD_ROT_PER_SEC) {
          ksNegative = Math.abs(probeCurrent);
          probeCurrent = 0;
          currentState = State.RECORD_AND_ADVANCE;
        } else if (Math.abs(probeCurrent) > MAX_PROBE_CURRENT_AMPS) {
          ksNegative = MAX_PROBE_CURRENT_AMPS;
          probeCurrent = 0;
          currentState = State.RECORD_AND_ADVANCE;
        }
        break;

      case RECORD_AND_ADVANCE:
        talonFX.setControl(probeRequest.withOutput(0));
        resultMapPositive.put(testPositions[currentIndex], ksPositive);
        resultMapNegative.put(testPositions[currentIndex], ksNegative);
        System.out.printf(
            "[TurretKsMap] Position %.4f rot: kS+ = %.3f A, kS- = %.3f A%n",
            testPositions[currentIndex], ksPositive, ksNegative);
        Logger.recordOutput(
            PREFIX + "kS+ @ " + String.format("%.1f", testPositions[currentIndex] * 360) + "deg",
            ksPositive);
        Logger.recordOutput(
            PREFIX + "kS- @ " + String.format("%.1f", testPositions[currentIndex] * 360) + "deg",
            ksNegative);

        currentIndex++;
        if (currentIndex < testPositions.length) {
          currentState = State.MOVE_TO_POSITION;
        } else {
          currentState = State.DONE;
        }
        break;

      case DONE:
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    talonFX.setControl(probeRequest.withOutput(0));
    // Re-disable the controller so the turret doesn't chase a stale target.
    controller.stop();
    if (currentState == State.DONE) {
      Logger.recordOutput(PREFIX + "Status", "Complete");
      System.out.println("[TurretKsMap] Mapping complete. Results:");
      for (int i = 0; i < testPositions.length; i++) {
        double pos = testPositions[i];
        System.out.printf(
            "  %.1f deg: kS+ = %.3f A, kS- = %.3f A%n",
            pos * 360, resultMapPositive.get(pos), resultMapNegative.get(pos));
      }
    } else {
      Logger.recordOutput(PREFIX + "Status", "Interrupted at index " + currentIndex);
    }
  }

  @Override
  public boolean isFinished() {
    return currentState == State.DONE;
  }

  public InterpolatingDoubleTreeMap getResultMapPositive() {
    return resultMapPositive;
  }

  public InterpolatingDoubleTreeMap getResultMapNegative() {
    return resultMapNegative;
  }
}
