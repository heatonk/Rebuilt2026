package frc.robot.rebuilt.commands;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rebuilt.subsystems.Launcher.SmartTurretController;
import java.util.LinkedList;
import java.util.List;
import org.frc5010.common.arch.GenericSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Quasistatic feedforward characterization for the turret using {@link TorqueCurrentFOC}.
 *
 * <p>Ramps torque current very slowly so the turret reaches near-steady-state velocity at each
 * sample. Fits a 2-parameter model: {@code current = kS + kV * velocity} via least-squares
 * regression. Samples are only recorded once velocity exceeds a threshold (turret is moving).
 *
 * <p>Uses 250 Hz status signals from the TalonFX for high-resolution data.
 *
 * <p>Results are logged under the "TurretQuasistatic/" prefix.
 */
public class TurretQuasistaticCommand extends Command {

  private final SmartTurretController controller;
  private final TalonFX talonFX;
  private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0);
  private final Timer timer = new Timer();

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;

  private final List<Double> currentSamples = new LinkedList<>();
  private final List<Double> velocitySamples = new LinkedList<>();

  private final double lowerLimitRot;
  private final double upperLimitRot;
  private boolean positionSafetyTriggered = false;

  /** Slow ramp rate so velocity has time to reach steady state at each current level. */
  private static final double RAMP_RATE_AMPS_PER_SEC = 0.5;

  /** Time to wait before ramping, letting the system settle. */
  private static final double SETTLE_DELAY_SECONDS = 1.0;

  /** Minimum velocity before we start recording samples (turret must be moving). */
  private static final double MIN_VELOCITY_ROT_PER_SEC = 0.01;

  /** Safety margin from soft limits in mechanism rotations. */
  private static final double POSITION_SAFETY_MARGIN_ROT = 5.0 / 360.0;

  private static final String PREFIX = "TurretQuasistatic/";

  public TurretQuasistaticCommand(
      SmartTurretController controller, GenericSubsystem requirement) {
    this.controller = controller;
    this.talonFX = controller.getTalonFX();
    this.positionSignal = controller.getPositionSignal();
    this.velocitySignal = controller.getVelocitySignal();
    this.lowerLimitRot = controller.getConfig().getLowerLimitRotations();
    this.upperLimitRot = controller.getConfig().getUpperLimitRotations();
    addRequirements(requirement);
  }

  @Override
  public void initialize() {
    controller.stop();
    timer.restart();
    currentSamples.clear();
    velocitySamples.clear();
    positionSafetyTriggered = false;
  }

  @Override
  public void execute() {
    // Refresh signals for latest data at 250 Hz.
    BaseStatusSignal.refreshAll(positionSignal, velocitySignal);

    double elapsed = timer.get();
    if (elapsed < SETTLE_DELAY_SECONDS) {
      talonFX.setControl(torqueRequest.withOutput(0));
      Logger.recordOutput(PREFIX + "State", "Settling");
      return;
    }

    double actualPos = positionSignal.getValueAsDouble();
    if (actualPos > (upperLimitRot - POSITION_SAFETY_MARGIN_ROT)
        || actualPos < (lowerLimitRot + POSITION_SAFETY_MARGIN_ROT)) {
      positionSafetyTriggered = true;
      talonFX.setControl(torqueRequest.withOutput(0));
      Logger.recordOutput(PREFIX + "State", "Position Safety");
      return;
    }

    double rampTime = elapsed - SETTLE_DELAY_SECONDS;
    double current = rampTime * RAMP_RATE_AMPS_PER_SEC;
    talonFX.setControl(torqueRequest.withOutput(current));

    double velocity = velocitySignal.getValueAsDouble(); // mechanism rot/s

    // Only record samples once the turret is actually moving.
    if (Math.abs(velocity) > MIN_VELOCITY_ROT_PER_SEC) {
      currentSamples.add(current);
      velocitySamples.add(velocity);
    }

    Logger.recordOutput(PREFIX + "Current (A)", current);
    Logger.recordOutput(PREFIX + "Velocity (rot/s)", velocity);
    Logger.recordOutput(PREFIX + "Position (rot)", actualPos);
    Logger.recordOutput(PREFIX + "SampleCount", currentSamples.size());
  }

  @Override
  public void end(boolean interrupted) {
    talonFX.setControl(torqueRequest.withOutput(0));
    controller.stop();

    if (positionSafetyTriggered) {
      Logger.recordOutput(PREFIX + "Status", "Position safety triggered");
      System.out.println("[TurretQuasistatic] Ended: position safety triggered");
    }

    if (currentSamples.size() < 10) {
      Logger.recordOutput(PREFIX + "Status", "Not enough samples (" + currentSamples.size() + ")");
      System.out.println("[TurretQuasistatic] Not enough samples: " + currentSamples.size());
      return;
    }

    // 2-parameter least-squares fit: current = kS + kV * velocity
    // Normal equations:
    // [n,   sum_v ] [kS]   [sum_i ]
    // [sum_v, sum_vv] [kV] = [sum_iv]
    double n = currentSamples.size();
    double sumV = 0, sumVV = 0, sumI = 0, sumIV = 0;

    for (int i = 0; i < currentSamples.size(); i++) {
      double c = currentSamples.get(i);
      double v = velocitySamples.get(i);
      sumV += v;
      sumVV += v * v;
      sumI += c;
      sumIV += c * v;
    }

    double det = n * sumVV - sumV * sumV;
    if (Math.abs(det) < 1e-12) {
      Logger.recordOutput(PREFIX + "Status", "Singular matrix");
      System.out.println("[TurretQuasistatic] Singular matrix — no solution");
      return;
    }

    double kS = (sumI * sumVV - sumV * sumIV) / det;
    double kV = (n * sumIV - sumV * sumI) / det;

    Logger.recordOutput(PREFIX + "kS (Amps)", kS);
    Logger.recordOutput(PREFIX + "kV (Amps per rot/s)", kV);
    Logger.recordOutput(PREFIX + "Status", "Complete (" + currentSamples.size() + " samples)");

    System.out.println("[TurretQuasistatic] kS = " + kS + " A, kV = " + kV + " A/(rot/s)");
    System.out.println("[TurretQuasistatic] Samples: " + currentSamples.size());
  }

  @Override
  public boolean isFinished() {
    return positionSafetyTriggered;
  }
}
