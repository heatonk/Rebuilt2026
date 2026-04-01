package frc.robot.rebuilt.commands;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.LinkedList;
import java.util.List;
import org.frc5010.common.arch.GenericSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Feedforward characterization command for the turret using {@link TorqueCurrentFOC}.
 *
 * <p>Ramps torque current from zero and measures resulting velocity and acceleration to compute kS
 * (static friction current in Amps), kV (velocity constant in Amps per mechanism rot/s), and kA
 * (acceleration constant in Amps per mechanism rot/s^2) via 3-parameter least-squares regression.
 *
 * <p>Results are logged via AdvantageKit {@code Logger.recordOutput()} under the "TurretFFChar/"
 * prefix.
 */
public class TurretFFCharacterizationCommand extends Command {

  private final TalonFX talonFX;
  private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0);
  private final Timer timer = new Timer();

  private final List<Double> currentSamples = new LinkedList<>();
  private final List<Double> velocitySamples = new LinkedList<>();
  private final List<Double> accelerationSamples = new LinkedList<>();

  private double previousVelocity = 0.0;
  private double previousTimestamp = 0.0;

  private final double lowerLimitRot;
  private final double upperLimitRot;
  private boolean positionSafetyTriggered = false;

  private static final double RAMP_RATE_AMPS_PER_SEC = 0.5;
  private static final double SETTLE_DELAY_SECONDS = 2.0;
  private static final double POSITION_SAFETY_MARGIN_ROT = 5.0 / 360.0; // 5 degrees
  private static final String PREFIX = "TurretFFChar/";

  public TurretFFCharacterizationCommand(
      TalonFX talonFX, double lowerLimitRot, double upperLimitRot, GenericSubsystem requirement) {
    this.talonFX = talonFX;
    this.lowerLimitRot = lowerLimitRot;
    this.upperLimitRot = upperLimitRot;
    addRequirements(requirement);
  }

  @Override
  public void initialize() {
    timer.restart();
    currentSamples.clear();
    velocitySamples.clear();
    accelerationSamples.clear();
    previousVelocity = 0.0;
    previousTimestamp = 0.0;
    positionSafetyTriggered = false;
  }

  @Override
  public void execute() {
    double elapsed = timer.get();
    if (elapsed < SETTLE_DELAY_SECONDS) {
      // Hold at zero current while settling.
      talonFX.setControl(torqueRequest.withOutput(0));
      previousTimestamp = elapsed;
      return;
    }

    double actualPos = talonFX.getPosition().getValueAsDouble();
    if (actualPos > (upperLimitRot - POSITION_SAFETY_MARGIN_ROT)
        || actualPos < (lowerLimitRot + POSITION_SAFETY_MARGIN_ROT)) {
      positionSafetyTriggered = true;
      talonFX.setControl(torqueRequest.withOutput(0));
      return;
    }

    double rampTime = elapsed - SETTLE_DELAY_SECONDS;
    double current = rampTime * RAMP_RATE_AMPS_PER_SEC;
    talonFX.setControl(torqueRequest.withOutput(current));

    double velocity = talonFX.getVelocity().getValueAsDouble(); // mechanism rot/s

    // Numerical acceleration via differentiation of velocity samples.
    double dt = elapsed - previousTimestamp;
    double acceleration = 0.0;
    if (dt > 1e-6) {
      acceleration = (velocity - previousVelocity) / dt;
    }
    previousVelocity = velocity;
    previousTimestamp = elapsed;

    currentSamples.add(current);
    velocitySamples.add(velocity);
    accelerationSamples.add(acceleration);

    Logger.recordOutput(PREFIX + "Current (A)", current);
    Logger.recordOutput(PREFIX + "Velocity (rot/s)", velocity);
    Logger.recordOutput(PREFIX + "Acceleration (rot/s^2)", acceleration);
    Logger.recordOutput(PREFIX + "Position (rot)", actualPos);
  }

  @Override
  public void end(boolean interrupted) {
    talonFX.setControl(torqueRequest.withOutput(0));

    if (positionSafetyTriggered) {
      Logger.recordOutput(PREFIX + "Status", "Position safety triggered");
      System.out.println("[TurretFFChar] Ended: position safety triggered near limit");
    }

    if (currentSamples.size() < 10) {
      Logger.recordOutput(PREFIX + "Status", "Not enough samples");
      return;
    }

    // 3-parameter least-squares fit: current = kS + kV * velocity + kA * acceleration
    // Normal equations (3x3):
    // [n,   ΣV,  ΣA ] [kS]   [ΣI ]
    // [ΣV,  ΣV², ΣVA] [kV] = [ΣIV]
    // [ΣA,  ΣVA, ΣA²] [kA]   [ΣIA]
    double n = currentSamples.size();
    double sumV = 0, sumA = 0, sumVV = 0, sumVA = 0, sumAA = 0;
    double sumI = 0, sumIV = 0, sumIA = 0;

    for (int i = 0; i < currentSamples.size(); i++) {
      double c = currentSamples.get(i);
      double v = velocitySamples.get(i);
      double a = accelerationSamples.get(i);
      sumV += v;
      sumA += a;
      sumVV += v * v;
      sumVA += v * a;
      sumAA += a * a;
      sumI += c;
      sumIV += c * v;
      sumIA += c * a;
    }

    // Solve via Cramer's rule for the 3x3 system.
    double det =
        n * (sumVV * sumAA - sumVA * sumVA)
            - sumV * (sumV * sumAA - sumVA * sumA)
            + sumA * (sumV * sumVA - sumVV * sumA);

    if (Math.abs(det) < 1e-12) {
      Logger.recordOutput(PREFIX + "Status", "Singular matrix");
      return;
    }

    double detKS =
        sumI * (sumVV * sumAA - sumVA * sumVA)
            - sumV * (sumIV * sumAA - sumIA * sumVA)
            + sumA * (sumIV * sumVA - sumIA * sumVV);

    double detKV =
        n * (sumIV * sumAA - sumIA * sumVA)
            - sumI * (sumV * sumAA - sumVA * sumA)
            + sumA * (sumV * sumIA - sumIV * sumA);

    double detKA =
        n * (sumVV * sumIA - sumVA * sumIV)
            - sumV * (sumV * sumIA - sumIV * sumA)
            + sumI * (sumV * sumVA - sumVV * sumA);

    double kS = detKS / det;
    double kV = detKV / det;
    double kA = detKA / det;

    Logger.recordOutput(PREFIX + "kS (Amps)", kS);
    Logger.recordOutput(PREFIX + "kV (Amps per rot/s)", kV);
    Logger.recordOutput(PREFIX + "kA (Amps per rot/s^2)", kA);
    Logger.recordOutput(PREFIX + "Status", "Complete (" + currentSamples.size() + " samples)");

    System.out.println(
        "[TurretFFChar] kS = " + kS + " A, kV = " + kV + " A/(rot/s), kA = " + kA + " A/(rot/s^2)");
    System.out.println("[TurretFFChar] Samples: " + currentSamples.size());
  }

  @Override
  public boolean isFinished() {
    return positionSafetyTriggered;
  }
}
