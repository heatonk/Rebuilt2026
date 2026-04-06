package frc.robot.rebuilt.commands;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rebuilt.subsystems.Launcher.SmartTurretController;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import org.frc5010.common.arch.GenericSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Quasistatic feedforward characterization for the turret using {@link TorqueCurrentFOC}.
 *
 * <p>Ramps torque current very slowly so the turret reaches near-steady-state velocity at each
 * sample. Fits a 2-parameter model: {@code current = kS + kV * velocity} via least-squares
 * regression. Samples are only recorded once velocity exceeds a threshold (turret is moving).
 *
 * <p>Data is collected at ~250 Hz via a dedicated {@link Notifier}, not at the 50 Hz robot loop.
 * This provides ~5x more samples for significantly smoother regression data.
 *
 * <p>Results are logged under the "TurretQuasistatic/" prefix.
 */
public class TurretQuasistaticCommand extends Command {

  private final SmartTurretController controller;
  private final TalonFX talonFX;
  private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0);
  private final Timer timer = new Timer();

  // High-frequency status signals from SmartTurretController (250 Hz on CANivore).
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<AngularAcceleration> accelerationSignal;
  private final StatusSignal<Current> torqueCurrentSignal;

  // 250 Hz sampling infrastructure.
  private final ConcurrentLinkedQueue<CharacterizationSample> sampleQueue =
      new ConcurrentLinkedQueue<>();
  private Notifier samplingNotifier;
  private volatile boolean recording = false;

  private final double lowerLimitRot;
  private final double upperLimitRot;
  private boolean positionSafetyTriggered = false;

  /** Slow ramp rate so velocity has time to reach steady state at each current level. */
  private static final double RAMP_RATE_AMPS_PER_SEC = 0.5;

  /** Time to wait before ramping, letting the system settle. */
  private static final double SETTLE_DELAY_SECONDS = 1.0;

  /** Minimum velocity before we include a sample in the regression (turret must be moving). */
  private static final double MIN_VELOCITY_ROT_PER_SEC = 0.01;

  /** Safety margin from soft limits in mechanism rotations. */
  private static final double POSITION_SAFETY_MARGIN_ROT = 5.0 / 360.0;

  /** Sampling period for the high-frequency Notifier (4 ms = 250 Hz). */
  private static final double SAMPLING_PERIOD_SECONDS = 1.0 / 250.0;

  private static final String PREFIX = "TurretQuasistatic/";

  public TurretQuasistaticCommand(
      SmartTurretController controller, GenericSubsystem requirement) {
    this.controller = controller;
    this.talonFX = controller.getTalonFX();
    this.positionSignal = controller.getPositionSignal();
    this.velocitySignal = controller.getVelocitySignal();
    this.accelerationSignal = controller.getAccelerationSignal();
    this.torqueCurrentSignal = controller.getTorqueCurrentSignal();
    this.lowerLimitRot = controller.getConfig().getLowerLimitRotations();
    this.upperLimitRot = controller.getConfig().getUpperLimitRotations();
    addRequirements(requirement);
  }

  @Override
  public void initialize() {
    controller.stop();
    timer.restart();
    sampleQueue.clear();
    positionSafetyTriggered = false;
    recording = false;

    // Start the 250 Hz sampling Notifier.
    samplingNotifier = new Notifier(this::collectSample);
    samplingNotifier.setName("TurretQuasistaticSampler");
    samplingNotifier.startPeriodic(SAMPLING_PERIOD_SECONDS);
  }

  /** Called at 250 Hz by the Notifier. Refreshes signals and enqueues a sample. */
  private void collectSample() {
    if (!recording) return;

    BaseStatusSignal.refreshAll(
        positionSignal, velocitySignal, accelerationSignal, torqueCurrentSignal);

    sampleQueue.add(
        new CharacterizationSample(
            RobotController.getFPGATime() / 1e6,
            positionSignal.getValueAsDouble(),
            velocitySignal.getValueAsDouble(),
            accelerationSignal.getValueAsDouble(),
            torqueCurrentSignal.getValueAsDouble()));
  }

  @Override
  public void execute() {
    double elapsed = timer.get();
    if (elapsed < SETTLE_DELAY_SECONDS) {
      talonFX.setControl(torqueRequest.withOutput(0));
      recording = false;
      Logger.recordOutput(PREFIX + "State", "Settling");
      return;
    }

    // Safety check (uses cached signal value — no refreshAll needed at 50 Hz).
    double actualPos = positionSignal.getValueAsDouble();
    if (actualPos > (upperLimitRot - POSITION_SAFETY_MARGIN_ROT)
        || actualPos < (lowerLimitRot + POSITION_SAFETY_MARGIN_ROT)) {
      positionSafetyTriggered = true;
      talonFX.setControl(torqueRequest.withOutput(0));
      recording = false;
      Logger.recordOutput(PREFIX + "State", "Position Safety");
      return;
    }

    double rampTime = elapsed - SETTLE_DELAY_SECONDS;
    double current = rampTime * RAMP_RATE_AMPS_PER_SEC;
    talonFX.setControl(torqueRequest.withOutput(current));
    recording = true;

    // 50 Hz dashboard telemetry for live viewing.
    Logger.recordOutput(PREFIX + "Current (A)", current);
    Logger.recordOutput(PREFIX + "Velocity (rot/s)", velocitySignal.getValueAsDouble());
    Logger.recordOutput(PREFIX + "Position (rot)", actualPos);
    Logger.recordOutput(PREFIX + "SampleCount", sampleQueue.size());
  }

  @Override
  public void end(boolean interrupted) {
    // Stop sampling first, then stop the motor.
    recording = false;
    if (samplingNotifier != null) {
      samplingNotifier.stop();
      samplingNotifier.close();
      samplingNotifier = null;
    }
    talonFX.setControl(torqueRequest.withOutput(0));
    controller.stop();

    if (positionSafetyTriggered) {
      Logger.recordOutput(PREFIX + "Status", "Position safety triggered");
      System.out.println("[TurretQuasistatic] Ended: position safety triggered");
    }

    // Drain the queue, filtering for samples where the turret is actually moving.
    List<CharacterizationSample> samples = new ArrayList<>();
    CharacterizationSample s;
    while ((s = sampleQueue.poll()) != null) {
      if (Math.abs(s.velocityRotPerSec()) > MIN_VELOCITY_ROT_PER_SEC) {
        samples.add(s);
      }
    }

    if (samples.size() < 10) {
      Logger.recordOutput(PREFIX + "Status", "Not enough samples (" + samples.size() + ")");
      System.out.println("[TurretQuasistatic] Not enough samples: " + samples.size());
      return;
    }

    // 2-parameter least-squares fit: current = kS + kV * velocity
    // Normal equations:
    // [n,     sum_v ] [kS]   [sum_i ]
    // [sum_v, sum_vv] [kV] = [sum_iv]
    double n = samples.size();
    double sumV = 0, sumVV = 0, sumI = 0, sumIV = 0;

    for (CharacterizationSample sample : samples) {
      double v = sample.velocityRotPerSec();
      double c = sample.currentAmps();
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
    Logger.recordOutput(PREFIX + "Status", "Complete (" + samples.size() + " samples)");

    System.out.println("[TurretQuasistatic] kS = " + kS + " A, kV = " + kV + " A/(rot/s)");
    System.out.println("[TurretQuasistatic] Samples: " + samples.size());
  }

  @Override
  public boolean isFinished() {
    return positionSafetyTriggered;
  }
}
