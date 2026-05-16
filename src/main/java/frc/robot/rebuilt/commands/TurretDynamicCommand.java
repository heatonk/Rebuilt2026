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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.rebuilt.subsystems.Launcher.SmartTurretController;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import org.littletonrobotics.junction.Logger;

/**
 * Dynamic feedforward characterization for the turret using bidirectional current pulses.
 *
 * <p>Applies alternating positive and negative current steps to create many acceleration transients
 * in both directions. The bidirectional pulsing keeps the turret oscillating near its starting
 * position so more transients can be recorded before hitting a soft limit.
 *
 * <p>Two distinct quantities are measured from the same dataset:
 *
 * <ul>
 *   <li><b>kS_dynamic</b>: kinetic (dynamic) friction during motion. Computed from near-steady-
 *       state samples (|acceleration| below {@link #STEADY_STATE_ACCEL_THRESHOLD}) where {@code
 *       kS_dyn = sgn(v) * (current - kV * velocity)}. This is often different from the static kS
 *       measured by the quasistatic command.
 *   <li><b>kA</b>: inertia feedforward. Computed from transient samples (|acceleration| above
 *       {@link #MIN_ACCEL_ROT_PER_SEC_SQ}) using kS_dynamic in the residual: {@code (current -
 *       kS_dyn * sgn(v) - kV * v) = kA * acceleration}.
 * </ul>
 *
 * <p>kS_dynamic is written back to the SmartDashboard key {@code "Dynamic kS"} at the end of each
 * run so it is automatically available for the next run's input (closing the tuning loop).
 *
 * <p>Data is collected at 250 Hz via a dedicated {@link Notifier}. Results are logged under the
 * {@code "TurretDynamic/"} prefix.
 */
public class TurretDynamicCommand extends Command {

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

  // Values read from SmartDashboard at command start.
  private double inputKV;
  private double stepAmps;

  /** Time to hold zero current before applying pulses. */
  private static final double SETTLE_DELAY_SECONDS = 1.0;

  /** Default step current (Amps). Configurable via SmartDashboard. */
  private static final double DEFAULT_STEP_AMPS = 50.0;

  /**
   * Duration of each current pulse in seconds. Short enough for multiple oscillation cycles, long
   * enough to generate a clean acceleration transient at each direction change.
   */
  private static final double PULSE_DURATION_SECS = 0.08;

  /**
   * Acceleration threshold below which a sample is considered steady-state (used for kS_dynamic).
   * Units: mechanism rot/s^2.
   */
  private static final double STEADY_STATE_ACCEL_THRESHOLD = 0.3;

  /**
   * Minimum acceleration magnitude for transient samples used in kA regression. Units: mechanism
   * rot/s^2.
   */
  private static final double MIN_ACCEL_ROT_PER_SEC_SQ = 0.5;

  /** Minimum velocity magnitude to include a sample (filters near-zero noise). */
  private static final double MIN_VELOCITY_ROT_PER_SEC = 0.05;

  /** Safety margin from soft limits in mechanism rotations. */
  private static final double POSITION_SAFETY_MARGIN_ROT = 5.0 / 360.0;

  /** Sampling period for the high-frequency Notifier (4 ms = 250 Hz). */
  private static final double SAMPLING_PERIOD_SECONDS = 1.0 / 250.0;

  private static final String PREFIX = "TurretDynamic/";

  public TurretDynamicCommand(SmartTurretController controller, SubsystemBase requirement) {
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

    // Publish defaults — kV comes from the controller config; kS is not needed as an input
    // because this command measures kS_dynamic directly from the data.
    SmartDashboard.putNumber(
        "Dynamic kV", SmartDashboard.getNumber("Dynamic kV", controller.getConfig().getKV()));
    SmartDashboard.putNumber(
        "Dynamic Step Amps", SmartDashboard.getNumber("Dynamic Step Amps", DEFAULT_STEP_AMPS));

    inputKV = SmartDashboard.getNumber("Dynamic kV", controller.getConfig().getKV());
    stepAmps = SmartDashboard.getNumber("Dynamic Step Amps", DEFAULT_STEP_AMPS);

    System.out.println("[TurretDynamic] Step=" + stepAmps + " A, kV=" + inputKV);
    System.out.println(
        "[TurretDynamic] Bidirectional pulsing: "
            + PULSE_DURATION_SECS
            + "s pulses. kS_dynamic will be computed from data.");

    samplingNotifier = new Notifier(this::collectSample);
    samplingNotifier.setName("TurretDynamicSampler");
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

    // Safety check (uses cached signal value).
    double actualPos = positionSignal.getValueAsDouble();
    if (actualPos > (upperLimitRot - POSITION_SAFETY_MARGIN_ROT)
        || actualPos < (lowerLimitRot + POSITION_SAFETY_MARGIN_ROT)) {
      positionSafetyTriggered = true;
      talonFX.setControl(torqueRequest.withOutput(0));
      recording = false;
      Logger.recordOutput(PREFIX + "State", "Position Safety");
      return;
    }

    // Bidirectional pulsing: alternate polarity every PULSE_DURATION_SECS.
    // Each direction change creates a fresh acceleration transient and near-zero-accel
    // steady-state.
    double pulseElapsed = elapsed - SETTLE_DELAY_SECONDS;
    int pulseIndex = (int) (pulseElapsed / PULSE_DURATION_SECS);
    double appliedAmps = (pulseIndex % 2 == 0) ? stepAmps : -stepAmps;

    talonFX.setControl(torqueRequest.withOutput(appliedAmps));
    recording = true;

    Logger.recordOutput(PREFIX + "Applied Current (A)", appliedAmps);
    Logger.recordOutput(PREFIX + "Velocity (rot-s)", velocitySignal.getValueAsDouble());
    Logger.recordOutput(PREFIX + "Acceleration (rot-s2)", accelerationSignal.getValueAsDouble());
    Logger.recordOutput(PREFIX + "Position (rot)", actualPos);
    Logger.recordOutput(PREFIX + "PulseIndex", pulseIndex);
    Logger.recordOutput(PREFIX + "SampleCount", sampleQueue.size());
  }

  @Override
  public void end(boolean interrupted) {
    recording = false;
    if (samplingNotifier != null) {
      samplingNotifier.stop();
      samplingNotifier.close();
      samplingNotifier = null;
    }
    talonFX.setControl(torqueRequest.withOutput(0));
    controller.stop();

    if (positionSafetyTriggered) {
      Logger.recordOutput(PREFIX + "Status", "Position safety triggered — partial data");
      System.out.println("[TurretDynamic] Ended: position safety triggered (using collected data)");
    }

    // Drain and filter samples: require minimum velocity to avoid near-zero noise.
    List<CharacterizationSample> allSamples = new ArrayList<>();
    CharacterizationSample s;
    while ((s = sampleQueue.poll()) != null) {
      if (Math.abs(s.velocityRotPerSec()) > MIN_VELOCITY_ROT_PER_SEC) {
        allSamples.add(s);
      }
    }

    if (allSamples.size() < 10) {
      Logger.recordOutput(PREFIX + "Status", "Not enough samples (" + allSamples.size() + ")");
      System.out.println("[TurretDynamic] Not enough samples: " + allSamples.size());
      return;
    }

    // ---- Pass 1: Compute kS_dynamic from near-steady-state samples -------------------------
    // At near-constant velocity: current ≈ kS_dyn * sgn(v) + kV * v
    // → kS_dyn estimate per sample = sgn(v) * (current - kV * v)
    List<Double> ksDynEstimates = new ArrayList<>();
    for (CharacterizationSample sample : allSamples) {
      if (Math.abs(sample.accelerationRotPerSecSq()) < STEADY_STATE_ACCEL_THRESHOLD) {
        double ksDynEst =
            Math.signum(sample.velocityRotPerSec())
                * (sample.currentAmps() - inputKV * sample.velocityRotPerSec());
        ksDynEstimates.add(ksDynEst);
      }
    }

    double kSDynamic;
    if (ksDynEstimates.size() >= 5) {
      kSDynamic = ksDynEstimates.stream().mapToDouble(Double::doubleValue).average().orElse(0);
    } else {
      // Not enough steady-state samples — fall back to the controller config kS.
      kSDynamic = controller.getConfig().getKS();
      Logger.recordOutput(
          PREFIX + "kS_dynamic Status",
          "Insufficient steady-state samples (" + ksDynEstimates.size() + "), using config kS");
      System.out.println(
          "[TurretDynamic] Not enough steady-state samples for kS_dynamic ("
              + ksDynEstimates.size()
              + "); using config kS = "
              + kSDynamic);
    }

    // ---- Pass 2: Compute kA from transient samples using kS_dynamic -----------------------
    // Model: current - kS_dyn * sgn(v) - kV * v = kA * acceleration
    // 1-parameter LS: kA = sum(residual * a) / sum(a^2)
    double sumRA = 0, sumAA = 0;
    int transientCount = 0;
    for (CharacterizationSample sample : allSamples) {
      if (Math.abs(sample.accelerationRotPerSecSq()) > MIN_ACCEL_ROT_PER_SEC_SQ) {
        double residual =
            sample.currentAmps()
                - kSDynamic * Math.signum(sample.velocityRotPerSec())
                - inputKV * sample.velocityRotPerSec();
        double a = sample.accelerationRotPerSecSq();
        sumRA += residual * a;
        sumAA += a * a;
        transientCount++;
      }
    }

    if (transientCount < 5 || Math.abs(sumAA) < 1e-12) {
      Logger.recordOutput(
          PREFIX + "Status", "Not enough transient samples for kA (" + transientCount + ")");
      System.out.println("[TurretDynamic] Not enough transient samples: " + transientCount);
      // Still output kS_dynamic even if kA failed.
      outputKSDynamic(kSDynamic, ksDynEstimates.size());
      return;
    }

    double kA = sumRA / sumAA;

    // ---- Output results -------------------------------------------------------------------
    Logger.recordOutput(PREFIX + "kS_dynamic (Amps)", kSDynamic);
    Logger.recordOutput(PREFIX + "kS_dynamic Sample Count", ksDynEstimates.size());
    Logger.recordOutput(PREFIX + "kA (Amps per rot-s2)", kA);
    Logger.recordOutput(PREFIX + "kA Sample Count", transientCount);
    Logger.recordOutput(PREFIX + "Input kV (Amps per rot-s)", inputKV);
    Logger.recordOutput(PREFIX + "Total Sample Count", allSamples.size());
    Logger.recordOutput(PREFIX + "Status", "Complete");

    System.out.println(
        "[TurretDynamic] kS_dynamic = "
            + kSDynamic
            + " A  ("
            + ksDynEstimates.size()
            + " samples)");
    System.out.println(
        "[TurretDynamic] kA         = " + kA + " A/(rot/s^2)  (" + transientCount + " samples)");
    System.out.println(
        "[TurretDynamic] Used kV=" + inputKV + ", total samples=" + allSamples.size());

    outputKSDynamic(kSDynamic, ksDynEstimates.size());
  }

  /**
   * Publishes kS_dynamic to the SmartDashboard key {@code "Dynamic kS"} so it is available as the
   * starting value for the next characterization run and for use in other commands.
   */
  private void outputKSDynamic(double kSDynamic, int sampleCount) {
    SmartDashboard.putNumber("Dynamic kS", kSDynamic);
    System.out.println(
        "[TurretDynamic] kS_dynamic="
            + kSDynamic
            + " A written to SmartDashboard 'Dynamic kS' ("
            + sampleCount
            + " steady-state samples).");
  }

  @Override
  public boolean isFinished() {
    return positionSafetyTriggered;
  }
}
