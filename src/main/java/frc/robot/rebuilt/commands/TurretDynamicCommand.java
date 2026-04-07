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
import frc.robot.rebuilt.subsystems.Launcher.SmartTurretController;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import org.frc5010.common.arch.GenericSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Dynamic feedforward characterization for the turret using {@link TorqueCurrentFOC}.
 *
 * <p>Applies a constant current step above kS and measures the acceleration response to determine
 * kA. Unlike a slow ramp (quasistatic), a step input causes rapid acceleration, making the kA term
 * dominant and measurable.
 *
 * <p>Requires kS and kV values from a prior quasistatic characterization. These are read from
 * SmartDashboard keys {@code "Dynamic kS"} and {@code "Dynamic kV"}. The step current level is also
 * tunable via {@code "Dynamic Step Amps"}.
 *
 * <p>Data is collected at ~250 Hz via a dedicated {@link Notifier} using the TalonFX's hardware
 * acceleration signal (not numerical differentiation), providing smooth, high-resolution data.
 *
 * <p>The model: {@code current = kS + kV * velocity + kA * acceleration} is rearranged to:
 * {@code (current - kS - kV * velocity) = kA * acceleration}, then kA is determined via
 * least-squares regression on the acceleration transient samples.
 *
 * <p>Results are logged under the "TurretDynamic" prefix.
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
  private double inputKS;
  private double inputKV;
  private double stepAmps;

  /** Time to hold zero current before applying the step. */
  private static final double SETTLE_DELAY_SECONDS = 1.0;

  /** Default step current (Amps). */
  private static final double DEFAULT_STEP_AMPS = 35.0;

  /** Minimum acceleration threshold to filter out near-zero noise samples. */
  private static final double MIN_ACCEL_ROT_PER_SEC_SQ = 0.1;

  /** Safety margin from soft limits in mechanism rotations. */
  private static final double POSITION_SAFETY_MARGIN_ROT = 5.0 / 360.0;

  /** Sampling period for the high-frequency Notifier (4 ms = 250 Hz). */
  private static final double SAMPLING_PERIOD_SECONDS = 1.0 / 250.0;

  private static final String PREFIX = "TurretDynamic";

  public TurretDynamicCommand(
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

    // Publish defaults if not already on SmartDashboard.
    SmartDashboard.putNumber(
        "Dynamic kS",
        SmartDashboard.getNumber("Dynamic kS", controller.getConfig().getKS()));
    SmartDashboard.putNumber(
        "Dynamic kV",
        SmartDashboard.getNumber("Dynamic kV", controller.getConfig().getKV()));
    SmartDashboard.putNumber(
        "Dynamic Step Amps",
        SmartDashboard.getNumber("Dynamic Step Amps", DEFAULT_STEP_AMPS));

    // Read the operator-supplied values.
    inputKS = SmartDashboard.getNumber("Dynamic kS", controller.getConfig().getKS());
    inputKV = SmartDashboard.getNumber("Dynamic kV", controller.getConfig().getKV());
    stepAmps = SmartDashboard.getNumber("Dynamic Step Amps", DEFAULT_STEP_AMPS);

    System.out.println(
        "[TurretDynamic] Step=" + stepAmps + " A, kS=" + inputKS + ", kV=" + inputKV);

    // Start the 250 Hz sampling Notifier.
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

    // Apply constant step current.
    talonFX.setControl(torqueRequest.withOutput(stepAmps));
    recording = true;

    // 50 Hz dashboard telemetry for live viewing.
    Logger.recordOutput(PREFIX + "Current (A)", stepAmps);
    Logger.recordOutput(PREFIX + "Velocity (rot/s)", velocitySignal.getValueAsDouble());
    Logger.recordOutput(
        PREFIX + "Acceleration (rot/s^2)", accelerationSignal.getValueAsDouble());
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
      System.out.println("[TurretDynamic] Ended: position safety triggered");
    }

    // Drain the queue, filtering for samples with meaningful acceleration.
    List<CharacterizationSample> samples = new ArrayList<>();
    CharacterizationSample s;
    while ((s = sampleQueue.poll()) != null) {
      if (Math.abs(s.accelerationRotPerSecSq()) > MIN_ACCEL_ROT_PER_SEC_SQ) {
        samples.add(s);
      }
    }

    if (samples.size() < 5) {
      Logger.recordOutput(PREFIX + "Status", "Not enough samples (" + samples.size() + ")");
      System.out.println("[TurretDynamic] Not enough samples: " + samples.size());
      return;
    }

    // 1-parameter least-squares: residual = kA * acceleration
    // where residual = measuredCurrent - kS - kV * velocity
    double sumRA = 0, sumAA = 0;
    for (CharacterizationSample sample : samples) {
      double residual =
          sample.currentAmps() - inputKS - inputKV * sample.velocityRotPerSec();
      double a = sample.accelerationRotPerSecSq();
      sumRA += residual * a;
      sumAA += a * a;
    }

    if (Math.abs(sumAA) < 1e-12) {
      Logger.recordOutput(PREFIX + "Status", "No acceleration variance");
      System.out.println("[TurretDynamic] No acceleration variance — cannot solve for kA");
      return;
    }

    double kA = sumRA / sumAA;

    Logger.recordOutput(PREFIX + "kA (Amps per rot/s^2)", kA);
    Logger.recordOutput(PREFIX + "Input kS (Amps)", inputKS);
    Logger.recordOutput(PREFIX + "Input kV (Amps per rot/s)", inputKV);
    Logger.recordOutput(PREFIX + "Status", "Complete (" + samples.size() + " samples)");

    System.out.println("[TurretDynamic] kA = " + kA + " A/(rot/s^2)");
    System.out.println("[TurretDynamic] Used kS=" + inputKS + ", kV=" + inputKV);
    System.out.println("[TurretDynamic] Samples: " + samples.size());
  }

  @Override
  public boolean isFinished() {
    return positionSafetyTriggered;
  }
}
