package frc.robot.rebuilt.commands;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rebuilt.subsystems.Launcher.SmartTurretController;
import java.util.LinkedList;
import java.util.List;
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
 * <p>Uses 250 Hz status signals from the TalonFX for high-resolution acceleration data.
 *
 * <p>The model: {@code current = kS + kV * velocity + kA * acceleration} is rearranged to:
 * {@code (current - kS - kV * velocity) = kA * acceleration}, then kA is determined via
 * least-squares regression on the acceleration transient samples.
 *
 * <p>Results are logged under the "TurretDynamic/" prefix.
 */
public class TurretDynamicCommand extends Command {

  private final SmartTurretController controller;
  private final TalonFX talonFX;
  private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0);
  private final Timer timer = new Timer();

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;

  // Samples: residual current (I - kS - kV*v) and acceleration
  private final List<Double> residualSamples = new LinkedList<>();
  private final List<Double> accelerationSamples = new LinkedList<>();

  private double previousVelocity = 0.0;
  private double previousTimestamp = 0.0;

  private final double lowerLimitRot;
  private final double upperLimitRot;
  private boolean positionSafetyTriggered = false;

  // Values read from SmartDashboard at command start
  private double inputKS;
  private double inputKV;
  private double stepAmps;

  /** Time to hold zero current before applying the step. */
  private static final double SETTLE_DELAY_SECONDS = 1.0;

  /** Default step current above kS (Amps). */
  private static final double DEFAULT_STEP_AMPS = 35.0;

  /** Minimum acceleration threshold to filter out near-zero noise samples. */
  private static final double MIN_ACCEL_ROT_PER_SEC_SQ = 0.1;

  /** Safety margin from soft limits in mechanism rotations. */
  private static final double POSITION_SAFETY_MARGIN_ROT = 5.0 / 360.0;

  private static final String PREFIX = "TurretDynamic/";

  public TurretDynamicCommand(
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
    residualSamples.clear();
    accelerationSamples.clear();
    previousVelocity = 0.0;
    previousTimestamp = 0.0;
    positionSafetyTriggered = false;

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
  }

  @Override
  public void execute() {
    // Refresh signals for latest 250 Hz data.
    BaseStatusSignal.refreshAll(positionSignal, velocitySignal);

    double elapsed = timer.get();
    if (elapsed < SETTLE_DELAY_SECONDS) {
      talonFX.setControl(torqueRequest.withOutput(0));
      previousTimestamp = elapsed;
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

    // Apply constant step current.
    talonFX.setControl(torqueRequest.withOutput(stepAmps));

    double velocity = velocitySignal.getValueAsDouble(); // mechanism rot/s

    // Numerical acceleration via differentiation.
    double dt = elapsed - previousTimestamp;
    double acceleration = 0.0;
    if (dt > 1e-6) {
      acceleration = (velocity - previousVelocity) / dt;
    }
    previousVelocity = velocity;
    previousTimestamp = elapsed;

    // Compute the residual: what's left after subtracting kS and kV*v from the applied current.
    // residual = current - kS - kV * velocity = kA * acceleration
    double residual = stepAmps - inputKS - inputKV * velocity;

    // Only record samples with meaningful acceleration (during the transient).
    if (Math.abs(acceleration) > MIN_ACCEL_ROT_PER_SEC_SQ) {
      residualSamples.add(residual);
      accelerationSamples.add(acceleration);
    }

    Logger.recordOutput(PREFIX + "Current (A)", stepAmps);
    Logger.recordOutput(PREFIX + "Velocity (rot/s)", velocity);
    Logger.recordOutput(PREFIX + "Acceleration (rot/s^2)", acceleration);
    Logger.recordOutput(PREFIX + "Residual (A)", residual);
    Logger.recordOutput(PREFIX + "Position (rot)", actualPos);
    Logger.recordOutput(PREFIX + "SampleCount", residualSamples.size());
  }

  @Override
  public void end(boolean interrupted) {
    talonFX.setControl(torqueRequest.withOutput(0));
    controller.stop();

    if (positionSafetyTriggered) {
      Logger.recordOutput(PREFIX + "Status", "Position safety triggered");
      System.out.println("[TurretDynamic] Ended: position safety triggered");
    }

    if (residualSamples.size() < 5) {
      Logger.recordOutput(PREFIX + "Status", "Not enough samples (" + residualSamples.size() + ")");
      System.out.println("[TurretDynamic] Not enough samples: " + residualSamples.size());
      return;
    }

    // 1-parameter least-squares: residual = kA * acceleration
    // kA = sum(residual * accel) / sum(accel * accel)
    double sumRA = 0, sumAA = 0;
    for (int i = 0; i < residualSamples.size(); i++) {
      double r = residualSamples.get(i);
      double a = accelerationSamples.get(i);
      sumRA += r * a;
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
    Logger.recordOutput(PREFIX + "Status", "Complete (" + residualSamples.size() + " samples)");

    System.out.println("[TurretDynamic] kA = " + kA + " A/(rot/s^2)");
    System.out.println("[TurretDynamic] Used kS=" + inputKS + ", kV=" + inputKV);
    System.out.println("[TurretDynamic] Samples: " + residualSamples.size());
  }

  @Override
  public boolean isFinished() {
    return positionSafetyTriggered;
  }
}
