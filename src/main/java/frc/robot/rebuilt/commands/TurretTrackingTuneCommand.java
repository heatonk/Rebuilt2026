package frc.robot.rebuilt.commands;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.rebuilt.subsystems.Launcher.SmartTurretController;
import org.littletonrobotics.junction.Logger;

/**
 * Generates a sinusoidal reference signal for the turret and measures tracking error.
 *
 * <p>Amplitude, frequency, PID gains (Slot1: tracking), and feedforward gains (kV, kA) are
 * adjustable via SmartDashboard in real-time. The amplitude is automatically clamped so the
 * sinusoidal stays within safe turret limits centered at 0. The analytically-derived velocity and
 * acceleration of the sinusoid are passed as feedforward to the SmartTurretController.
 *
 * <p>Slot1 kS is always 0 in the firmware — the position-dependent kS is injected externally by the
 * SmartTurretController. kV and kA run at full firmware frequency (1 kHz) for best performance.
 */
public class TurretTrackingTuneCommand extends Command {

  private final SmartTurretController controller;
  private final TalonFX talonFX;
  private final Timer timer = new Timer();

  private final double maxSafeAmplitudeRot;

  private double amplitudeRotations;
  private double frequencyHz;

  // Cached gain values for change detection.
  private double lastKP = -1, lastKI = -1, lastKD = -1;
  private double lastKV = -1, lastKA = -1;

  private static final double SAFETY_MARGIN_ROT = 10.0 / 360.0; // 10 degrees
  private static final String PREFIX = "TurretTrackTune/";

  public TurretTrackingTuneCommand(SmartTurretController controller, SubsystemBase requirement) {
    this.controller = controller;
    this.talonFX = controller.getTalonFX();
    addRequirements(requirement);

    // Compute max safe amplitude: min(|lower|, |upper|) - safety margin, centered at 0.
    double lowerLimit = Math.abs(controller.getConfig().getLowerLimitRotations());
    double upperLimit = Math.abs(controller.getConfig().getUpperLimitRotations());
    maxSafeAmplitudeRot = Math.max(0, Math.min(lowerLimit, upperLimit) - SAFETY_MARGIN_ROT);
  }

  @Override
  public void initialize() {
    timer.restart();

    // Initialize SmartDashboard defaults.
    amplitudeRotations = SmartDashboard.getNumber(PREFIX + "Amplitude (rot)", 0.1);
    frequencyHz = SmartDashboard.getNumber(PREFIX + "Frequency (Hz)", 0.5);
    SmartDashboard.putNumber(PREFIX + "Amplitude (rot)", amplitudeRotations);
    SmartDashboard.putNumber(PREFIX + "Frequency (Hz)", frequencyHz);

    double currentKP = controller.getConfig().getTrackingKP();
    double currentKI = controller.getConfig().getTrackingKI();
    double currentKD = controller.getConfig().getTrackingKD();
    SmartDashboard.putNumber(PREFIX + "Tracking kP", currentKP);
    SmartDashboard.putNumber(PREFIX + "Tracking kI", currentKI);
    SmartDashboard.putNumber(PREFIX + "Tracking kD", currentKD);
    lastKP = currentKP;
    lastKI = currentKI;
    lastKD = currentKD;

    // Publish tunable feedforward gains (Amps). kV and kA run in the firmware slot.
    double currentKV = controller.getConfig().getKV();
    double currentKA = controller.getConfig().getKA();
    SmartDashboard.putNumber(PREFIX + "Tracking kV (A/rps)", currentKV);
    SmartDashboard.putNumber(PREFIX + "Tracking kA (A/rps2)", currentKA);
    lastKV = currentKV;
    lastKA = currentKA;

    Logger.recordOutput(PREFIX + "MaxSafeAmplitudeRot", maxSafeAmplitudeRot);
  }

  @Override
  public void execute() {
    // Read tunable parameters.
    amplitudeRotations = SmartDashboard.getNumber(PREFIX + "Amplitude (rot)", 0.1);
    frequencyHz = SmartDashboard.getNumber(PREFIX + "Frequency (Hz)", 0.5);

    // Clamp amplitude to safe range.
    amplitudeRotations = MathUtil.clamp(amplitudeRotations, 0, maxSafeAmplitudeRot);

    double t = timer.get();
    double omega = 2.0 * Math.PI * frequencyHz;

    // Sinusoidal position reference (mechanism rotations).
    double positionRot = amplitudeRotations * Math.sin(omega * t);
    // Analytical velocity (rad/s): d/dt [A * sin(wt)] * 2pi = A * w * cos(wt) * 2pi
    double velocityRadPerSec = amplitudeRotations * omega * Math.cos(omega * t) * 2.0 * Math.PI;
    // Analytical acceleration (rad/s^2): d/dt [velocity] = -A * w^2 * sin(wt) * 2pi
    double accelRadPerSecSq =
        -amplitudeRotations * omega * omega * Math.sin(omega * t) * 2.0 * Math.PI;

    controller.setTarget(Rotations.of(positionRot), velocityRadPerSec, accelRadPerSecSq);

    // Check if any gains changed on dashboard and apply Slot1 update.
    double newKP = SmartDashboard.getNumber(PREFIX + "Tracking kP", lastKP);
    double newKI = SmartDashboard.getNumber(PREFIX + "Tracking kI", lastKI);
    double newKD = SmartDashboard.getNumber(PREFIX + "Tracking kD", lastKD);
    double newKV = SmartDashboard.getNumber(PREFIX + "Tracking kV (A/rps)", lastKV);
    double newKA = SmartDashboard.getNumber(PREFIX + "Tracking kA (A/rps2)", lastKA);

    if (newKP != lastKP
        || newKI != lastKI
        || newKD != lastKD
        || newKV != lastKV
        || newKA != lastKA) {
      Slot1Configs slot1 = new Slot1Configs();
      slot1.kP = newKP;
      slot1.kI = newKI;
      slot1.kD = newKD;
      // kS = 0: position-dependent kS is injected externally by SmartTurretController.
      slot1.kS = 0;
      slot1.kV = newKV;
      slot1.kA = newKA;
      talonFX.getConfigurator().apply(slot1);
      lastKP = newKP;
      lastKI = newKI;
      lastKD = newKD;
      lastKV = newKV;
      lastKA = newKA;
    }

    // Log tracking data via AdvantageKit.
    double actualPos = controller.getActualPositionMechRot();
    double trackingError = positionRot - actualPos;

    Logger.recordOutput(PREFIX + "ReferencePositionRot", positionRot);
    Logger.recordOutput(PREFIX + "ActualPositionRot", actualPos);
    Logger.recordOutput(PREFIX + "TrackingErrorRot", trackingError);
    Logger.recordOutput(PREFIX + "TrackingErrorDeg", trackingError * 360.0);
    Logger.recordOutput(PREFIX + "VelocityFFRadPerSec", velocityRadPerSec);
    Logger.recordOutput(PREFIX + "AccelFFRadPerSecSq", accelRadPerSecSq);
    Logger.recordOutput(PREFIX + "ClampedAmplitudeRot", amplitudeRotations);
  }

  @Override
  public void end(boolean interrupted) {
    controller.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // Run until interrupted.
  }
}
