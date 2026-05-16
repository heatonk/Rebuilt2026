package frc.robot.rebuilt.commands;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.rebuilt.subsystems.Launcher.SmartTurretController;
import org.littletonrobotics.junction.Logger;

/**
 * Tunes the SEEKING (MotionMagicExpo) mode by alternating between two target positions.
 *
 * <p>All Slot0 PID/FF gains and the MotionMagicExpo plant model (kV/kA in Volts) are adjustable via
 * SmartDashboard in real-time. The target positions and dwell time at each target are also tunable.
 *
 * <p>This lets the operator observe overshoot, settling time, and profile shape by simply watching
 * the turret move back and forth while tweaking gains on the dashboard.
 */
public class TurretSeekingTuneCommand extends Command {

  private final SmartTurretController controller;
  private final TalonFX talonFX;
  private final Timer timer = new Timer();
  private final Timer settleTimer = new Timer();

  private boolean atTargetA = true;

  // Cached Slot0 gains for change detection.
  private double lastKP = -1, lastKI = -1, lastKD = -1;
  private double lastKS = -1, lastKV = -1, lastKA = -1;
  // Cached expo gains for change detection.
  private double lastExpoKV = -1, lastExpoKA = -1;

  private static final String PREFIX = "TurretSeekTune/";

  public TurretSeekingTuneCommand(SmartTurretController controller, SubsystemBase requirement) {
    this.controller = controller;
    this.talonFX = controller.getTalonFX();
    addRequirements(requirement);
  }

  @Override
  public void initialize() {
    timer.restart();
    settleTimer.restart();
    atTargetA = true;

    var config = controller.getConfig();
    double lowerDeg = config.getLowerLimitRotations() * 360.0;
    double upperDeg = config.getUpperLimitRotations() * 360.0;

    // Target positions (degrees) — default to ±50% of range from center.
    SmartDashboard.putNumber(
        PREFIX + "Target A (deg)",
        SmartDashboard.getNumber(PREFIX + "Target A (deg)", lowerDeg * 0.5));
    SmartDashboard.putNumber(
        PREFIX + "Target B (deg)",
        SmartDashboard.getNumber(PREFIX + "Target B (deg)", upperDeg * 0.5));
    SmartDashboard.putNumber(
        PREFIX + "Dwell Time (s)", SmartDashboard.getNumber(PREFIX + "Dwell Time (s)", 2.0));

    // Slot0 PID gains.
    SmartDashboard.putNumber(PREFIX + "Seeking kP", config.getSeekingKP());
    SmartDashboard.putNumber(PREFIX + "Seeking kI", config.getSeekingKI());
    SmartDashboard.putNumber(PREFIX + "Seeking kD", config.getSeekingKD());
    lastKP = config.getSeekingKP();
    lastKI = config.getSeekingKI();
    lastKD = config.getSeekingKD();

    // Slot0 feedforward gains (Amps — TorqueCurrentFOC).
    SmartDashboard.putNumber(PREFIX + "Seeking kS (A)", config.getKS());
    SmartDashboard.putNumber(PREFIX + "Seeking kV (A/rps)", config.getKV());
    SmartDashboard.putNumber(PREFIX + "Seeking kA (A/rps2)", config.getKA());
    lastKS = config.getKS();
    lastKV = config.getKV();
    lastKA = config.getKA();

    // Expo plant model (Volts — always V/rps and V/rps^2).
    SmartDashboard.putNumber(PREFIX + "Expo kV (V/rps)", config.getExpoKV());
    SmartDashboard.putNumber(PREFIX + "Expo kA (V/rps2)", config.getExpoKA());
    lastExpoKV = config.getExpoKV();
    lastExpoKA = config.getExpoKA();
  }

  @Override
  public void execute() {
    // Read target positions and dwell time.
    double targetADeg = SmartDashboard.getNumber(PREFIX + "Target A (deg)", -60);
    double targetBDeg = SmartDashboard.getNumber(PREFIX + "Target B (deg)", 60);
    double dwellTimeSec = SmartDashboard.getNumber(PREFIX + "Dwell Time (s)", 2.0);

    // Switch targets after dwell time elapses.
    if (settleTimer.hasElapsed(dwellTimeSec)) {
      atTargetA = !atTargetA;
      settleTimer.restart();
    }

    double targetDeg = atTargetA ? targetADeg : targetBDeg;
    double targetRot = targetDeg / 360.0;

    // Command the turret (zero velocity/accel FF — SEEKING mode handles the profile).
    controller.setTarget(Rotations.of(targetRot), 0, 0);

    // --- Live gain updates from SmartDashboard ---
    double newKP = SmartDashboard.getNumber(PREFIX + "Seeking kP", lastKP);
    double newKI = SmartDashboard.getNumber(PREFIX + "Seeking kI", lastKI);
    double newKD = SmartDashboard.getNumber(PREFIX + "Seeking kD", lastKD);
    double newKS = SmartDashboard.getNumber(PREFIX + "Seeking kS (A)", lastKS);
    double newKV = SmartDashboard.getNumber(PREFIX + "Seeking kV (A/rps)", lastKV);
    double newKA = SmartDashboard.getNumber(PREFIX + "Seeking kA (A/rps2)", lastKA);

    if (newKP != lastKP
        || newKI != lastKI
        || newKD != lastKD
        || newKS != lastKS
        || newKV != lastKV
        || newKA != lastKA) {
      Slot0Configs slot0 = new Slot0Configs();
      slot0.kP = newKP;
      slot0.kI = newKI;
      slot0.kD = newKD;
      slot0.kS = newKS;
      slot0.kV = newKV;
      slot0.kA = newKA;
      talonFX.getConfigurator().apply(slot0);
      lastKP = newKP;
      lastKI = newKI;
      lastKD = newKD;
      lastKS = newKS;
      lastKV = newKV;
      lastKA = newKA;
    }

    // Expo plant model updates (Volts).
    double newExpoKV = SmartDashboard.getNumber(PREFIX + "Expo kV (V/rps)", lastExpoKV);
    double newExpoKA = SmartDashboard.getNumber(PREFIX + "Expo kA (V/rps2)", lastExpoKA);

    if (newExpoKV != lastExpoKV || newExpoKA != lastExpoKA) {
      MotionMagicConfigs mmConfig = new MotionMagicConfigs();
      mmConfig.MotionMagicExpo_kV = newExpoKV;
      mmConfig.MotionMagicExpo_kA = newExpoKA;
      mmConfig.MotionMagicCruiseVelocity = controller.getConfig().getMaxVelocityMechRotPerSec();
      talonFX.getConfigurator().apply(mmConfig);
      lastExpoKV = newExpoKV;
      lastExpoKA = newExpoKA;
    }

    // Logging.
    double actualPos = controller.getActualPositionMechRot();
    double positionError = targetRot - actualPos;

    Logger.recordOutput(PREFIX + "TargetPositionDeg", targetDeg);
    Logger.recordOutput(PREFIX + "ActualPositionDeg", actualPos * 360.0);
    Logger.recordOutput(PREFIX + "PositionErrorDeg", positionError * 360.0);
    Logger.recordOutput(
        PREFIX + "ActualVelocityRPS", controller.getActualVelocityRadPerSec() / (2.0 * Math.PI));
    Logger.recordOutput(PREFIX + "AtTargetA", atTargetA);
    Logger.recordOutput(PREFIX + "State", controller.getCurrentTurretState().name());
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
