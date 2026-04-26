package frc.robot.rebuilt.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import frc.robot.rebuilt.subsystems.Launcher.ShotCalculator;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShotTuningCommand extends Command {

  private static final String PREFIX = "ShotTuning/";

  private static final double HOOD_ANGLE_STEP_DEGREES = 0.5;
  private static final double FLYWHEEL_SPEED_STEP_RPM = 10.0;

  private static final Map<Double, TunedValues> currentTunedValues = new HashMap<>();

  private static final Map<Double, TunedValues> confirmedOptimalShots = new HashMap<>();

  private static Launcher staticLauncher;
  private static DoubleSupplier distanceSupplier;
  private final Launcher launcher;
  private final ShotCalculator shotCalculator;

  private DoubleSupplier hoodAngleSupplier;
  private DoubleSupplier flywheelSpeedSupplier;
  private DoubleSupplier turretAngleSupplier;

  private static double pendingHoodAdjustment = 0.0;
  private static double pendingFlywheelAdjustment = 0.0;

  private static double lastKnownDistance = Double.NaN;

  private static String lastAdjustmentMessage = "No adjustments yet";
  private static String lastConfirmMessage = "No confirmations yet";
  private static double lastConfirmedDistance = Double.NaN;
  private static double lastConfirmedHood = Double.NaN;
  private static double lastConfirmedRpm = Double.NaN;

  public ShotTuningCommand(Launcher launcher, Subsystem... requirements) {

    this.launcher = launcher;
    ShotTuningCommand.staticLauncher = launcher;
    this.shotCalculator = ShotCalculator.getInstance();
    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    pendingHoodAdjustment = 0.0;
    pendingFlywheelAdjustment = 0.0;
    currentTunedValues.clear();
  }

  public ShotTuningCommand withDistanceSupplier(DoubleSupplier supplier) {
    this.distanceSupplier = supplier;
    ShotTuningCommand.distanceSupplier = supplier;
    return this;
  }

  public ShotTuningCommand withHoodAngleSupplier(DoubleSupplier supplier) {
    this.hoodAngleSupplier = supplier;
    return this;
  }

  public ShotTuningCommand withFlywheelSpeedSupplier(DoubleSupplier supplier) {
    this.flywheelSpeedSupplier = supplier;
    return this;
  }

  public ShotTuningCommand withTurretAngleSupplier(DoubleSupplier supplier) {
    this.turretAngleSupplier = supplier;
    return this;
  }

  @Override
  public void execute() {
    if (distanceSupplier == null
        || hoodAngleSupplier == null
        || flywheelSpeedSupplier == null
        || turretAngleSupplier == null) {
      return;
    }

    double currentDistance = distanceSupplier.getAsDouble();
    double currentHoodAngle = hoodAngleSupplier.getAsDouble();
    double currentFlywheelRpm = flywheelSpeedSupplier.getAsDouble();
    double currentTurretAngle = turretAngleSupplier.getAsDouble();

    if (Double.isNaN(currentDistance) || currentDistance <= 0) {
      return;
    }

    lastKnownDistance = currentDistance;

    TunedValues tuned =
        currentTunedValues.computeIfAbsent(
            currentDistance,
            d ->
                new TunedValues(
                    shotCalculator.getLookupHoodAngleDegrees(d),
                    shotCalculator.getLookupFlywheelSpeed(d)));

    double newHoodAngle = tuned.baseHoodAngle + pendingHoodAdjustment;
    double newFlywheelRpm = tuned.baseFlywheelSpeed + pendingFlywheelAdjustment;

    launcher.usePresets(
        Degrees.of(newHoodAngle), Degrees.of(currentTurretAngle), RPM.of(newFlywheelRpm));

    Logger.recordOutput(PREFIX + "CurrentDistance", currentDistance);
    Logger.recordOutput(PREFIX + "CurrentHoodAngle", newHoodAngle);
    Logger.recordOutput(PREFIX + "CurrentFlywheelRPM", newFlywheelRpm);
    Logger.recordOutput(PREFIX + "BaseHoodAngle", tuned.baseHoodAngle);
    Logger.recordOutput(PREFIX + "BaseFlywheelSpeed", tuned.baseFlywheelSpeed);
    Logger.recordOutput(PREFIX + "PendingHoodAdj", pendingHoodAdjustment);
    Logger.recordOutput(PREFIX + "PendingFlywheelAdj", pendingFlywheelAdjustment);
    Logger.recordOutput(PREFIX + "LastAdjustment", lastAdjustmentMessage);
    Logger.recordOutput(PREFIX + "LastConfirm", lastConfirmMessage);
  }

  public static void increaseHoodAngle() {
    adjustHoodAngle(HOOD_ANGLE_STEP_DEGREES);
  }

  public static void decreaseHoodAngle() {
    adjustHoodAngle(-HOOD_ANGLE_STEP_DEGREES);
  }

  private static void adjustHoodAngle(double amount) {
    pendingHoodAdjustment += amount;

    double currentDist = computeCurrentDistance();
    if (Double.isNaN(currentDist) || currentDist <= 0) {
      return;
    }

    TunedValues tuned = currentTunedValues.get(currentDist);
    if (tuned == null) {
      return;
    }

    double newAngle = tuned.baseHoodAngle + pendingHoodAdjustment;
    double oldAngle = newAngle - amount;

    String direction = amount > 0 ? "UP" : "DOWN";
    lastAdjustmentMessage =
        String.format(
            "[ShotTuning] %s @ %.2fm -> hood: %.1f deg (was %.1f deg)",
            direction, currentDist, newAngle, oldAngle);

    System.out.println(lastAdjustmentMessage);

    staticLauncher.usePresets(
        Degrees.of(newAngle),
        Degrees.of(staticLauncher.getTurretAngleActual().in(Degrees)),
        RPM.of(tuned.baseFlywheelSpeed + pendingFlywheelAdjustment));

    updateShotTable(currentDist, newAngle, tuned.baseFlywheelSpeed + pendingFlywheelAdjustment);
  }

  public static void increaseFlywheelSpeed() {
    adjustFlywheelSpeed(FLYWHEEL_SPEED_STEP_RPM);
  }

  public static void decreaseFlywheelSpeed() {
    adjustFlywheelSpeed(-FLYWHEEL_SPEED_STEP_RPM);
  }

  private static void adjustFlywheelSpeed(double amount) {
    pendingFlywheelAdjustment += amount;

    double currentDist = computeCurrentDistance();
    if (Double.isNaN(currentDist) || currentDist <= 0) {
      return;
    }

    TunedValues tuned = currentTunedValues.get(currentDist);
    if (tuned == null) {
      return;
    }

    double newRpm = tuned.baseFlywheelSpeed + pendingFlywheelAdjustment;
    double oldRpm = newRpm - amount;

    String direction = amount > 0 ? "UP" : "DOWN";
    lastAdjustmentMessage =
        String.format(
            "[ShotTuning] %s @ %.2fm -> RPM: %.0f (was %.0f)",
            direction, currentDist, newRpm, oldRpm);

    System.out.println(lastAdjustmentMessage);

    staticLauncher.usePresets(
        Degrees.of(tuned.baseHoodAngle + pendingHoodAdjustment),
        Degrees.of(staticLauncher.getTurretAngleActual().in(Degrees)),
        RPM.of(newRpm));

    updateShotTable(currentDist, tuned.baseHoodAngle + pendingHoodAdjustment, newRpm);
  }

  public static void confirmOptimal() {
    double currentDist = computeCurrentDistance();
    if (Double.isNaN(currentDist) || currentDist <= 0) {
      System.out.println("[ShotTuning] Cannot confirm - no valid distance");
      return;
    }

    TunedValues tuned = currentTunedValues.get(currentDist);
    if (tuned == null) {
      System.out.println("[ShotTuning] Cannot confirm - no tuned values for this distance");
      return;
    }

    double confirmedHood = tuned.baseHoodAngle + pendingHoodAdjustment;
    double confirmedRpm = tuned.baseFlywheelSpeed + pendingFlywheelAdjustment;

    confirmedOptimalShots.put(currentDist, new TunedValues(confirmedHood, confirmedRpm));

    lastConfirmedDistance = currentDist;
    lastConfirmedHood = confirmedHood;
    lastConfirmedRpm = confirmedRpm;

    lastConfirmMessage =
        String.format(
            "[ShotTuning] CONFIRMED OPTIMAL - Distance: %.2fm, Hood: %.1f deg, RPM: %.0f",
            currentDist, confirmedHood, confirmedRpm);

    System.out.println(lastConfirmMessage);

    String logKey = PREFIX + "Confirmed/Distance_" + String.format("%.2f", currentDist);
    Logger.recordOutput(
        logKey, String.format("hood: %.1f deg, RPM: %.0f", confirmedHood, confirmedRpm));
  }

  public static void resetAdjustments() {
    double currentDist = computeCurrentDistance();
    if (Double.isNaN(currentDist) || currentDist <= 0) {
      pendingHoodAdjustment = 0.0;
      pendingFlywheelAdjustment = 0.0;
      return;
    }

    TunedValues tuned = currentTunedValues.get(currentDist);
    if (tuned == null) {
      pendingHoodAdjustment = 0.0;
      pendingFlywheelAdjustment = 0.0;
      return;
    }

    pendingHoodAdjustment = 0.0;
    pendingFlywheelAdjustment = 0.0;

    lastAdjustmentMessage =
        String.format(
            "[ShotTuning] RESET @ %.2fm -> hood: %.1f deg, RPM: %.0f",
            currentDist, tuned.baseHoodAngle, tuned.baseFlywheelSpeed);

    System.out.println(lastAdjustmentMessage);

    staticLauncher.usePresets(
        Degrees.of(tuned.baseHoodAngle),
        Degrees.of(staticLauncher.getTurretAngleActual().in(Degrees)),
        RPM.of(tuned.baseFlywheelSpeed));

    updateShotTable(currentDist, tuned.baseHoodAngle, tuned.baseFlywheelSpeed);
  }

  public static double getLastKnownDistance() {
    return lastKnownDistance;
  }

  private static double computeCurrentDistance() {
    if (distanceSupplier != null) {
      double dist = distanceSupplier.getAsDouble();
      if (!Double.isNaN(dist) && dist > 0) {
        lastKnownDistance = dist;
        return dist;
      }
    }
    return lastKnownDistance;
  }

  private static void updateShotTable(double distance, double hoodAngle, double flywheelRpm) {
    ShotCalculator.getInstance()
        .addDataPoint(distance, hoodAngle, flywheelRpm, estimateTimeOfFlight(distance));
  }

  private static double estimateTimeOfFlight(double distanceMeters) {
    return distanceMeters / 15.0;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("[ShotTuning] Tuning ended. Interrupted: " + interrupted);
  }

  private static class TunedValues {
    double baseHoodAngle;
    double baseFlywheelSpeed;

    TunedValues(double baseHoodAngle, double baseFlywheelSpeed) {
      this.baseHoodAngle = Double.isNaN(baseHoodAngle) ? 35.0 : baseHoodAngle;
      this.baseFlywheelSpeed = Double.isNaN(baseFlywheelSpeed) ? 1050.0 : baseFlywheelSpeed;
    }
  }
}
