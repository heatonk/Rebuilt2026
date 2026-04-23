// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.FieldConstants;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.commands.IndexerCommands.IndexerState;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import frc.robot.rebuilt.subsystems.intake.IntakeIOSim;
import java.util.Map;
import java.util.TreeMap;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.drive.GenericDrivetrain;
import org.littletonrobotics.junction.Logger;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceProjectile;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

/** Add your docs here. */
public class LauncherIOSim extends LauncherIOReal {
  private static final double TURRET_SIM_PERIOD_SECONDS = 0.02;

  private static final double AVG_BPS = 9.0;
  private static final double AVG_TIME_BETWEEN_SHOTS = 1.0 / AVG_BPS;
  private static final double VARIATION_IN_SHOT_RATE_SECONDS = 0.05;
  private double nextShotTimeSeconds = 0.0;

  protected GamePieceProjectile gamePieceProjectile;
  private final TalonFXSimState turretTalonSimState;
  private final CANcoderSimState crtEncoder40SimState;
  private final CANcoderSimState crtEncoder36SimState;
  private final double turretEncoder40RotationsPerMechanismRotation;
  private final double turretEncoder36RotationsPerMechanismRotation;
  private final double turretMaxVelocityRotationsPerSecond;
  private double simulatedTurretPositionRotations;
  private double simulatedTurretVelocityRotationsPerSecond;
  // The commanded target position — set directly by setTurretRotation/WithFeedforward overrides.
  // This replaces reading back from YAMS getMechanismPositionSetpoint(), which is unreliable once
  // the YAMS trapezoidal profile completes (returns Optional.empty()) and can cause the sim turret
  // to lose track of the last commanded position after an autonomous routine ends.
  private double simDesiredTurretPositionRotations;

  public LauncherIOSim(Map<String, Object> devices, Map<String, GenericSubsystem> subsystems) {
    super(devices, subsystems, false);

    Object rawController = turret.getMotorController().getMotorController();
    turretTalonSimState = rawController instanceof TalonFX talonFX ? talonFX.getSimState() : null;
    crtEncoder40SimState = crtEncoder40.getSimState();
    crtEncoder36SimState = crtEncoder36.getSimState();
    turretEncoder40RotationsPerMechanismRotation =
        TURRET_GEAR_RATIO * CRT_DRIVE_GEAR_TEETH / CRT_ENCODER_40_TEETH;
    turretEncoder36RotationsPerMechanismRotation =
        TURRET_GEAR_RATIO * CRT_DRIVE_GEAR_TEETH / CRT_ENCODER_36_TEETH;
    turretMaxVelocityRotationsPerSecond =
        turret
            .getMotorController()
            .getConfig()
            .getTrapezoidProfile()
            .map(c -> c.maxVelocity)
            .orElse(3.0);
    simulatedTurretPositionRotations =
        MathUtil.clamp(
            turret.getAngle().in(Rotations),
            turretLowLimit.in(Rotations),
            turretHighLimit.in(Rotations));
    simulatedTurretVelocityRotationsPerSecond = 0.0;
    simDesiredTurretPositionRotations = simulatedTurretPositionRotations;
    syncTurretSimulationState();

    IntakeIOSim.intakeSimulation.addGamePiecesToIntake(8);
    // Start with 8 gamepieces in the
    // intake
  }

  @Override
  /** Configures the shot calculator and calculates measurements for parts of the lancher */
  public void configureShotCalculator(ShotCalculator shotCalculator) {
    super.configureShotCalculator(shotCalculator);
    double circumferenceMeters = flyWheel.getShooterConfig().getCircumference().in(Meters);
    double wheelRadiusMeters = circumferenceMeters / (2.0 * Math.PI);
    double minFlywheelRadPerSec =
        flyWheel.getShooterConfig().getLowerSoftLimit().orElse(RPM.of(0.0)).in(RadiansPerSecond);
    double maxFlywheelRadPerSec =
        flyWheel.getShooterConfig().getUpperSoftLimit().orElse(RPM.of(5000.0)).in(RadiansPerSecond);
    /** Reads the hood angle limits */
    Rotation2d minHoodAngle =
        Rotation2d.fromDegrees(
            hood.getMotorController().getConfig().getMechanismLowerLimit().get().in(Degrees));
    Rotation2d maxHoodAngle =
        Rotation2d.fromDegrees(
            hood.getMotorController().getConfig().getMechanismUpperLimit().get().in(Degrees));
    Rotation2d hoodStep = Rotation2d.fromDegrees(0.5);

    double launchHeight = flyWheel.getRelativeMechanismPosition().getZ();
    double targetHeight = FieldConstants.Hub.height;
    /** Creates ballistic configuration for the shot calculator */
    ShotCalculator.BallisticConfig config =
        new ShotCalculator.BallisticConfig(
            1.0,
            6.0,
            0.1,
            minHoodAngle,
            maxHoodAngle,
            hoodStep,
            minFlywheelRadPerSec,
            maxFlywheelRadPerSec,
            wheelRadiusMeters,
            launchHeight,
            targetHeight,
            0.0,
            9.80665,
            Math.toRadians(90.0));

    shotCalculator.setBallisticConfig(config);
    ShotCalculator.ShotTables ballisticTables = ShotCalculator.createBallisticTables(config);
    double exitSpeedPerCommandRpm = getFlyWheelExitSpeed(RPM.of(1.0)).in(MetersPerSecond);
    Map<Double, Double> simFlywheelSpeeds = new TreeMap<>();
    ballisticTables
        .flywheelSpeeds()
        .forEach(
            (distanceMeters, ballisticFlywheelRadPerSec) -> {
              double launchVelocityMetersPerSecond = ballisticFlywheelRadPerSec * wheelRadiusMeters;
              simFlywheelSpeeds.put(
                  distanceMeters, launchVelocityMetersPerSecond / exitSpeedPerCommandRpm);
            });
    ShotCalculator.ShotTables simTables =
        new ShotCalculator.ShotTables(
            ballisticTables.hoodAngles(),
            simFlywheelSpeeds,
            ballisticTables.timeOfFlightSeconds(),
            ballisticTables.minDistanceMeters(),
            ballisticTables.maxDistanceMeters(),
            ballisticTables.phaseDelaySeconds());
    shotCalculator.setShotTables(simTables);
  }

  @Override
  public void updateSimulation(Launcher launcher, Indexer indexer) {
    updateTurretSimulation();

    int amount = IntakeIOSim.intakeSimulation.getGamePiecesAmount();
    // Update simulated mechanism states here
    // We should simulate a shot rate of about 10-15 gamepieces per second
    // Every other time this is called, determine a randome number and if > 0.5, shoot a gamepiece.
    // This would mean we try to shoot 25 times per second, and on average shoot about 12-13
    // gamepieces per second.

    if (Timer.getFPGATimestamp() >= nextShotTimeSeconds) {
      if ((indexer.isCurrent(IndexerState.FEED) && launcher.isShooting())
          || (indexer.isCurrent(IndexerState.FORCE))) {
        if (IntakeIOSim.intakeSimulation.obtainGamePieceFromIntake()) {
          Pose2d worldPose = getSimulationRobotPose();
          gamePieceProjectile =
              new RebuiltFuelOnFly(
                      worldPose.getTranslation(),
                      flyWheel.getRelativeMechanismPosition().toTranslation2d(),
                      Rebuilt.drivetrain.getFieldVelocity(),
                      Rotation2d.fromDegrees(
                          worldPose.getRotation().getMeasure().plus(getTurretAngle()).in(Degrees)),
                      flyWheel.getRelativeMechanismPosition().getMeasureZ(),
                      getFlyWheelExitSpeed(flyWheel.getSpeed()),
                      Degrees.of(90.0).minus(hood.getAngle()))
                  .withProjectileTrajectoryDisplayCallBack(
                      (pose3ds) -> {
                        Logger.recordOutput(
                            "Launcher/GamePieceTrajectory", pose3ds.toArray(Pose3d[]::new));
                      });
          SimulatedArena.getInstance().addGamePieceProjectile(gamePieceProjectile);
          // Create a new gamepiece on-the-fly and add it to the field simulation
          nextShotTimeSeconds =
              Timer.getFPGATimestamp()
                  + AVG_TIME_BETWEEN_SHOTS
                  + (Math.random() - 0.5) * 2 * VARIATION_IN_SHOT_RATE_SECONDS;
        }
      }
    }
  }

  @Override
  public void zeroTurret() {
    super.zeroTurret();
    double zeroPosition =
        MathUtil.clamp(
            HARD_STOP.in(Rotations), turretLowLimit.in(Rotations), turretHighLimit.in(Rotations));
    simulatedTurretPositionRotations = zeroPosition;
    simDesiredTurretPositionRotations = zeroPosition;
    simulatedTurretVelocityRotationsPerSecond = 0.0;
    syncTurretSimulationState();
  }

  @Override
  public void setTurretRotation(Angle angle) {
    simDesiredTurretPositionRotations =
        MathUtil.clamp(
            angle.in(Rotations), turretLowLimit.in(Rotations), turretHighLimit.in(Rotations));
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    super.updateInputs(inputs);
    inputs.turretAngleDesired = Rotations.of(simDesiredTurretPositionRotations);
    inputs.turretAngleError = inputs.turretAngleActual.minus(inputs.turretAngleDesired).in(Degrees);
    inputs.turretAngleAtGoal =
        Math.abs(inputs.turretAngleError) <= Constants.Launcher.TURRET_ANGLE_TOLERANCE_DEGREES;
  }

  @Override
  public void setTurretRotationWithFeedforward(
      Angle angle, double feedforwardRadPerSec, double accelerationRadPerSecSq) {
    simDesiredTurretPositionRotations =
        MathUtil.clamp(
            angle.in(Rotations), turretLowLimit.in(Rotations), turretHighLimit.in(Rotations));
  }

  private void updateTurretSimulation() {
    // Use the directly-captured desired position rather than YAMS getMechanismPositionSetpoint().
    // YAMS's setpoint becomes Optional.empty() after the profile completes, causing the sim
    // turret to freeze at wherever it last arrived rather than tracking new commands.
    double desiredTurretPositionRotations = simDesiredTurretPositionRotations;

    double errorRotations = desiredTurretPositionRotations - simulatedTurretPositionRotations;
    double maxStepRotations = turretMaxVelocityRotationsPerSecond * TURRET_SIM_PERIOD_SECONDS;
    double appliedStepRotations =
        MathUtil.clamp(errorRotations, -maxStepRotations, maxStepRotations);

    simulatedTurretPositionRotations += appliedStepRotations;
    if (Math.abs(errorRotations) <= maxStepRotations) {
      simulatedTurretPositionRotations = desiredTurretPositionRotations;
      simulatedTurretVelocityRotationsPerSecond = 0.0;
    } else {
      simulatedTurretVelocityRotationsPerSecond = appliedStepRotations / TURRET_SIM_PERIOD_SECONDS;
    }

    syncTurretSimulationState();

    Logger.recordOutput("Launcher/TurretSim/DesiredPositionRot", desiredTurretPositionRotations);
    Logger.recordOutput(
        "Launcher/TurretSim/MechanismPositionRot", simulatedTurretPositionRotations);
    Logger.recordOutput(
        "Launcher/TurretSim/MechanismVelocityRotPerSec", simulatedTurretVelocityRotationsPerSecond);
  }

  @Override
  protected Angle getTurretAngle() {
    return Rotations.of(simulatedTurretPositionRotations);
  }

  @Override
  protected double getTurretVelocityDegreesPerSecond() {
    return simulatedTurretVelocityRotationsPerSecond * 360.0;
  }

  private void syncTurretSimulationState() {
    turret.getMotor().setEncoderPosition(Rotations.of(simulatedTurretPositionRotations));
    if (turretTalonSimState != null) {
      turretTalonSimState.setSupplyVoltage(Volts.of(RobotController.getBatteryVoltage()));
      turretTalonSimState.setRawRotorPosition(
          Rotations.of(simulatedTurretPositionRotations * TURRET_GEAR_RATIO));
      turretTalonSimState.setRotorVelocity(
          RotationsPerSecond.of(simulatedTurretVelocityRotationsPerSecond * TURRET_GEAR_RATIO));
    }

    double encoder40RawRotations =
        wrapAbsoluteRotation(
            encoder40Offset
                - simulatedTurretPositionRotations * turretEncoder40RotationsPerMechanismRotation);
    double encoder36RawRotations =
        wrapAbsoluteRotation(
            encoder36Offset
                + simulatedTurretPositionRotations * turretEncoder36RotationsPerMechanismRotation);
    double encoder40VelocityRotationsPerSecond =
        -simulatedTurretVelocityRotationsPerSecond * turretEncoder40RotationsPerMechanismRotation;
    double encoder36VelocityRotationsPerSecond =
        simulatedTurretVelocityRotationsPerSecond * turretEncoder36RotationsPerMechanismRotation;

    crtEncoder40SimState.setRawPosition(Rotations.of(encoder40RawRotations));
    crtEncoder40SimState.setVelocity(RotationsPerSecond.of(encoder40VelocityRotationsPerSecond));
    crtEncoder36SimState.setRawPosition(Rotations.of(encoder36RawRotations));
    crtEncoder36SimState.setVelocity(RotationsPerSecond.of(encoder36VelocityRotationsPerSecond));
  }

  private double wrapAbsoluteRotation(double rotations) {
    return MathUtil.inputModulus(rotations, 0.0, 1.0);
  }

  private Pose2d getSimulationRobotPose() {
    return GenericDrivetrain.getMapleSimDrive()
        .map(it -> it.getSimulatedDriveTrainPose())
        .orElse(Rebuilt.drivetrain.getPoseEstimator().getCurrentPose());
  }
}
