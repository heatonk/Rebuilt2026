// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.commands.IntakeCommands.IntakeState;
import frc.robot.rebuilt.commands.LauncherCommands;
import frc.robot.rebuilt.subsystems.intake.Intake;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.motors.SystemIdentification;
import org.frc5010.common.subsystems.LEDStrip;
import org.frc5010.common.vision.AprilTags;
import yams.mechanisms.config.SensorConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.simulation.Sensor;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

/** Add your docs here. */
public class LauncherIOReal implements LauncherIO {

  protected Map<String, Object> devices;
  protected Pivot turret;
  protected Arm hood;
  protected GenericDrivetrain drivetrain;
  protected FlyWheel flyWheel;
  protected CANcoder crtEncoder40;
  protected CANcoder crtEncoder36;
  protected final Sensor crtSensor40;
  protected final Sensor crtSensor36;
  protected EasyCRT easyCrtSolver;
  /** Initializes the launcher hardware, encoders, simulated sensors, and angle solver */
  EasyCRTConfig easyCrt;

  private boolean isNearTrench = false;
  private IntakeState lastState = IntakeState.RETRACTED;

  protected Intake intake;

  protected static Translation2d robotToTurret;

  Angle turretLowLimit = Degrees.of(-90);
  Angle turretHighLimit = Degrees.of(90);

  /**
   * Turret motor kV in V/(rot/s), read from the YAMS {@link
   * yams.motorcontrollers.SmartMotorControllerConfig#getSimpleFeedforward()} at construction time
   * so that the feedforward voltage conversion stays in sync with the tuned gains.
   */
  protected double turretKv = 0.0;

  /** RoboRIO-side trapezoidal profile controller for the turret. */
  protected TurretProfileController turretProfileController;

  public LauncherIOReal(Map<String, Object> devices, Map<String, GenericSubsystem> subsystems) {
    this.devices = devices;
    drivetrain = (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);
    intake = (Intake) subsystems.get(Constants.INTAKE);
    turret = (Pivot) devices.get("turret");
    robotToTurret =
        turret
            .getPivotConfig()
            .getMechanismPositionConfig()
            .getRelativePosition()
            .get()
            .toTranslation2d();

    hood = (Arm) devices.get("hood");
    flyWheel = (FlyWheel) devices.get("flywheel");

    turretLowLimit =
        turret.getMotorController().getConfig().getMechanismLowerLimit().orElse(turretLowLimit);
    turretHighLimit =
        turret.getMotorController().getConfig().getMechanismUpperLimit().orElse(turretHighLimit);

    CANBus canivoreBus = new CANBus("canivore");
    crtEncoder40 = new CANcoder(21, canivoreBus);
    crtEncoder36 = new CANcoder(22, canivoreBus);
    double sensor40Sim = 0.391;
    double sensor36Sim = 0.274;
    crtSensor40 =
        new SensorConfig("CRT sensor 40")
            .withField("angle", () -> crtEncoder40.getAbsolutePosition().getValueAsDouble(), 0.0)
            .withSimulatedValue("angle", Seconds.of(0), Seconds.of(0.5), sensor40Sim)
            .getSensor();
    crtSensor36 =
        new SensorConfig("CRT sensor 36")
            .withField("angle", () -> crtEncoder36.getAbsolutePosition().getValueAsDouble(), 0.0)
            .withSimulatedValue("angle", Seconds.of(0), Seconds.of(0.5), sensor36Sim)
            .getSensor();

    easyCrt =
        new EasyCRTConfig(
                () -> Rotations.of(crtSensor40.getAsDouble("angle")),
                () -> Rotations.of(crtSensor36.getAsDouble("angle")))
            .withCommonDriveGear(
                /* commonRatio (mech:drive) */ 30.0,
                /* driveGearTeeth */ 12,
                /* encoder1Pinion */ 40,
                /* encoder2Pinion */ 36)
            .withAbsoluteEncoderOffsets( // -0.474609375
                Rotations.of(0.474609375),
                Rotations.of(-0.009521484375)) // set after mechanical zero
            .withMechanismRange(Degrees.of(-168), Degrees.of(173)) // -360 deg to +720 deg
            .withMatchTolerance(Rotations.of(0.06)) // ~1.08 deg at encoder2 for the example ratio
            .withAbsoluteEncoderInversions(true, false)
            .withCrtGearRecommendationConstraints(
                /* coverageMargin */ 1.2,
                /* minTeeth */ 15,
                /* maxTeeth */ 45,
                /* maxIterations */ 30);

    easyCrtSolver = new EasyCRT(easyCrt);
    // // Test Values
    SmartDashboard.putNumber(
        "EasyCRT/Unique Coverage", easyCrt.getUniqueCoverage().orElse(Degrees.of(0.0)).in(Degrees));
    SmartDashboard.putBoolean("EasyCRT/Coverage Satisfies Range", easyCrt.coverageSatisfiesRange());
    SmartDashboard.putNumber("EasyCRT/Enc 1", easyCrt.getAbsoluteEncoder1Angle().in(Degrees));
    SmartDashboard.putNumber(
        "EasyCRT/Enc 1 Ratio", easyCrt.getEncoder1RotationsPerMechanismRotation());
    SmartDashboard.putNumber("EasyCRT/Enc 2", easyCrt.getAbsoluteEncoder2Angle().in(Degrees));
    SmartDashboard.putNumber(
        "EasyCRT/Enc 2 Ratio", easyCrt.getEncoder2RotationsPerMechanismRotation());
    Angle calculatedAngle;
    Optional<Angle> optionalAngle = (easyCrtSolver.getAngleOptional());
    if (optionalAngle.isPresent()) {
      calculatedAngle = optionalAngle.get();
    } else {
      calculatedAngle = Degrees.of(0);
    }

    SmartDashboard.putNumber("EasyCRT/CRT Angle", calculatedAngle.in(Degrees));
    SmartDashboard.putString("EasyCRT/CRT Status", easyCrtSolver.getLastStatus().name());
    SmartDashboard.putNumber("EasyCRT/CRT Error Rot", easyCrtSolver.getLastErrorRotations());
    turret.getMotor().setEncoderPosition(calculatedAngle);

    // Read kV directly from the YAMS SmartMotorControllerConfig (populated from turret.json).
    // If the YAMS config doesn't carry a SimpleMotorFeedforward (e.g. old config), fall back to
    // reading the live TalonFX Slot0 so the value is still hardware-consistent.
    turretKv =
        turret
            .getMotorController()
            .getConfig()
            .getSimpleFeedforward()
            .map(ff -> ff.getKv())
            .orElseGet(
                () -> {
                  try {
                    Object rawController = turret.getMotorController().getMotorController();
                    if (rawController instanceof com.ctre.phoenix6.hardware.TalonFX talonFX) {
                      var cfg = new com.ctre.phoenix6.configs.TalonFXConfiguration();
                      talonFX.getConfigurator().refresh(cfg);
                      if (cfg.Slot0.kV > 0.0) return cfg.Slot0.kV;
                    }
                  } catch (Exception ignored) {
                  }
                  return 0.0;
                });

    // Create the RoboRIO-side trapezoidal profile controller for the turret.
    // This replaces the TalonFX-internal MotionMagic profile so that feedforward velocity
    // can be integrated into the profile goal rather than fighting the controller.
    {
      var turretConfig = turret.getMotorController().getConfig();
      var trapConstraints = turretConfig.getTrapezoidProfile();
      // YAMS stores maxVelocity/maxAcceleration in rot/s at the motor; convert to mechanism rot/s.
      double gearRatioValue = 30.0; // 30:1 gear ratio from turret.json
      double maxVelMechRotPerSec =
          trapConstraints.map(c -> c.maxVelocity / gearRatioValue).orElse(1080.0 / 360.0);
      double maxAccelMechRotPerSecSq =
          trapConstraints.map(c -> c.maxAcceleration / gearRatioValue).orElse(1000.0 / 360.0);
      double lowerLimitRot =
          turretConfig.getMechanismLowerLimit().orElse(Degrees.of(-150)).in(Rotations);
      double upperLimitRot =
          turretConfig.getMechanismUpperLimit().orElse(Degrees.of(150)).in(Rotations);

      Object rawController = turret.getMotorController().getMotorController();
      if (rawController instanceof com.ctre.phoenix6.hardware.TalonFX talonFXRaw) {
        turretProfileController =
            new TurretProfileController(
                talonFXRaw,
                maxVelMechRotPerSec,
                maxAccelMechRotPerSecSq,
                gearRatioValue,
                lowerLimitRot,
                upperLimitRot);
        // Reset profile to the CRT-solved initial position.
        turretProfileController.reset(calculatedAngle.in(Rotations), 0);
      }
    }

    turret.min().or(turret.max()).onTrue(Commands.runOnce(() -> turret.getMotor().setDutyCycle(0)));
  }

  public ShotCalculator.ShootingParameters getShootingParameters(
      Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d> targetPositionSupplier) {
    ShotCalculator.getInstance().clearShootingParameters();
    return ShotCalculator.getInstance()
        .getParameters(
            robotToTurret,
            Rotation2d.fromDegrees(turret.getAngle().in(Degrees)),
            robotPoseSupplier,
            targetPositionSupplier);
  }

  @Override()
  /** Updating launcher sensor data, calculates shot parameters, and populates input telemetry */
  public void updateInputs(LauncherIOInputs inputs) {
    SmartDashboard.putNumber("EasyCRT/Encoder 40", crtSensor40.getAsDouble("angle"));
    SmartDashboard.putNumber("EasyCRT/Enc 2", easyCrt.getAbsoluteEncoder2Angle().in(Degrees));
    SmartDashboard.putNumber("EasyCRT/Encoder 36", crtSensor36.getAsDouble("angle"));
    SmartDashboard.putNumber("EasyCRT/Enc 1", easyCrt.getAbsoluteEncoder1Angle().in(Degrees));
    SmartDashboard.putNumber(
        "Distance to tag 27",
        drivetrain
            .getPoseEstimator()
            .getCurrentPose3d()
            .toPose2d()
            .minus(AprilTags.aprilTagFieldLayout.getTagPose(21).get().toPose2d())
            .getTranslation()
            .getNorm());
    // Angle calculatedAngle =
    // easyCrtSolver.getAngleOptional().orElse(Degrees.of(0.0));
    // SmartDashboard.putNumber("CRT Angle", calculatedAngle.in(Degrees));
    // SmartDashboard.putString("CRT Status", easyCrtSolver.getLastStatus().name());
    // SmartDashboard.putNumber("CRT Error Rot",
    // easyCrtSolver.getLastErrorRotations());

    Optional<Translation2d> targetPose = determineTarget();
    inputs.isValidCalculation = false;
    SmartDashboard.putNumber("Flywheel Multiplier", ShotCalculator.getFlywheelMultiplier());
    if (targetPose.isPresent()) {
      ShotCalculator.getInstance().clearShootingParameters();
      ShotCalculator.ShootingParameters params =
          ShotCalculator.getInstance()
              .getParameters(
                  robotToTurret,
                  Rotation2d.fromDegrees(turret.getAngle().in(Degrees)),
                  () -> Rebuilt.drivetrain.getPoseEstimator().getCurrentPose(),
                  () -> targetPose.get());
      if (params != null) {
        inputs.isValidCalculation = params.isValid();
        inputs.hoodAngleCalculated = Radian.of(params.hoodAngle());
        inputs.turretAngleCalculated = params.turretAngle().getMeasure();
        inputs.flyWheelSpeedCalculated =
            RPM.of(params.flywheelSpeed() * ShotCalculator.getFlywheelMultiplier());
        inputs.distanceToVirtualTarget = params.distanceToVirtualTarget();
        inputs.turretFeedforwardRadPerSec = params.solution().turretFeedforwardRadPerSec();
      }
      inputs.robotToTarget = LauncherCommands.getRobotToTarget(targetPose.get());

      inputs.targetDistance = Meters.of(inputs.robotToTarget.getDistance(new Translation2d()));
    }
    /** Reads the desired flywheel, hood, and turret setpoints */
    inputs.flyWheelSpeedDesired =
        flyWheel
            .getMotorController()
            .getMechanismSetpointVelocity()
            .map(it -> it)
            .orElse(RPM.of(0.0));
    inputs.hoodAngleDesired =
        hood.getMotorController().getMechanismPositionSetpoint().orElse(Degrees.of(0.0));
    // Read the turret desired angle from the profile controller's final goal rather than the YAMS
    // setpoint. YAMS setpoint is never updated since we bypass it and command the TalonFX directly.
    inputs.turretAngleDesired =
        turretProfileController != null
            ? Rotations.of(turretProfileController.getGoalPositionMechRot())
            : turret.getMotorController().getMechanismPositionSetpoint().orElse(Degrees.of(0.0));

    inputs.flyWheelSpeedActual = flyWheel.getSpeed();
    inputs.hoodAngleActual = hood.getAngle();
    inputs.turretAngleActual = turret.getAngle();

    inputs.flyWheelSpeedError = inputs.flyWheelSpeedActual.minus(inputs.flyWheelSpeedDesired);
    inputs.hoodAngleError = inputs.hoodAngleActual.minus(inputs.hoodAngleDesired).in(Degrees);
    inputs.turretAngleError = inputs.turretAngleActual.minus(inputs.turretAngleDesired).in(Degrees);

    inputs.flyWheelSpeedAtGoal =
        Math.abs(inputs.flyWheelSpeedError.in(RPM)) <= Constants.Launcher.SHOOTER_TOLERANCE_RPM;
    inputs.hoodAngleAtGoal =
        Math.abs(inputs.hoodAngleError) <= Constants.Launcher.HOOD_ANGLE_TOLERANCE_DEGREES;
    inputs.turretAngleAtGoal =
        Math.abs(inputs.turretAngleError) <= Constants.Launcher.TURRET_ANGLE_TOLERANCE_DEGREES;

    inputs.hoodVelocity = hood.getMotorController().getMechanismVelocity().in(Degrees.per(Second));
    inputs.turretVelocity =
        turret.getMotorController().getMechanismVelocity().in(Degrees.per(Second));
    inputs.flyWheelMotorOutput = flyWheel.getMotor().getStatorCurrent().in(Amps);
    isNearTrench();
  }

  /** Configuring the shot calculator with limits and constraints */
  @Override
  public void configureShotCalculator(ShotCalculator shotCalculator) {
    var turretConfig = turret.getMotorController().getConfig();

    shotCalculator.setShotTables(ShotCalculator.createDefaultTables());

    // Turret angular limits and aim tolerance — read directly from the YAMS config so they stay
    // in sync with the soft-limit values defined in launcher/turret.json.
    Rotation2d aimTolerance =
        Rotation2d.fromDegrees(
            turretConfig.getClosedLoopTolerance().orElse(Degrees.of(10.0)).in(Degrees));
    shotCalculator.setTurretConstraints(
        Rotation2d.fromDegrees(turretConfig.getMechanismLowerLimit().get().in(Degrees)),
        Rotation2d.fromDegrees(turretConfig.getMechanismUpperLimit().get().in(Degrees)),
        aimTolerance);

    // Trapezoidal motion profile constraints — read from the YAMS config (populated from
    // launcher/turret.json motorSystemId.maxVelocity / maxAcceleration).
    // YAMS stores these in rot/s and rot/s², so multiply by 2π to get rad/s and rad/s².
    var trapConstraints = turretConfig.getTrapezoidProfile();
    double maxVelRadPerSec =
        trapConstraints.map(c -> c.maxVelocity * 2.0 * Math.PI).orElse(Math.toRadians(1080.0));
    double maxAccelRadPerSecSq =
        trapConstraints.map(c -> c.maxAcceleration * 2.0 * Math.PI).orElse(Math.toRadians(360.0));
    shotCalculator.setTurretMotionConstraints(maxVelRadPerSec, maxAccelRadPerSecSq, 0.85);

    // Provide the turret profile controller's velocity to the shot calculator for
    // velocity-aware settling time estimation.
    if (turretProfileController != null) {
      shotCalculator.setTurretVelocitySupplier(
          turretProfileController::getCurrentVelocityRadPerSec);
    }
  }

  @Override
  public TurretProfileController getTurretProfileController() {
    return turretProfileController;
  }

  /** Sets the flywheel motor's duty cycle */
  public void runShooter(double speed) {
    flyWheel.getMotor().setDutyCycle(speed);
  }

  /** Sets the flywheel motor's angular velocity */
  public void setFlyWheelVelocity(AngularVelocity speed) {
    flyWheel.getMotor().setVelocity(speed);
  }

  /** Sets the hood angle and overrides the requested angle if the hood is near the trench */
  public void setHoodAngle(Angle angle) {
    hood.getMotorController().setPosition(angle);
  }

  /** Sets the low hard limit to 30 degrees and updates LED's */
  public void setHoodAngleLow() {
    hood.getMotorController()
        .setPosition(hood.getArmConfig().getLowerHardLimit().orElse(Degrees.of(30)));
    LEDStrip.changeSegmentPattern(ConfigConstants.ALL_LEDS, LEDStrip.getSolidPattern(Color.kGreen));
  }

  /** Sets the angle of the turret via the RoboRIO-side profile controller (zero feedforward). */
  public void setTurretRotation(Angle angle) {
    if (angle.gt(turretHighLimit)) {
      SmartDashboard.putBoolean("Launcher/Turret Limit", true);
      angle = turretHighLimit;
    } else if (angle.lt(turretLowLimit)) {
      SmartDashboard.putBoolean("Launcher/Turret Limit", true);
      angle = turretLowLimit;
    } else {
      SmartDashboard.putBoolean("Launcher/Turret Limit", false);
    }
    if (turretProfileController != null) {
      turretProfileController.setGoal(angle, 0.0);
    } else {
      turret.getMotorController().setPosition(angle);
    }
  }

  /**
   * Sets the turret angle with an additional velocity feedforward integrated into the RoboRIO-side
   * trapezoidal profile. The feedforward becomes the profile's goal velocity so that the profile
   * naturally includes tracking velocity rather than fighting a zero-velocity-targeting controller.
   *
   * @param angle desired turret mechanism angle
   * @param feedforwardRadPerSec angular velocity feedforward in rad/s (mechanism units)
   */
  @Override
  public void setTurretRotationWithFeedforward(Angle angle, double feedforwardRadPerSec) {
    if (angle.gt(turretHighLimit)) {
      SmartDashboard.putBoolean("Launcher/Turret Limit", true);
      angle = turretHighLimit;
    } else if (angle.lt(turretLowLimit)) {
      SmartDashboard.putBoolean("Launcher/Turret Limit", true);
      angle = turretLowLimit;
    } else {
      SmartDashboard.putBoolean("Launcher/Turret Limit", false);
    }

    if (turretProfileController != null) {
      turretProfileController.setGoal(angle, feedforwardRadPerSec);
    } else {
      // Fallback: YAMS setPosition without feedforward
      turret.getMotorController().setPosition(angle);
    }
  }

  /** Converts the flywheel angular velocity into speed */
  public LinearVelocity getFlyWheelExitSpeed(AngularVelocity velocity) {
    return MetersPerSecond.of(
        flyWheel.getShooterConfig().getCircumference().in(Meters)
            * Math.PI // This is a total fudge on the math, but it gives us a more realistic exit
            // velocity for the flywheel speeds we are commanding
            * (velocity.in(RadiansPerSecond)));
  }

  /** Returns SysId command for the hood */
  public Command getHoodSysIdCommand() {
    return hood.sysId(Volts.of(4), Volts.of(0.5).per(Seconds), Seconds.of(8));
  }

  /** Runs sysid for the cahracterized hood motor and stops at limits */
  public Command getHoodSysIdCommand(GenericSubsystem launcher) {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.angleSysIdRoutine(hood.getMotorController(), hood.getName(), launcher),
        5,
        3,
        3,
        () ->
            hood.isNear(
                    hood.getMotorController().getConfig().getMechanismUpperLimit().get(),
                    Degrees.of(10))
                .getAsBoolean(),
        () ->
            hood.isNear(
                    hood.getMotorController().getConfig().getMechanismLowerLimit().get(),
                    Degrees.of(10))
                .getAsBoolean(),
        () -> hood.getMotor().setDutyCycle(0));
  }

  public Command getTurretSysIdCommand() {
    return turret.sysId(Volts.of(4), Volts.of(0.5).per(Seconds), Seconds.of(8));
  }

  /** Characterizes the turret */
  public Command getTurretSysIdCommand(GenericSubsystem launcher) {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.angleSysIdRoutine(
            turret.getMotorController(), turret.getName(), launcher),
        5,
        3.5,
        3,
        () ->
            turret
                .isNear(
                    turret.getMotorController().getConfig().getMechanismUpperLimit().get(),
                    Degrees.of(10))
                .getAsBoolean(),
        () ->
            turret
                .isNear(
                    turret.getMotorController().getConfig().getMechanismLowerLimit().get(),
                    Degrees.of(10))
                .getAsBoolean(),
        () -> turret.getMotor().setDutyCycle(0));
  }

  /** Applies voltage and measures hood velocity to characterize the feed forward */
  public Command getHoodCharacterizationCommand(GenericSubsystem launcher) {
    return SystemIdentification.feedforwardCharacterization(
        launcher,
        (Voltage voltage) -> hood.getMotor().setVoltage(voltage),
        () -> hood.getMotorController().getMechanismVelocity().in(Degrees.per(Second)));
  }

  /** Applies voltage and measures turret velocity to characterize the feedfoward */
  public Command getTurretCharacterizationCommand(GenericSubsystem launcher) {
    return SystemIdentification.feedforwardCharacterization(
        launcher,
        (Voltage voltage) -> turret.getMotor().setVoltage(voltage),
        () -> turret.getMotorController().getMechanismVelocity().in(Degrees.per(Second)));
  }

  /** sets the flywheel, hood, and turret motor duty cycles to 0, which stops the motors */
  public void stopAllMotors() {
    flyWheel.getMotor().setDutyCycle(0);
    hood.getMotor().setDutyCycle(0);
    turret.getMotor().setDutyCycle(0);
  }

  public Command getFlyWheelSysIdCommand() {
    return flyWheel.sysId(Volts.of(8), Volts.of(0.5).per(Seconds), Seconds.of(8));
  }

  public boolean isNearTrench() {
    Pose2d current = drivetrain.getPoseEstimator().getCurrentPose();
    double currentX = current.getX();
    double currentY = current.getY();

    return FieldRegions.isNearTrench(currentX, currentY);
  }

  public Optional<Translation2d> determineTarget() {
    Pose2d current = drivetrain.getPoseEstimator().getCurrentPose();
    return FieldRegions.determineTargetPose(current);
  }

  public Command getFlyWheelSysIdCommand(GenericSubsystem launcher) {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.rpmSysIdRoutine(
            flyWheel.getMotorController(), flyWheel.getName(), launcher),
        8,
        3,
        3);
  }
}
