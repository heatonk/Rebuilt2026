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
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.commands.IntakeCommands.IntakeState;
import frc.robot.rebuilt.commands.LauncherCommands;
import frc.robot.rebuilt.subsystems.intake.Intake;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
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
public class LauncherIOReal implements LauncherIO { // -0.030679615757712823
  protected static final Angle HARD_STOP = Radians.of(2.9437091319525455);
  protected static final double encoder40Offset = 0.4423828125;
  protected static final double encoder36Offset = -0.095947265625;
  protected Map<String, Object> devices;
  protected Pivot turret;
  protected Arm hood;
  private TalonFX hoodTalonFX;
  private final MotionMagicTorqueCurrentFOC hoodMotionMagicRequest =
      new MotionMagicTorqueCurrentFOC(0).withSlot(0);
  private Angle hoodAngleSetpoint = Degrees.of(0.0);
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
  private DigitalInput turretZeroButton;
  private Trigger turretZeroTrigger;

  Angle turretLowLimit = Degrees.of(-90);
  Angle turretHighLimit = Degrees.of(90);

  /** 2-state turret controller: SEEKING (MotionMagic) and TRACKING (Position + FF). */
  protected SmartTurretController smartTurretController;

  /** Previous turret desired angle (rad) for numerical feedforward differentiation. */
  private double previousTurretDesiredAngleRad = 0.0;

  /** Previous turret velocity feedforward (rad/s) for numerical acceleration computation. */
  private double previousTurretVelocityRadPerSec = 0.0;

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

    turretZeroButton = new DigitalInput(0);

    hood = (Arm) devices.get("hood");
    hoodAngleSetpoint = hood.getAngle();
    Object rawHoodController = hood.getMotorController().getMotorController();
    if (!RobotBase.isSimulation() && rawHoodController instanceof TalonFX talonFX) {
      hoodTalonFX = talonFX;
    }
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
                Rotations.of(encoder40Offset),
                Rotations.of(encoder36Offset)) // set after mechanical zero
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

    // Create the 2-state SmartTurretController (replaces TurretProfileController).
    // Uses MotionMagicTorqueCurrentFOC for seeking and PositionTorqueCurrentFOC for tracking.
    {
      var turretConfig = turret.getMotorController().getConfig();
      var trapConstraints = turretConfig.getTrapezoidProfile();
      // Mechanism rot/s and rot/s^2; fallback values from turret.json maxVelocity/maxAcceleration.
      double maxVelMechRotPerSec = trapConstraints.map(c -> c.maxVelocity).orElse(1080.0 / 360.0);
      double maxAccelMechRotPerSecSq =
          trapConstraints.map(c -> c.maxAcceleration).orElse(5080.0 / 360.0);
      double lowerLimitRot =
          turretConfig.getMechanismLowerLimit().orElse(Degrees.of(-150)).in(Rotations);
      double upperLimitRot =
          turretConfig.getMechanismUpperLimit().orElse(Degrees.of(150)).in(Rotations);

      Object rawController = turret.getMotorController().getMotorController();
      if (rawController instanceof com.ctre.phoenix6.hardware.TalonFX talonFXRaw) {
        // Load feedforward from YAMS mechanism config (populated from turret.json motorSystemId).
        // Turret is a Pivot so getArmFeedforward() contains the characterised kS/kV/kA in SI units.
        // Fallback values match turret.json in case the YAMS FF was not set.
        ArmFeedforward yamsFf = turretConfig.getArmFeedforward().orElse(null);
        double kS = yamsFf != null ? yamsFf.getKs() : 12;
        double kV = yamsFf != null ? yamsFf.getKv() : 0.0;
        double kA = yamsFf != null ? yamsFf.getKa() : 2.0;
        SmartTurretConfig smartConfig =
            new SmartTurretConfig.Builder()
                .withTalonFX(talonFXRaw)
                .withYAMSController(turret.getMotorController())
                .withGearRatio(30.0)
                .withMotionConstraints(maxVelMechRotPerSec, maxAccelMechRotPerSecSq)
                .withSeekingPID(650, 0, 50) // Initial values from turret.json
                .withTrackingPID(650, 0, 50) // Start same, tune separately
                .withFeedforward(kS, kV, kA)
                .withSeekingThreshold(Degrees.of(100).in(Rotations))
                .withHysteresisBuffer(Degrees.of(12).in(Rotations))
                .withSoftLimits(lowerLimitRot, upperLimitRot)
                .build();

        smartTurretController = new SmartTurretController(smartConfig);
        // Reset controller to the CRT-solved initial position.
        smartTurretController.reset(calculatedAngle.in(Rotations), 0);
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
    org.littletonrobotics.junction.Logger.recordOutput(
        "Turret Zero Button", turretZeroButton.get());
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
        inputs.turretFeedforwardAccelRadPerSecSq =
            (inputs.turretFeedforwardRadPerSec - previousTurretVelocityRadPerSec)
                / org.frc5010.common.constants.Constants.loopPeriodSecs;
        previousTurretVelocityRadPerSec = inputs.turretFeedforwardRadPerSec;
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
        hoodTalonFX != null
            ? hoodAngleSetpoint
            : hood.getMotorController().getMechanismPositionSetpoint().orElse(Degrees.of(0.0));
    // Read turret desired angle from the SmartTurretController's goal, not YAMS (which is
    // bypassed).
    inputs.turretAngleDesired =
        smartTurretController != null
            ? Rotations.of(smartTurretController.getGoalPositionMechRot())
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

    // Provide the turret motor's actual velocity to the shot calculator for
    // velocity-aware settling time estimation. Uses encoder velocity (not profile velocity)
    // to avoid a feedback loop between the solver and the controller.
    if (smartTurretController != null) {
      shotCalculator.setTurretVelocitySupplier(smartTurretController::getActualVelocityRadPerSec);
    }
  }

  @Override
  public SmartTurretController getSmartTurretController() {
    return smartTurretController;
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
    requestHoodAngle(angle);
  }

  /** Sets the low hard limit to 30 degrees and updates LED's */
  public void setHoodAngleLow() {
    requestHoodAngle(hood.getArmConfig().getLowerHardLimit().orElse(Degrees.of(30)));
    LEDStrip.changeSegmentPattern(ConfigConstants.ALL_LEDS, LEDStrip.getSolidPattern(Color.kGreen));
  }

  private void requestHoodAngle(Angle angle) {
    hoodAngleSetpoint = angle;
    if (hoodTalonFX != null) {
      hoodTalonFX.setControl(hoodMotionMagicRequest.withPosition(angle.in(Rotations)));
      return;
    }
    hood.getMotorController().setPosition(angle);
  }

  /** Sets the angle of the turret via the SmartTurretController (zero feedforward). */
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
    if (smartTurretController != null) {
      smartTurretController.setTarget(angle, 0.0, 0.0);
    } else {
      turret.getMotorController().setPosition(angle);
    }
  }

  /**
   * Sets the turret angle with velocity and acceleration feedforward for the SmartTurretController.
   *
   * @param angle desired turret mechanism angle
   * @param feedforwardRadPerSec angular velocity feedforward in rad/s (mechanism units)
   * @param accelerationRadPerSecSq angular acceleration feedforward in rad/s^2 (mechanism units)
   */
  @Override
  public void setTurretRotationWithFeedforward(
      Angle angle, double feedforwardRadPerSec, double accelerationRadPerSecSq) {
    if (angle.gt(turretHighLimit)) {
      SmartDashboard.putBoolean("Launcher/Turret Limit", true);
      angle = turretHighLimit;
    } else if (angle.lt(turretLowLimit)) {
      SmartDashboard.putBoolean("Launcher/Turret Limit", true);
      angle = turretLowLimit;
    } else {
      SmartDashboard.putBoolean("Launcher/Turret Limit", false);
    }

    if (smartTurretController != null) {
      smartTurretController.setTarget(angle, feedforwardRadPerSec, accelerationRadPerSecSq);
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

  @Override
  public Command getTurretQuasistaticCommand(GenericSubsystem launcher) {
    if (smartTurretController == null) return Commands.none();
    return new frc.robot.rebuilt.commands.TurretQuasistaticCommand(smartTurretController, launcher);
  }

  @Override
  public Command getTurretDynamicCommand(GenericSubsystem launcher) {
    if (smartTurretController == null) return Commands.none();
    return new frc.robot.rebuilt.commands.TurretDynamicCommand(smartTurretController, launcher);
  }

  @Override
  public Command getTurretKsMapCommand(GenericSubsystem launcher) {
    if (smartTurretController == null) return Commands.none();
    return new frc.robot.rebuilt.commands.TurretKsMapCommand(
        smartTurretController, turretLowLimit, turretHighLimit, launcher);
  }

  @Override
  public Command getTurretTrackingTuneCommand(GenericSubsystem launcher) {
    if (smartTurretController == null) return Commands.none();
    return new frc.robot.rebuilt.commands.TurretTrackingTuneCommand(
        smartTurretController, launcher);
  }

  @Override
  public Command getTurretSeekingTuneCommand(GenericSubsystem launcher) {
    if (smartTurretController == null) return Commands.none();
    return new frc.robot.rebuilt.commands.TurretSeekingTuneCommand(smartTurretController, launcher);
  }

  @Override
  public void zeroTurret() {
    turret.getMotor().setEncoderPosition(HARD_STOP);
  }

  @Override
  public boolean isTurretAtZero() {
    return Math.abs(turret.getAngle().in(Degrees) - HARD_STOP.in(Degrees)) < 2.0;
  }

  @Override
  public BooleanSupplier getTurretZeroButtonSupplier() {
    return turretZeroButton::get;
  }
}
