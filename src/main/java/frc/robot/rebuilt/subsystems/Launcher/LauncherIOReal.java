// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.FieldConstants;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.subsystems.drive.RebuiltDrivetrain;
import frc.robot.rebuilt.subsystems.intake.Intake;
import frc.robot.rebuilt.util.AllianceFlipUtil;
import frc.robot.rebuilt.util.LedStrip;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Real-hardware IO layer for the Launcher subsystem, built directly on Phoenix 6. */
public class LauncherIOReal implements LauncherIO {
  protected static final Angle HARD_STOP = edu.wpi.first.units.Units.Radians.of(2.9145634969827183);
  private static final double MIN_DYNAMIC_TURRET_TOLERANCE_DEGREES = 2.0;
  private static final double MIN_DYNAMIC_TURRET_SHUTTLE_TOLERANCE_DEGREES = 4.0;
  private static final double MAX_DYNAMIC_TURRET_SHUTTLE_TOLERANCE_DEGREES = 20.0;

  private static final double TURRET_GEAR_RATIO = 30.0;
  private static final double HOOD_GEAR_RATIO = 1015.0 / 33.0;
  private static final double FLYWHEEL_GEAR_RATIO = 18.0;

  protected TalonFX turret;
  protected TalonFX hoodTalonFX;
  protected TalonFX flywheelLeader;
  protected TalonFX flywheelFollower;

  private final MotionMagicTorqueCurrentFOC hoodMotionMagicRequest =
      new MotionMagicTorqueCurrentFOC(0).withSlot(0);
  private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0).withSlot(0);
  private final DutyCycleOut flywheelDutyCycle = new DutyCycleOut(0);
  private final DutyCycleOut hoodDutyCycle = new DutyCycleOut(0);
  private final DutyCycleOut turretDutyCycle = new DutyCycleOut(0);

  private Angle hoodAngleSetpoint = Degrees.of(0.0);

  protected RebuiltDrivetrain drivetrain;

  private enum TargetProfile {
    NONE,
    HUB,
    SHUTTLE
  }

  protected Intake intake;

  protected static Translation2d robotToTurret;
  private DigitalInput turretZeroButton;

  private final Angle turretLowLimit = Degrees.of(Constants.Launcher.Turret.LOWER_SOFT_LIMIT_DEG);
  private final Angle turretHighLimit = Degrees.of(Constants.Launcher.Turret.UPPER_SOFT_LIMIT_DEG);

  private Debouncer hoodNotMoving;

  /** 2-state turret controller: SEEKING (MotionMagic) and TRACKING (Position + FF). */
  protected SmartTurretController smartTurretController;

  /** Previous turret velocity feedforward (rad/s) for numerical acceleration computation. */
  private double previousTurretVelocityRadPerSec = 0.0;

  public LauncherIOReal(SubsystemBase parent) {
    drivetrain = Rebuilt.drivetrain;
    intake = Rebuilt.intake;

    turret = buildTurret();
    hoodTalonFX = buildHood();
    flywheelLeader = buildFlywheelLeader();
    flywheelFollower = buildFlywheelFollower(flywheelLeader);

    robotToTurret =
        new Translation2d(
            Inches.of(Constants.Launcher.Turret.ROBOT_TO_MOTOR_X_IN).in(Meters),
            Inches.of(Constants.Launcher.Turret.ROBOT_TO_MOTOR_Y_IN).in(Meters));

    hoodNotMoving = new Debouncer(0.25, Debouncer.DebounceType.kRising);
    turretZeroButton = new DigitalInput(0);
    hoodAngleSetpoint = getHoodAngle();

    // Create the 2-state SmartTurretController. It performs its own read-modify-write of
    // the TalonFX config to install Slot0/Slot1 gains and MotionMagicExpo parameters.
    smartTurretController =
        new SmartTurretController(
            new SmartTurretConfig.Builder()
                .withTalonFX(turret)
                .withGearRatio(TURRET_GEAR_RATIO)
                .withMotionConstraints(
                    Constants.Launcher.Turret.MAX_VEL_DEG_PER_SEC / 360.0,
                    Constants.Launcher.Turret.MAX_ACCEL_DEG_PER_SEC_SQ / 360.0)
                .withSeekingPID(
                    Constants.Launcher.Turret.SMART_SEEKING_KP,
                    Constants.Launcher.Turret.SMART_SEEKING_KI,
                    Constants.Launcher.Turret.SMART_SEEKING_KD)
                .withTrackingPID(
                    Constants.Launcher.Turret.SMART_TRACKING_KP,
                    Constants.Launcher.Turret.SMART_TRACKING_KI,
                    Constants.Launcher.Turret.SMART_TRACKING_KD)
                .withFeedforward(
                    Constants.Launcher.Turret.SMART_FALLBACK_KS,
                    Constants.Launcher.Turret.SMART_FALLBACK_KV,
                    Constants.Launcher.Turret.SMART_FALLBACK_KA)
                .withSeekingThreshold(Degrees.of(5).in(Rotations))
                .withHysteresisBuffer(Degrees.of(12).in(Rotations))
                .withSoftLimits(
                    Degrees.of(Constants.Launcher.Turret.LOWER_SOFT_LIMIT_DEG).in(Rotations),
                    Degrees.of(Constants.Launcher.Turret.UPPER_SOFT_LIMIT_DEG).in(Rotations))
                .build());
    smartTurretController.reset(getTurretAngle().in(Rotations), 0);
  }

  private static TalonFX buildTurret() {
    TalonFX motor =
        new TalonFX(
            Constants.Launcher.Turret.CAN_ID, new CANBus(Constants.Launcher.Turret.CAN_BUS));

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        Constants.Launcher.Turret.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = TURRET_GEAR_RATIO;

    // Soft limits are enforced by the SmartTurretController's own configure step as well,
    // but we set defaults here for safety before that controller is constructed.
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Degrees.of(Constants.Launcher.Turret.UPPER_SOFT_LIMIT_DEG).in(Rotations);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Degrees.of(Constants.Launcher.Turret.LOWER_SOFT_LIMIT_DEG).in(Rotations);

    motor.getConfigurator().apply(config);
    motor.setPosition(Degrees.of(Constants.Launcher.Turret.STARTING_ANGLE_DEG).in(Rotations));
    return motor;
  }

  private static TalonFX buildHood() {
    TalonFX motor = new TalonFX(Constants.Launcher.Hood.CAN_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        Constants.Launcher.Hood.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = HOOD_GEAR_RATIO;

    Slot0Configs slot0 = config.Slot0;
    slot0.kP = Constants.Launcher.Hood.KP;
    slot0.kI = Constants.Launcher.Hood.KI;
    slot0.kD = Constants.Launcher.Hood.KD;
    slot0.kS = Constants.Launcher.Hood.KS;
    slot0.kV = Constants.Launcher.Hood.KV;
    slot0.kA = Constants.Launcher.Hood.KA;
    slot0.kG = Constants.Launcher.Hood.KG;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // MotionMagic in mechanism rotations/sec.
    config.MotionMagic.MotionMagicCruiseVelocity =
        Constants.Launcher.Hood.MAX_VEL_DEG_PER_SEC / 360.0;
    config.MotionMagic.MotionMagicAcceleration =
        Constants.Launcher.Hood.MAX_ACCEL_DEG_PER_SEC_SQ / 360.0;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Degrees.of(Constants.Launcher.Hood.UPPER_SOFT_LIMIT_DEG).in(Rotations);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Degrees.of(Constants.Launcher.Hood.LOWER_SOFT_LIMIT_DEG).in(Rotations);

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Launcher.Hood.CURRENT_LIMIT_AMPS;

    motor.getConfigurator().apply(config);
    motor.setPosition(Degrees.of(Constants.Launcher.Hood.STARTING_ANGLE_DEG).in(Rotations));
    return motor;
  }

  private static TalonFX buildFlywheelLeader() {
    TalonFX motor = new TalonFX(Constants.Launcher.FlyWheel.CAN_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        Constants.Launcher.FlyWheel.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = FLYWHEEL_GEAR_RATIO;

    Slot0Configs slot0 = config.Slot0;
    slot0.kP = Constants.Launcher.FlyWheel.KP;
    slot0.kI = Constants.Launcher.FlyWheel.KI;
    slot0.kD = Constants.Launcher.FlyWheel.KD;
    slot0.kS = Constants.Launcher.FlyWheel.KS;
    slot0.kV = Constants.Launcher.FlyWheel.KV;
    slot0.kA = Constants.Launcher.FlyWheel.KA;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Launcher.FlyWheel.CURRENT_LIMIT_AMPS;

    motor.getConfigurator().apply(config);
    return motor;
  }

  private static TalonFX buildFlywheelFollower(TalonFX leader) {
    TalonFX motor = new TalonFX(Constants.Launcher.FlyWheel.FOLLOWER_CAN_ID);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Launcher.FlyWheel.CURRENT_LIMIT_AMPS;
    motor.getConfigurator().apply(config);
    motor.setControl(
        new Follower(
            leader.getDeviceID(),
            Constants.Launcher.FlyWheel.FOLLOWER_INVERTED
                ? MotorAlignmentValue.Opposed
                : MotorAlignmentValue.Aligned));
    return motor;
  }

  // ---- Helper accessors used by the sim layer -------------------------------

  Angle getTurretAngle() {
    return Rotations.of(turret.getPosition().getValueAsDouble());
  }

  Angle getHoodAngle() {
    return Rotations.of(hoodTalonFX.getPosition().getValueAsDouble());
  }

  AngularVelocity getFlywheelSpeed() {
    return RotationsPerSecond.of(flywheelLeader.getVelocity().getValueAsDouble());
  }

  // ---- LauncherIO interface implementation ---------------------------------

  public ShotCalculator.ShootingParameters getShootingParameters(
      Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d> targetPositionSupplier) {
    Translation2d targetPosition = targetPositionSupplier.get();
    ShotCalculator.getInstance().useShotProfile(getShotProfile(targetPosition));
    ShotCalculator.getInstance().clearShootingParameters();
    return ShotCalculator.getInstance()
        .getParameters(
            robotToTurret,
            Rotation2d.fromDegrees(getTurretAngle().in(Degrees)),
            robotPoseSupplier,
            () -> targetPosition);
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    Logger.recordOutput("Turret Zero Button", turretZeroButton.get());
    SmartDashboard.putNumber(
        "Distance to tag 27",
        drivetrain
            .getPoseEstimator()
            .getCurrentPose3d()
            .toPose2d()
            .minus(FieldConstants.aprilTagFieldLayout.getTagPose(21).get().toPose2d())
            .getTranslation()
            .getNorm());

    Pose2d currentPose = drivetrain.getPoseEstimator().getCurrentPose();
    Optional<Translation2d> targetPose = FieldRegions.determineTargetPose(currentPose);
    TargetProfile targetProfile = TargetProfile.NONE;
    inputs.isValidCalculation = false;
    SmartDashboard.putNumber("Flywheel Multiplier", ShotCalculator.getFlywheelMultiplier());

    Translation2d SOTMOffset = new Translation2d();
    edu.wpi.first.units.measure.Distance distanceToVirtualTarget = Meters.of(0.0001);

    double hoodVelocityDegPerSec = hoodTalonFX.getVelocity().getValueAsDouble() * 360.0;
    inputs.hoodMoving = !hoodNotMoving.calculate(hoodVelocityDegPerSec < 1.0);

    if (targetPose.isPresent()) {
      targetProfile = getTargetProfile(targetPose.get());
      ShotCalculator.getInstance().useShotProfile(getShotProfile(targetPose.get()));
      ShotCalculator.getInstance().clearShootingParameters();
      ShotCalculator.ShootingParameters params =
          ShotCalculator.getInstance()
              .getParameters(
                  robotToTurret,
                  Rotation2d.fromDegrees(getTurretAngle().in(Degrees)),
                  () -> currentPose,
                  () -> targetPose.get());
      if (params != null) {
        inputs.isValidCalculation = params.isValid();
        inputs.hoodAngleCalculated = edu.wpi.first.units.Units.Radian.of(params.hoodAngle());
        inputs.turretAngleCalculated = params.turretAngle().getMeasure();
        inputs.flyWheelSpeedCalculated =
            RPM.of(params.flywheelSpeed() * ShotCalculator.getFlywheelMultiplier());
        inputs.distanceToVirtualTarget = params.distanceToVirtualTarget();
        inputs.turretFeedforwardRadPerSec = params.solution().turretFeedforwardRadPerSec();
        inputs.turretFeedforwardAccelRadPerSecSq =
            (inputs.turretFeedforwardRadPerSec - previousTurretVelocityRadPerSec) / 0.02;
        previousTurretVelocityRadPerSec = inputs.turretFeedforwardRadPerSec;

        edu.wpi.first.math.kinematics.ChassisSpeeds virtualTargetOffsetparams =
            params
                .solution()
                .finalSolverState()
                .robotStateAtFire()
                .velocity()
                .times(-params.solution().estimatedTimeOfFlight());
        SOTMOffset =
            new Translation2d(
                virtualTargetOffsetparams.vxMetersPerSecond,
                virtualTargetOffsetparams.vyMetersPerSecond);
        distanceToVirtualTarget = params.distanceToVirtualTarget();
      }
      inputs.robotToTarget =
          AllianceFlipUtil.apply(targetPose.get()).minus(currentPose.getTranslation());

      inputs.targetDistance = Meters.of(inputs.robotToTarget.getDistance(new Translation2d()));
    } else {
      ShotCalculator.getInstance().useShotProfile(ShotCalculator.ShotProfile.NORMAL);
    }

    inputs.flyWheelSpeedDesired = RotationsPerSecond.of(flywheelVelocityRequest.Velocity);
    inputs.hoodAngleDesired = hoodAngleSetpoint;
    inputs.turretAngleDesired =
        smartTurretController != null
            ? Rotations.of(smartTurretController.getGoalPositionMechRot())
            : getTurretAngle();

    double[] turretAngleToleranceDegrees =
        getTurretAngleToleranceDegrees(
            currentPose,
            inputs.turretAngleDesired,
            targetPose.orElse(null),
            SOTMOffset,
            distanceToVirtualTarget,
            targetProfile);
    SmartDashboard.putString("Launcher/Target Profile", targetProfile.name());
    Logger.recordOutput("Launcher/Lower Turret Tolerance Deg", turretAngleToleranceDegrees[0]);
    Logger.recordOutput("Launcher/Upper Turret Tolerance Deg", turretAngleToleranceDegrees[1]);

    inputs.flyWheelSpeedActual = getFlywheelSpeed();
    inputs.hoodAngleActual = getHoodAngle();
    inputs.turretAngleActual = getTurretAngle();

    inputs.flyWheelSpeedError = inputs.flyWheelSpeedActual.minus(inputs.flyWheelSpeedDesired);
    inputs.hoodAngleError = inputs.hoodAngleActual.minus(inputs.hoodAngleDesired).in(Degrees);
    inputs.turretAngleError = inputs.turretAngleActual.minus(inputs.turretAngleDesired).in(Degrees);

    inputs.flyWheelSpeedAtGoal =
        Math.abs(inputs.flyWheelSpeedError.in(RPM)) <= Constants.Launcher.SHOOTER_TOLERANCE_RPM;
    inputs.hoodAngleAtGoal =
        Math.abs(inputs.hoodAngleError) <= Constants.Launcher.HOOD_ANGLE_TOLERANCE_DEGREES;
    inputs.turretAngleAtGoal =
        turretAngleToleranceDegrees[0] <= inputs.turretAngleError
            && inputs.turretAngleError <= turretAngleToleranceDegrees[1];

    inputs.hoodVelocity = hoodVelocityDegPerSec;
    inputs.turretVelocity = turret.getVelocity().getValueAsDouble() * 360.0;
    inputs.flyWheelMotorOutput = flywheelLeader.getStatorCurrent().getValueAsDouble();
    isNearTrench();
  }

  @Override
  public void configureShotCalculator(ShotCalculator shotCalculator) {
    ShotCalculator.ShotTables defaultTables = ShotCalculator.createDefaultTables();
    shotCalculator.setShotTables(defaultTables);
    shotCalculator.setShuttleShotTables(ShotCalculator.copyShotTables(defaultTables));

    Rotation2d aimTolerance =
        Rotation2d.fromDegrees(Constants.Launcher.TURRET_ANGLE_TOLERANCE_DEGREES);
    shotCalculator.setTurretConstraints(
        Rotation2d.fromDegrees(Constants.Launcher.Turret.LOWER_SOFT_LIMIT_DEG),
        Rotation2d.fromDegrees(Constants.Launcher.Turret.UPPER_SOFT_LIMIT_DEG),
        aimTolerance);

    double maxVelRadPerSec = Math.toRadians(Constants.Launcher.Turret.MAX_VEL_DEG_PER_SEC);
    double maxAccelRadPerSecSq = Math.toRadians(Constants.Launcher.Turret.MAX_ACCEL_DEG_PER_SEC_SQ);
    shotCalculator.setTurretMotionConstraints(maxVelRadPerSec, maxAccelRadPerSecSq, 0.85);

    if (smartTurretController != null) {
      shotCalculator.setTurretVelocitySupplier(smartTurretController::getActualVelocityRadPerSec);
    }
  }

  @Override
  public SmartTurretController getSmartTurretController() {
    return smartTurretController;
  }

  public void resetHoodAngle(Angle angle) {
    hoodTalonFX.setPosition(angle.in(Rotations));
  }

  public void runShooter(double speed) {
    flywheelLeader.setControl(flywheelDutyCycle.withOutput(speed));
  }

  public void setFlyWheelVelocity(AngularVelocity speed) {
    flywheelLeader.setControl(flywheelVelocityRequest.withVelocity(speed.in(RotationsPerSecond)));
  }

  public void setHoodAngle(Angle angle) {
    requestHoodAngle(angle);
  }

  public void setHoodAngleLow() {
    requestHoodAngle(Degrees.of(Constants.Launcher.Hood.LOWER_HARD_LIMIT_DEG));
    LedStrip.changeSegmentPattern(LedStrip.ALL_LEDS, LedStrip.getSolidPattern(Color.kGreen));
  }

  public void runHoodDown() {
    hoodTalonFX.setControl(hoodDutyCycle.withOutput(-1.0));
  }

  public void stopHood() {
    hoodTalonFX.setControl(hoodDutyCycle.withOutput(0.0));
  }

  public Boolean isHoodStalled() {
    return hoodTalonFX.getStatorCurrent().getValueAsDouble()
        > Constants.Launcher.HOOD_STALL_CURRENT_THRESHOLD;
  }

  private void requestHoodAngle(Angle angle) {
    hoodAngleSetpoint = angle;
    hoodTalonFX.setControl(hoodMotionMagicRequest.withPosition(angle.in(Rotations)));
  }

  /** Sets the turret angle via the SmartTurretController (zero feedforward). */
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
    smartTurretController.setTarget(angle, 0.0, 0.0);
  }

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
    smartTurretController.setTarget(angle, feedforwardRadPerSec, accelerationRadPerSecSq);
  }

  /** Converts the flywheel angular velocity to an exit linear speed at the wheel rim. */
  public LinearVelocity getFlyWheelExitSpeed(AngularVelocity velocity) {
    double circumferenceMeters =
        2.0 * Math.PI * Inches.of(Constants.Launcher.FlyWheel.RADIUS_INCHES).in(Meters);
    return MetersPerSecond.of(
        circumferenceMeters * Math.PI * velocity.in(RadiansPerSecond));
  }

  // ---- Characterization / SysId commands are provided by Phoenix Tuner X post-migration ----

  public Command getHoodSysIdCommand() {
    return Commands.none();
  }

  public Command getHoodSysIdCommand(SubsystemBase launcher) {
    return Commands.none();
  }

  public Command getTurretSysIdCommand() {
    return Commands.none();
  }

  public Command getTurretSysIdCommand(SubsystemBase launcher) {
    return Commands.none();
  }

  public Command getHoodCharacterizationCommand(SubsystemBase launcher) {
    return Commands.none();
  }

  public Command getTurretCharacterizationCommand(SubsystemBase launcher) {
    return Commands.none();
  }

  public Command getFlyWheelSysIdCommand() {
    return Commands.none();
  }

  public Command getFlyWheelSysIdCommand(SubsystemBase launcher) {
    return Commands.none();
  }

  public void stopAllMotors() {
    flywheelLeader.setControl(flywheelDutyCycle.withOutput(0));
    hoodTalonFX.setControl(hoodDutyCycle.withOutput(0));
    turret.setControl(turretDutyCycle.withOutput(0));
  }

  public boolean isNearTrench() {
    Pose2d current = drivetrain.getPoseEstimator().getCurrentPose();
    return FieldRegions.isNearTrench(current.getX(), current.getY());
  }

  public Optional<Translation2d> determineTarget() {
    Pose2d current = drivetrain.getPoseEstimator().getCurrentPose();
    return FieldRegions.determineTargetPose(current);
  }

  private TargetProfile getTargetProfile(Translation2d targetPose) {
    Translation2d fieldTarget = AllianceFlipUtil.apply(targetPose);
    Translation2d hubTarget =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    if (fieldTarget.getDistance(hubTarget) < 1e-6) {
      return TargetProfile.HUB;
    }
    return TargetProfile.SHUTTLE;
  }

  private ShotCalculator.ShotProfile getShotProfile(Translation2d targetPose) {
    return getTargetProfile(targetPose) == TargetProfile.SHUTTLE
        ? ShotCalculator.ShotProfile.SHUTTLE
        : ShotCalculator.ShotProfile.NORMAL;
  }

  private double[] getTurretAngleToleranceDegrees(
      Pose2d currentPose,
      Angle desiredTurretAngle,
      Translation2d targetPose,
      Translation2d SOTMOffset,
      Distance distanceToVirtualTarget,
      TargetProfile targetProfile) {
    if (targetPose == null || targetProfile == TargetProfile.NONE) {
      return new double[] {
        -Constants.Launcher.TURRET_ANGLE_TOLERANCE_DEGREES,
        Constants.Launcher.TURRET_ANGLE_TOLERANCE_DEGREES
      };
    }

    Translation2d turretFieldPosition = getTurretFieldPosition(currentPose);
    Rotation2d desiredFieldHeading =
        currentPose
            .getRotation()
            .plus(
                Rotation2d.fromRadians(
                    desiredTurretAngle.in(edu.wpi.first.units.Units.Radians)));

    if (targetProfile == TargetProfile.HUB) {
      return getHubTurretAngleToleranceDegrees(
          turretFieldPosition,
          desiredFieldHeading,
          SOTMOffset,
          Meters.of(
              AllianceFlipUtil.apply(targetPose)
                  .minus(currentPose.getTranslation())
                  .plus(SOTMOffset)
                  .getNorm()));
    }

    return getShuttleTurretAngleToleranceDegrees(
        turretFieldPosition, desiredFieldHeading, targetPose, SOTMOffset);
  }

  private double[] getHubTurretAngleToleranceDegrees(
      Translation2d turretFieldPosition,
      Rotation2d desiredFieldHeading,
      Translation2d SOTMOffset,
      Distance distanceToVirtualTarget) {
    Translation2d adjustedNearLeftCorner =
        AllianceFlipUtil.apply(FieldConstants.Hub.nearLeftCorner).plus(SOTMOffset);
    Translation2d adjustedNearRightCorner =
        AllianceFlipUtil.apply(FieldConstants.Hub.nearRightCorner).plus(SOTMOffset);
    Logger.recordOutput("Launcher/Adjusted Near Left Corner", adjustedNearLeftCorner);
    Logger.recordOutput("Launcher/Adjusted Near Right Corner", adjustedNearRightCorner);
    double toleranceDegrees =
        Math.max(
            Math.toDegrees(
                Math.atan(
                    (FieldConstants.Hub.innerWidth / 2 - 0.075)
                        / distanceToVirtualTarget.in(Meters))),
            MIN_DYNAMIC_TURRET_TOLERANCE_DEGREES);
    return new double[] {-toleranceDegrees, toleranceDegrees};
  }

  private double[] getShuttleTurretAngleToleranceDegrees(
      Translation2d turretFieldPosition,
      Rotation2d desiredFieldHeading,
      Translation2d targetPose,
      Translation2d SOTMOffset) {
    double allianceZoneFarX =
        FieldConstants.TrenchZoneBottom.nearAlliance.getX() - 0.5 * FieldConstants.LeftTrench.depth;
    Translation2d upperFieldEdge =
        AllianceFlipUtil.apply(new Translation2d(allianceZoneFarX, FieldConstants.fieldWidth));
    Translation2d lowerFieldEdge =
        AllianceFlipUtil.apply(new Translation2d(allianceZoneFarX, 0.0));
    Translation2d upperLaneEdge =
        AllianceFlipUtil.apply(
            new Translation2d(allianceZoneFarX, FieldConstants.Hub.nearLeftCorner.getY()));
    Translation2d lowerLaneEdge =
        AllianceFlipUtil.apply(
            new Translation2d(allianceZoneFarX, FieldConstants.Hub.nearRightCorner.getY()));

    if (AllianceFlipUtil.applyY(turretFieldPosition.getY()) >= FieldConstants.fieldWidth / 2.0) {
      Translation2d adjustedUpperFieldEdge = upperFieldEdge.plus(SOTMOffset);
      Translation2d adjustedUpperLaneEdge = upperLaneEdge.plus(SOTMOffset);
      Logger.recordOutput("Launcher/Adjusted Upper Field Edge", adjustedUpperFieldEdge);
      Logger.recordOutput("Launcher/Adjusted Upper Lane Edge", adjustedUpperLaneEdge);
      return getAngularMarginDegrees(
          turretFieldPosition, desiredFieldHeading, adjustedUpperFieldEdge, adjustedUpperLaneEdge);
    }
    Translation2d adjustedLowerFieldEdge = lowerFieldEdge.plus(SOTMOffset);
    Translation2d adjustedLowerLaneEdge = lowerLaneEdge.plus(SOTMOffset);
    Logger.recordOutput("Launcher/Adjusted Lower Field Edge", adjustedLowerFieldEdge);
    Logger.recordOutput("Launcher/Adjusted Lower Lane Edge", adjustedLowerLaneEdge);
    return getAngularMarginDegrees(
        turretFieldPosition, desiredFieldHeading, adjustedLowerFieldEdge, adjustedLowerLaneEdge);
  }

  private double[] getAngularMarginDegrees(
      Translation2d origin,
      Rotation2d desiredFieldHeading,
      Translation2d boundaryA,
      Translation2d boundaryB) {

    double marginA = boundaryA.minus(origin).getAngle().minus(desiredFieldHeading).getDegrees();
    double marginB = boundaryB.minus(origin).getAngle().minus(desiredFieldHeading).getDegrees();

    double lowerBound = Math.min(marginA, marginB);
    double upperBound = Math.max(marginA, marginB);
    lowerBound =
        Math.max(
            Math.min(lowerBound, -MIN_DYNAMIC_TURRET_SHUTTLE_TOLERANCE_DEGREES),
            -MAX_DYNAMIC_TURRET_SHUTTLE_TOLERANCE_DEGREES);
    upperBound =
        Math.min(
            Math.max(upperBound, MIN_DYNAMIC_TURRET_SHUTTLE_TOLERANCE_DEGREES),
            MAX_DYNAMIC_TURRET_SHUTTLE_TOLERANCE_DEGREES);

    return new double[] {lowerBound, upperBound};
  }

  private Translation2d getTurretFieldPosition(Pose2d robotPose) {
    return robotPose.getTranslation().plus(robotToTurret.rotateBy(robotPose.getRotation()));
  }

  @Override
  public void zeroTurret() {
    turret.setPosition(HARD_STOP.in(Rotations));
  }

  @Override
  public boolean isTurretAtZero() {
    return Math.abs(getTurretAngle().in(Degrees) - HARD_STOP.in(Degrees)) < 2.0;
  }

  @Override
  public BooleanSupplier getTurretZeroButtonSupplier() {
    return turretZeroButton::get;
  }
}
