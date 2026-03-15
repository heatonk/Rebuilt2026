// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.swerve;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.frc5010.common.config.UnitsParser;
import org.frc5010.common.config.json.devices.DrivetrainConstantsJson;
import org.frc5010.common.drive.SwerveDriveConfig;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

/** Add your docs here. */
public class AkitSwerveConfig extends SwerveDriveConfig {
  public final SwerveDrivetrainConstants DrivetrainConstants;

  public final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft;
  public final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight;
  public final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft;
  public final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight;
  public double ODOMETRY_FREQUENCY;
  public final double DRIVE_BASE_RADIUS;
  protected SwerveModuleConstants[] MODULES;
  private final CANBus kCANBus;

  public AkitSwerveConfig(AkitTalonFXSwerveConfigBuilder builder) {
    super(builder);
    this.DrivetrainConstants = builder.DrivetrainConstants;
    this.FrontLeft = builder.FrontLeft;
    this.FrontRight = builder.FrontRight;
    this.BackLeft = builder.BackLeft;
    this.BackRight = builder.BackRight;
    MODULES = new SwerveModuleConstants[] {FrontLeft, FrontRight, BackLeft, BackRight};
    this.kCANBus = builder.kCANBus;
    DRIVE_BASE_RADIUS =
        Math.max(
            Math.max(
                Math.hypot(FrontLeft.LocationX, FrontLeft.LocationY),
                Math.hypot(FrontRight.LocationX, FrontRight.LocationY)),
            Math.max(
                Math.hypot(BackLeft.LocationX, BackLeft.LocationY),
                Math.hypot(BackRight.LocationX, BackRight.LocationY)));
  }

  public SwerveModuleConstants getModuleConstants(int index) {
    return MODULES[index];
  }

  public static AkitSwerveConfig builder(
      DrivetrainConstantsJson constants, SubsystemBase subsystem) {
    return new AkitSwerveConfig(new AkitTalonFXSwerveConfigBuilder(constants, subsystem));
  }

  public CANBus getCANBus() {
    return kCANBus;
  }

  public static class AkitTalonFXSwerveConfigBuilder
      extends SwerveDriveConfig.SwerveDriveConfigBuilder {
    public final SwerveDrivetrainConstants DrivetrainConstants;

    private final SwerveModuleConstantsFactory<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        ConstantCreator;

    public final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontLeft;
    public final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontRight;
    public final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackLeft;
    public final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackRight;
    public final CANBus kCANBus;

    public AkitTalonFXSwerveConfigBuilder(
        DrivetrainConstantsJson constants, SubsystemBase subsystem) {
      super();
      kCANBus = new CANBus(constants.canbus, "./logs/example.hoot");
      bumperFrameLength = UnitsParser.parseDistance(constants.bumperFrameLength);
      bumperFrameWidth = UnitsParser.parseDistance(constants.bumperFrameWidth);
      maxDriveSpeed = UnitsParser.parseVelocity(constants.maxDriveSpeed);
      robotMass = UnitsParser.parseMass(constants.robotMass);
      driveInertia = UnitsParser.parseMomentOfInertia(constants.driveInertia);
      steerInertia = UnitsParser.parseMomentOfInertia(constants.steerInertia);
      canbus = Constants.CURRENT_MODE == Constants.SIM_MODE ? "" : constants.canbus;
      wheelDiameter = UnitsParser.parseDistance(constants.wheelDiameter);
      driveGearRatio =
          new MechanismGearing(GearBox.fromStages(constants.driveGearRatio))
              .getRotorToMechanismRatio();
      steerGearRatio =
          new MechanismGearing(GearBox.fromStages(constants.steerGearRatio))
              .getRotorToMechanismRatio();
      DrivetrainConstants =
          new SwerveDrivetrainConstants()
              .withCANBusName(kCANBus.getName())
              .withPigeon2Id(constants.gyro.id)
              .withPigeon2Configs(null);

      CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
      ConstantCreator =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(
                  new MechanismGearing(GearBox.fromStages(constants.driveGearRatio))
                      .getRotorToMechanismRatio())
              .withSteerMotorGearRatio(
                  new MechanismGearing(GearBox.fromStages(constants.steerGearRatio))
                      .getRotorToMechanismRatio())
              .withCouplingGearRatio(constants.coupleRatio)
              .withWheelRadius(UnitsParser.parseDistance(constants.wheelDiameter).div(2))
              .withSteerMotorGains(
                  new Slot0Configs()
                      .withKP(constants.steerMotorControl.feedBack.p)
                      .withKI(constants.steerMotorControl.feedBack.i)
                      .withKD(constants.steerMotorControl.feedBack.d)
                      .withKS(constants.steerMotorControl.feedForward.s)
                      .withKV(constants.steerMotorControl.feedForward.v)
                      .withKA(constants.steerMotorControl.feedForward.a)
                      .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
              .withDriveMotorGains(
                  new Slot0Configs()
                      .withKP(constants.driveMotorControl.feedBack.p)
                      .withKI(constants.driveMotorControl.feedBack.i)
                      .withKD(constants.driveMotorControl.feedBack.d)
                      .withKS(constants.driveMotorControl.feedForward.s)
                      .withKV(constants.driveMotorControl.feedForward.v)
                      .withKA(constants.driveMotorControl.feedForward.a))
              .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
              .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
              .withSlipCurrent(UnitsParser.parseAmps(constants.slipCurrent))
              .withSpeedAt12Volts(UnitsParser.parseVelocity(constants.maxDriveSpeed))
              .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
              .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
              .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
              .withDriveMotorInitialConfigs(new TalonFXConfiguration())
              .withSteerMotorInitialConfigs(
                  new TalonFXConfiguration()
                      .withCurrentLimits(
                          new CurrentLimitsConfigs()
                              // Swerve azimuth does not require much torque output, so we can set
                              // arelatively low stator current limit to help avoid brownouts
                              // without impacting performance.
                              .withStatorCurrentLimit(Amps.of(30))
                              .withStatorCurrentLimitEnable(true)))
              .withEncoderInitialConfigs(encoderInitialConfigs)
              .withSteerInertia(UnitsParser.parseMomentOfInertia(constants.steerInertia))
              .withDriveInertia(UnitsParser.parseMomentOfInertia(constants.driveInertia))
              .withSteerFrictionVoltage(constants.steerMotorControl.feedForward.s)
              .withDriveFrictionVoltage(constants.driveMotorControl.feedForward.s);
      trackWidth = UnitsParser.parseDistance(constants.trackWidth);
      wheelBase = UnitsParser.parseDistance(constants.wheelBase);
      Distance kFrontLeftXPos = trackWidth.div(2);
      Distance kFrontLeftYPos = wheelBase.div(2);
      FrontLeft =
          ConstantCreator.createModuleConstants(
              constants.modules.get("frontLeft").steerMotorSetup.canId,
              constants.modules.get("frontLeft").driveMotorSetup.canId,
              constants.modules.get("frontLeft").encoderId,
              UnitsParser.parseAngle(constants.modules.get("frontLeft").absoluteOffset),
              kFrontLeftXPos,
              kFrontLeftYPos,
              constants.invertLeftSide,
              constants.modules.get("frontLeft").steerMotorSetup.inverted,
              constants.modules.get("frontLeft").encoderInverted);
      Distance kFrontRightXPos = trackWidth.div(2);
      Distance kFrontRightYPos = wheelBase.div(-2);
      FrontRight =
          ConstantCreator.createModuleConstants(
              constants.modules.get("frontRight").steerMotorSetup.canId,
              constants.modules.get("frontRight").driveMotorSetup.canId,
              constants.modules.get("frontRight").encoderId,
              UnitsParser.parseAngle(constants.modules.get("frontRight").absoluteOffset),
              kFrontRightXPos,
              kFrontRightYPos,
              constants.invertRightSide,
              constants.modules.get("frontRight").steerMotorSetup.inverted,
              constants.modules.get("frontRight").encoderInverted);

      Distance kBackLeftXPos = trackWidth.div(-2);
      Distance kBackLeftYPos = wheelBase.div(2);
      BackLeft =
          ConstantCreator.createModuleConstants(
              constants.modules.get("backLeft").steerMotorSetup.canId,
              constants.modules.get("backLeft").driveMotorSetup.canId,
              constants.modules.get("backLeft").encoderId,
              UnitsParser.parseAngle(constants.modules.get("backLeft").absoluteOffset),
              kBackLeftXPos,
              kBackLeftYPos,
              constants.invertLeftSide,
              constants.modules.get("backLeft").steerMotorSetup.inverted,
              constants.modules.get("backLeft").encoderInverted);

      Distance kBackRightXPos = trackWidth.div(-2);
      Distance kBackRightYPos = wheelBase.div(-2);
      BackRight =
          ConstantCreator.createModuleConstants(
              constants.modules.get("backRight").steerMotorSetup.canId,
              constants.modules.get("backRight").driveMotorSetup.canId,
              constants.modules.get("backRight").encoderId,
              UnitsParser.parseAngle(constants.modules.get("backRight").absoluteOffset),
              kBackRightXPos,
              kBackRightYPos,
              constants.invertRightSide,
              constants.modules.get("backRight").steerMotorSetup.inverted,
              constants.modules.get("backRight").encoderInverted);
    }
  }
}
