// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json.devices;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.frc5010.common.config.ConfigConstants.ControlAlgorithm;
import org.frc5010.common.config.DeviceConfiguration;
import org.frc5010.common.config.UnitsParser;
import org.frc5010.common.config.json.UnitValueJson;
import org.frc5010.common.config.units.AngularVelocityUnit;
import org.frc5010.common.config.units.DistanceUnit;
import org.frc5010.common.config.units.MassUnit;
import org.frc5010.common.config.units.VoltageUnit;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/** Add your docs here. */
public class YamsShooterConfigurationJson implements DeviceConfiguration {
  public MotorSetupJson motorSetup = new MotorSetupJson();
  public ControlAlgorithm controlAlgorithm = ControlAlgorithm.SIMPLE;
  public MotorSystemIdJson motorSystemId = new MotorSystemIdJson();
  public MotorSystemIdJson simSystemId = new MotorSystemIdJson();
  public UnitValueJson lowerSoftLimit =
      new UnitValueJson(0, AngularVelocityUnit.DEGREES_PER_SECOND.toString());
  public UnitValueJson upperSoftLimit =
      new UnitValueJson(0, AngularVelocityUnit.DEGREES_PER_SECOND.toString());
  public double[] gearing;
  public String gearStages = "";
  public UnitValueJson voltageCompensation = new UnitValueJson(12, VoltageUnit.VOLTS.toString());
  public UnitValueJson mass = new UnitValueJson(0, MassUnit.POUNDS.toString());
  public UnitValueJson radius = new UnitValueJson(0, DistanceUnit.INCHES.toString());

  /**
   * Configure the given GenericSubsystem with a shooter using the given json configuration.
   *
   * @param deviceHandler the GenericSubsystem to configure
   * @return the configured shooter
   */
  @Override
  public FlyWheel configure(SubsystemBase deviceHandler) {
    SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(deviceHandler)
            .withFeedforward(
                new SimpleMotorFeedforward(
                    motorSystemId.feedForward.s,
                    motorSystemId.feedForward.v,
                    motorSystemId.feedForward.a))
            .withSimFeedforward(
                new SimpleMotorFeedforward(
                    simSystemId.feedForward.s,
                    simSystemId.feedForward.v,
                    simSystemId.feedForward.a))
            .withControlMode(ControlMode.valueOf(motorSystemId.controlMode));
    YamsConfigCommon.PhysicalParameters physicalParams =
        new YamsConfigCommon.PhysicalParameters(
            voltageCompensation, mass, radius, gearing, gearStages);

    Optional<SmartMotorController> smartMotor =
        YamsConfigCommon.configureSmartMotorController(
            motorSetup, motorConfig, controlAlgorithm, motorSystemId, simSystemId, physicalParams);
    if (smartMotor.isEmpty()) {
      throw new RuntimeException(
          "Smart motor not found. ID: "
              + motorSetup.canId
              + " Controller: "
              + motorSetup.controllerType
              + " Motor: "
              + motorSetup.motorType);
    }
    FlyWheelConfig shooterConfig =
        new FlyWheelConfig(smartMotor.get())
            .withMechanismPositionConfig(motorSetup.getMechanismPositionConfig())
            .withDiameter(UnitsParser.parseDistance(radius).times(2.0))
            .withMass(UnitsParser.parseMass(mass))
            .withUpperSoftLimit(UnitsParser.parseAngularVelocity(upperSoftLimit))
            .withLowerSoftLimit(UnitsParser.parseAngularVelocity(lowerSoftLimit))
            .withSpeedometerSimulation(UnitsParser.parseAngularVelocity(upperSoftLimit))
            .withTelemetry(motorSetup.name, TelemetryVerbosity.valueOf(motorSetup.logLevel));
    shooterConfig.withMOI(UnitsParser.parseDistance(radius), UnitsParser.parseMass(mass));
    FlyWheel shooter = new FlyWheel(shooterConfig);
    return shooter;
  }
}
