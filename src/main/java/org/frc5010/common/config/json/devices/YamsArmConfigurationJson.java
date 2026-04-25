// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json.devices;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.frc5010.common.config.ConfigConstants.ControlAlgorithm;
import org.frc5010.common.config.DeviceConfiguration;
import org.frc5010.common.config.UnitsParser;
import org.frc5010.common.config.json.UnitValueJson;
import org.frc5010.common.config.units.AngleUnit;
import org.frc5010.common.config.units.DistanceUnit;
import org.frc5010.common.config.units.MassUnit;
import org.frc5010.common.config.units.VoltageUnit;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class YamsArmConfigurationJson implements DeviceConfiguration {
  public MotorSetupJson motorSetup = new MotorSetupJson();
  public ControlAlgorithm controlAlgorithm = ControlAlgorithm.SIMPLE;
  public MotorSystemIdJson motorSystemId = new MotorSystemIdJson();
  public MotorSystemIdJson simSystemId = new MotorSystemIdJson();
  public UnitValueJson length = new UnitValueJson(0, DistanceUnit.INCHES.toString());
  public UnitValueJson lowerHardLimit = new UnitValueJson(0, AngleUnit.DEGREES.toString());
  public UnitValueJson upperHardLimit = new UnitValueJson(0, AngleUnit.DEGREES.toString());
  public UnitValueJson startingAngle = new UnitValueJson(0, AngleUnit.DEGREES.toString());
  public UnitValueJson lowerSoftLimit = new UnitValueJson(0, AngleUnit.DEGREES.toString());
  public UnitValueJson upperSoftLimit = new UnitValueJson(0, AngleUnit.DEGREES.toString());
  public double[] gearing;
  public String gearStages = "";
  public UnitValueJson mass = new UnitValueJson(0, MassUnit.POUNDS.toString());
  public UnitValueJson voltageCompensation = new UnitValueJson(12, VoltageUnit.VOLTS.toString());
  public UnitValueJson horizontalZero = new UnitValueJson(0, AngleUnit.DEGREES.toString());

  /**
   * Configure the given GenericSubsystem with an arm using the given json configuration.
   *
   * @param deviceHandler the GenericSubsystem to configure
   * @return the configured arm
   */
  @Override
  public Arm configure(SubsystemBase deviceHandler) {
    SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(deviceHandler)
            .withSoftLimit(
                UnitsParser.parseAngle(lowerSoftLimit), UnitsParser.parseAngle(upperSoftLimit))
            .withFeedforward(
                new ArmFeedforward(
                    motorSystemId.feedForward.s,
                    motorSystemId.feedForward.g,
                    motorSystemId.feedForward.v,
                    motorSystemId.feedForward.a))
            .withSimFeedforward(
                new ArmFeedforward(
                    simSystemId.feedForward.s,
                    simSystemId.feedForward.g,
                    simSystemId.feedForward.v,
                    simSystemId.feedForward.a));

    YamsConfigCommon.PhysicalParameters physicalParams =
        new YamsConfigCommon.PhysicalParameters(
            voltageCompensation, mass, length, gearing, gearStages);

    Optional<SmartMotorController> smartMotor =
        YamsConfigCommon.configureSmartMotorController(
            motorSetup, motorConfig, controlAlgorithm, motorSystemId, simSystemId, physicalParams);
    if (smartMotor.isEmpty()) {
      throw new RuntimeException(
          "Smart motor configuration issue. ID: "
              + motorSetup.canId
              + " Controller: "
              + motorSetup.controllerType
              + " Motor: "
              + motorSetup.motorType);
    }

    ArmConfig armConfig =
        new ArmConfig(smartMotor.get())
            .withLength(physicalParams.length)
            .withHardLimit(
                UnitsParser.parseAngle(lowerHardLimit), UnitsParser.parseAngle(upperHardLimit))
            .withTelemetry(motorSetup.name, TelemetryVerbosity.valueOf(motorSetup.logLevel))
            .withMass(physicalParams.mass)
            .withStartingPosition(UnitsParser.parseAngle(startingAngle))
            .withMechanismPositionConfig(motorSetup.getMechanismPositionConfig())
            .withHorizontalZero(UnitsParser.parseAngle(horizontalZero));
    Arm arm = new Arm(armConfig);
    return arm;
  }
}
