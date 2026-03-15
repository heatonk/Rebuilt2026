package org.frc5010.common.config.json.devices;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.frc5010.common.config.ConfigConstants.ControlAlgorithm;
import org.frc5010.common.config.DeviceConfiguration;
import org.frc5010.common.config.UnitsParser;
import org.frc5010.common.config.json.UnitValueJson;
import org.frc5010.common.config.units.DistanceUnit;
import org.frc5010.common.config.units.MassUnit;
import org.frc5010.common.config.units.VoltageUnit;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class YamsElevatorConfigurationJson implements DeviceConfiguration {
  public MotorSetupJson motorSetup = new MotorSetupJson();
  public ControlAlgorithm controlAlgorithm = ControlAlgorithm.SIMPLE;
  public MotorSystemIdJson motorSystemId = new MotorSystemIdJson();
  public MotorSystemIdJson simSystemId = new MotorSystemIdJson();
  public int sprocketTeeth = 0;
  public UnitValueJson drumRadius = new UnitValueJson(0, DistanceUnit.INCHES.toString());
  public UnitValueJson lowerSoftLimit = new UnitValueJson(0, DistanceUnit.METERS.toString());
  public UnitValueJson upperSoftLimit = new UnitValueJson(0, DistanceUnit.METERS.toString());
  public UnitValueJson lowerHardLimit = new UnitValueJson(0, DistanceUnit.METERS.toString());
  public UnitValueJson upperHardLimit = new UnitValueJson(0, DistanceUnit.METERS.toString());
  public double[] gearing;
  public String gearStages = "";
  public UnitValueJson startingPosition = new UnitValueJson(0, DistanceUnit.METERS.toString());
  public UnitValueJson mass = new UnitValueJson(0, MassUnit.POUNDS.toString());
  public UnitValueJson voltageCompensation = new UnitValueJson(12, VoltageUnit.VOLTS.toString());

  /**
   * Configure the given GenericSubsystem with an elevator using the given json configuration.
   *
   * @param deviceHandler the GenericSubsystem to configure
   * @return the configured elevator
   */
  @Override
  public Elevator configure(SubsystemBase deviceHandler) {
    SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(deviceHandler);
    if (sprocketTeeth > 0) {
      motorConfig.withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * sprocketTeeth));
    } else if (drumRadius.val > 0) {
      motorConfig.withMechanismCircumference(UnitsParser.parseDistance(drumRadius));
    } else {
      throw new RuntimeException("Must specify either sproketTeeth or drumRadius");
    }

    motorConfig
        .withSoftLimit(
            UnitsParser.parseDistance(lowerSoftLimit), UnitsParser.parseDistance(upperSoftLimit))
        .withFeedforward(
            new ElevatorFeedforward(
                motorSystemId.feedForward.s,
                motorSystemId.feedForward.g,
                motorSystemId.feedForward.v,
                motorSystemId.feedForward.a))
        .withSimFeedforward(
            new ElevatorFeedforward(
                simSystemId.feedForward.s,
                simSystemId.feedForward.g,
                simSystemId.feedForward.v,
                simSystemId.feedForward.a));

    YamsConfigCommon.PhysicalParameters physicalParams =
        new YamsConfigCommon.PhysicalParameters(
            voltageCompensation, mass, drumRadius, gearing, gearStages);

    Optional<SmartMotorController> smartMotor =
        YamsConfigCommon.configureSmartMotorController(
            motorSetup, motorConfig, controlAlgorithm, motorSystemId, simSystemId, physicalParams);
    if (smartMotor.isEmpty()) {
      throw new RuntimeException(
          "Elevator Smart motor configuration issue. ID: "
              + motorSetup.canId
              + " Controller: "
              + motorSetup.controllerType
              + " Motor: "
              + motorSetup.motorType);
    }
    ElevatorConfig m_config =
        new ElevatorConfig(smartMotor.get())
            .withStartingHeight(UnitsParser.parseDistance(startingPosition))
            .withHardLimits(
                UnitsParser.parseDistance(lowerHardLimit),
                UnitsParser.parseDistance(upperHardLimit))
            .withTelemetry(motorSetup.name, TelemetryVerbosity.valueOf(motorSetup.logLevel))
            .withMechanismPositionConfig(motorSetup.getMechanismPositionConfig())
            .withMass(UnitsParser.parseMass(mass));
    Elevator elevator = new Elevator(m_config);
    return elevator;
  }
}
