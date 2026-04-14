// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json.devices;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;
import java.util.Optional;
import org.frc5010.common.config.ConfigConstants.ControlAlgorithm;
import org.frc5010.common.config.UnitsParser;
import org.frc5010.common.config.json.UnitValueJson;
import org.frc5010.common.motors.GenericMotorController;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.math.ExponentialProfilePIDController;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.SmartMotorFactory;

/** Add your docs here. */
public class YamsConfigCommon {
  public static class PhysicalParameters {
    public Voltage voltageCompensation;
    public Mass mass;
    public Distance length;
    public MechanismGearing gearing;

    public PhysicalParameters(
        UnitValueJson voltageCompensation,
        UnitValueJson mass,
        UnitValueJson length,
        double[] gearing,
        String gearStages) {
      this.voltageCompensation = UnitsParser.parseVolts(voltageCompensation);
      this.mass = UnitsParser.parseMass(mass);
      this.length = UnitsParser.parseDistance(length);
      if (!gearStages.isEmpty()) {
        this.gearing = new MechanismGearing(GearBox.fromStages(gearStages));
      } else { // gearStages is empty, so use gearing array
        this.gearing = new MechanismGearing(GearBox.fromReductionStages(gearing));
      }
    }
  }

  public static Optional<SmartMotorController> configureSmartMotorController(
      MotorSetupJson motorSetup,
      SmartMotorControllerConfig motorConfig,
      ControlAlgorithm controlAlgorithm,
      MotorSystemIdJson motorSystemId,
      MotorSystemIdJson simSystemId,
      PhysicalParameters physicalParams) {

    MotorSetupJson.setupFollowers(motorConfig, motorSetup);
    int numberOfMotors = motorConfig.getFollowers().map(it -> it.length + 1).orElse(1);
    Object motorController =
        DeviceConfigReader.getReflectedGenericMotor(
            motorSetup.controllerType, motorSetup.canId, motorSetup.canBus);
    DCMotor motorSim = DeviceConfigReader.getSimulatedMotor(motorSetup.motorType, numberOfMotors);

    if (Robot.isSimulation() && !ControlAlgorithm.SIMPLE.equals(controlAlgorithm)) {
      controlAlgorithm = ControlAlgorithm.SIMPLE;
    }
    switch (controlAlgorithm) {
      case PROFILED:
        {
          motorConfig
              .withClosedLoopController(
                  motorSystemId.feedBack.p,
                  motorSystemId.feedBack.i,
                  motorSystemId.feedBack.d,
                  UnitsParser.parseAngularVelocity(motorSystemId.maxVelocity),
                  UnitsParser.parseAngularAcceleration(motorSystemId.maxAcceleration))
              .withSimClosedLoopController(
                  simSystemId.feedBack.p,
                  simSystemId.feedBack.i,
                  simSystemId.feedBack.d,
                  UnitsParser.parseAngularVelocity(simSystemId.maxVelocity),
                  UnitsParser.parseAngularAcceleration(simSystemId.maxAcceleration));
          break;
        }
      case EXPO:
      case EXPO_ELEVATOR:
        {
          ExponentialProfile.Constraints constraints;
          if (controlAlgorithm == ControlAlgorithm.EXPO_ELEVATOR) {
            constraints =
                ExponentialProfilePIDController.createElevatorConstraints(
                    physicalParams.voltageCompensation,
                    motorSim,
                    physicalParams.mass,
                    physicalParams.length,
                    physicalParams.gearing);
          } else {
            constraints =
                ExponentialProfilePIDController.createArmConstraints(
                    physicalParams.voltageCompensation,
                    motorSim,
                    physicalParams.mass,
                    physicalParams.length,
                    physicalParams.gearing);
          }
          ExponentialProfilePIDController controller =
              new ExponentialProfilePIDController(
                  motorSystemId.feedBack.p,
                  motorSystemId.feedBack.i,
                  motorSystemId.feedBack.d,
                  constraints);
          ExponentialProfilePIDController simController =
              new ExponentialProfilePIDController(
                  simSystemId.feedBack.p,
                  simSystemId.feedBack.i,
                  simSystemId.feedBack.d,
                  constraints);
          motorConfig
              .withClosedLoopController(controller)
              .withSimClosedLoopController(simController);
          break;
        }
      case SIMPLE:
      default:
        {
          motorConfig
              .withClosedLoopController(
                  motorSystemId.feedBack.p, motorSystemId.feedBack.i, motorSystemId.feedBack.d)
              .withSimClosedLoopController(
                  simSystemId.feedBack.p, simSystemId.feedBack.i, simSystemId.feedBack.d);
          break;
        }
    }
    motorConfig
        .withGearing(physicalParams.gearing)
        .withControlMode(ControlMode.valueOf(motorSystemId.controlMode))
        .withClosedLoopRampRate(UnitsParser.parseTime(motorSystemId.closedLoopRamp))
        .withIdleMode(MotorMode.valueOf(motorSetup.idleMode))
        .withTelemetry(motorSetup.name + "Motor", TelemetryVerbosity.valueOf(motorSetup.logLevel))
        .withStatorCurrentLimit(UnitsParser.parseAmps(motorSetup.currentLimit))
        .withMotorInverted(motorSetup.inverted);

    switch (motorSetup.controllerType.toLowerCase()) {
      case "thrifty":
      case "nova":
      case "thriftynova":
      case "thrifty_nova":
        break;
      default:
        motorConfig.withOpenLoopRampRate(UnitsParser.parseTime(motorSystemId.openLoopRamp));
        break;
    }
    return SmartMotorFactory.create(
        ((GenericMotorController) motorController).getMotor(), motorSim, motorConfig);
  }
}
