package org.frc5010.common.config.json.devices;

import com.fasterxml.jackson.core.exc.StreamReadException;
import com.fasterxml.jackson.databind.DatabindException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.system.plant.DCMotor;
import java.io.File;
import java.io.IOException;
import java.util.Optional;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.config.json.devices.ReflectionsManager.VENDOR;
import org.frc5010.common.motors.GenericMotorController;
import org.frc5010.common.motors.MotorConstants.Motor;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.hardware.GenericRevBrushlessMotor;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorFactory;

/** A class for reading device configurations from JSON files. */
public class DeviceConfigReader {
  /**
   * Retrieves a motor controller based on the specified type and ID.
   *
   * @param type The type of motor, e.g., "neo" or "kraken".
   * @param id The unique identifier for the motor.
   * @return A MotorController5010 instance for the specified type and ID, or null if the type is
   *     not recognized.
   */
  public static GenericMotorController getMotor(String controller, String type, int id) {
    GenericMotorController motor;
    switch (controller) {
      case "spark":
        motor = MotorFactory.Spark(id, Motor.valueOf(type));
        break;
      case "sparkflex":
        motor = MotorFactory.SparkFlex(id, Motor.valueOf(type));
        break;
      case "talonfx":
        motor = MotorFactory.TalonFX(id, Motor.valueOf(type));
        break;
      case "thrifty":
        motor = MotorFactory.Thrifty(id, Motor.valueOf(type));
        break;
      default:
        return null;
    }
    return motor;
  }

  /**
   * Reads a device configuration from the given file and adds it to the system.
   *
   * @param system the system to add the device to
   * @param deviceFile the file containing the device configuration
   * @param key the key of the device to read (e.g., "gyro", "percent_motor", etc.)
   * @throws StreamReadException if the file cannot be read
   * @throws DatabindException if the file cannot be parsed
   * @throws IOException if there is an error reading the file
   */
  public static void readDeviceConfig(GenericSubsystem system, File deviceFile, String key)
      throws StreamReadException, DatabindException, IOException {
    switch (key) {
      case "gyro":
        GyroSettingsConfigurationJson gyroConfig =
            new ObjectMapper().readValue(deviceFile, GyroSettingsConfigurationJson.class);
        system.addDevice(ConfigConstants.GYRO, gyroConfig.configure(system));
        break;
      case "percent_motor":
        PercentMotorConfigurationJson percentMotorConfig =
            new ObjectMapper().readValue(deviceFile, PercentMotorConfigurationJson.class);
        system.addDevice(percentMotorConfig.name, percentMotorConfig.configure(system));
        break;
      case "velocity_motor":
        VelocityMotorConfigurationJson motorConfigurationJson =
            new ObjectMapper().readValue(deviceFile, VelocityMotorConfigurationJson.class);
        system.addDevice(motorConfigurationJson.name, motorConfigurationJson.configure(system));
        break;
      case "yams_elevator":
        YamsElevatorConfigurationJson yamsElevatorConfigurationJson =
            new ObjectMapper().readValue(deviceFile, YamsElevatorConfigurationJson.class);
        system.addDevice(
            yamsElevatorConfigurationJson.motorSetup.name,
            yamsElevatorConfigurationJson.configure(system));
        break;
      case "yams_arm":
        YamsArmConfigurationJson yamsArmConfigurationJson =
            new ObjectMapper().readValue(deviceFile, YamsArmConfigurationJson.class);
        system.addDevice(
            yamsArmConfigurationJson.motorSetup.name, yamsArmConfigurationJson.configure(system));
        break;
      case "yams_turret":
        YamsPivotConfigurationJson yamsTurretConfigurationJson =
            new ObjectMapper().readValue(deviceFile, YamsPivotConfigurationJson.class);
        system.addDevice(
            yamsTurretConfigurationJson.motorSetup.name,
            yamsTurretConfigurationJson.configure(system));
        break;
      case "yams_shooter":
        YamsShooterConfigurationJson yamsShooterConfigurationJson =
            new ObjectMapper().readValue(deviceFile, YamsShooterConfigurationJson.class);
        system.addDevice(
            yamsShooterConfigurationJson.motorSetup.name,
            yamsShooterConfigurationJson.configure(system));
        break;
      default:
        System.out.println("Unknown device key: " + key);
        break;
    }
  }

  /**
   * Get a {@link SmartMotorController} wrapper from the provided motor controller object and
   * simulate it with the provided motor simulator.
   *
   * @param controller Motor controller type. Supported types are "spark", "sparkmax", "talonfx",
   *     "talonfxs", "nova", "thrifty", "thrifty_nova", and "thriftynova".
   * @param type Motor type. Supported types are "neo", "neo550", "krakenx60", and "krakenx60foc".
   * @param id Motor ID.
   * @param config {@link SmartMotorControllerConfig} for the motor controller.
   * @return {@link SmartMotorController} wrapper for the motor controller.
   */
  public static Optional<SmartMotorController> getSmartMotor(
      String controller, String type, int id, SmartMotorControllerConfig config, String canbus) {
    Optional<SmartMotorController> motor = Optional.empty();
    DCMotor motorSim = null;
    int numberOfMotors = config.getFollowers().map(it -> it.length + 1).orElse(1);
    Object motorController = getReflectedGenericMotor(controller, id, canbus);
    motorSim = getSimulatedMotor(type, numberOfMotors);

    motor =
        SmartMotorFactory.create(
            ((GenericMotorController) motorController).getMotor(), motorSim, config);
    return motor;
  }

  public static DCMotor getSimulatedMotor(String type, int numberOfMotors) {
    DCMotor motorSim = null;
    switch (type.toLowerCase()) {
      case "neo":
        motorSim = DCMotor.getNEO(numberOfMotors);
        break;
      case "neo550":
        motorSim = DCMotor.getNeo550(numberOfMotors);
        break;
      case "neovortex":
        motorSim = DCMotor.getNeoVortex(numberOfMotors);
        break;
      case "krakenx60":
        motorSim = DCMotor.getKrakenX60(numberOfMotors);
        break;
      case "krakenx60foc":
        motorSim = DCMotor.getKrakenX60Foc(numberOfMotors);
        break;
      case "krakenx44":
        motorSim = DCMotor.getKrakenX44(numberOfMotors);
        break;
      default:
    }
    return motorSim;
  }

  /**
   * Get a motor controller based on the controller type and motor type.
   *
   * @param controller The type of motor controller, e.g. "spark", "talonfx", "nova", "thrifty",
   *     "thrifty_nova", or "thriftynova"
   * @param id The CAN ID of the motor controller
   * @param canbus The CAN bus of the motor controller
   * @return The motor controller object
   */
  public static Object getReflectedGenericMotor(String controller, int id, String canbus) {
    ReflectionsManager.VENDOR vendorType = VENDOR.REV;
    switch (controller.toLowerCase()) {
      case "spark":
      case "sparkmax":
        vendorType = VENDOR.REV;
        return ReflectionsManager.<GenericRevBrushlessMotor>create(
            vendorType,
            "org.frc5010.common.motors.hardware.GenericRevBrushlessMotor",
            new Class[] {int.class, boolean.class},
            new Object[] {id, true});
      case "sparkflex":
        vendorType = VENDOR.REV;
        return ReflectionsManager.<GenericRevBrushlessMotor>create(
            vendorType,
            "org.frc5010.common.motors.hardware.GenericRevBrushlessMotor",
            new Class[] {int.class, boolean.class},
            new Object[] {id, false});
      case "talonfx":
        vendorType = VENDOR.PHOENIX5;
        return ReflectionsManager.<SmartMotorController>create(
            vendorType,
            "org.frc5010.common.motors.hardware.GenericTalonFXMotor",
            new Class[] {int.class, String.class},
            new Object[] {id, canbus});
      case "talonfxs":
        vendorType = VENDOR.PHOENIX6;
        return ReflectionsManager.<SmartMotorController>create(
            vendorType,
            "org.frc5010.common.motors.hardware.GenericTalonFXSMotor",
            new Class[] {int.class, String.class},
            new Object[] {id, canbus});
      case "nova":
      case "thrifty":
      case "thrifty_nova":
      case "thriftynova":
        vendorType = VENDOR.THRIFTYBOT;
        return ReflectionsManager.<SmartMotorController>create(
            vendorType,
            "org.frc5010.common.motors.hardware.GenericThriftyNovaMotor",
            new Class[] {int.class},
            new Object[] {id});
      default:
        throw new RuntimeException("Unknown motor controller type: " + controller);
    }
  }

  /**
   * Method to check the existence of specific JSON configuration files in the provided directory.
   *
   * @param directory the directory to check for JSON configuration files
   */
  public static void checkDirectory(File directory) {
    assert new File(directory, "robot.json").exists();
  }
}
