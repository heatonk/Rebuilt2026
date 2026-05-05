// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.arch;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.Map;
import org.frc5010.common.arch.GenericRobot.LogLevel;
import org.frc5010.common.motors.function.GenericFunctionalMotor;
import org.frc5010.common.telemetry.DisplayValuesHelper;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import yams.mechanisms.SmartMechanism;

/** Base class for subsystems that provides default logging and network table support */
public class GenericSubsystem extends SubsystemBase
    implements WpiHelperInterface, GenericDeviceHandler {
  /** The display values helper */
  protected final DisplayValuesHelper DashBoard;
  /** The network table values */
  protected final WpiNetworkTableValuesHelper networkValues = new WpiNetworkTableValuesHelper();
  /** The log prefix */
  protected String logPrefix = getClass().getSimpleName();
  /** The mechanism simulation */
  protected LoggedMechanism2d mechanismSimulation;
  /** The map of devices created by the configuration system */
  protected Map<String, Object> devices = new HashMap<>();

  protected Alert loggingAlert =
      new Alert(logPrefix + " Logging Mode is not COMPETITION!", AlertType.kWarning);

  /** Creates a new LoggedSubsystem. */
  protected GenericSubsystem(LoggedMechanism2d mechanismSimulation) {
    this.mechanismSimulation = mechanismSimulation;
    DashBoard = new DisplayValuesHelper(logPrefix);
    WpiNetworkTableValuesHelper.register(this);
  }

  protected GenericSubsystem() {
    DashBoard = new DisplayValuesHelper(logPrefix);
    WpiNetworkTableValuesHelper.register(this);
  }

  protected GenericSubsystem(String configFile) {
    DashBoard = new DisplayValuesHelper(logPrefix);
    try {
      GenericRobot.subsystemParser.parseSubsystem(this, configFile);
    } catch (Exception e) {
      e.printStackTrace();
      System.exit(1);
    }
    WpiNetworkTableValuesHelper.register(this);
  }

  /**
   * Get the mechanism simulation visual
   *
   * @return the mechanism visual
   */
  public LoggedMechanism2d getMechVisual() {
    return mechanismSimulation;
  }

  /**
   * Set the mechanism simulation visual
   *
   * @param mechSim the mechanism visual
   */
  public void setMechSimulation(LoggedMechanism2d mechSim) {
    mechanismSimulation = mechSim;
  }

  /**
   * Add a device to the configuration
   *
   * @param name the name of the device
   * @param device the device
   */
  @Override
  public void addDevice(String name, Object device) {
    devices.put(name, device);
  }

  public void setLoggingLevel(LogLevel logLevel) {
    DashBoard.setLoggingLevel(logLevel);
    if (logLevel == LogLevel.COMPETITION) {
      loggingAlert.set(false);
      ;
    } else {
      loggingAlert.set(true);
    }
  }

  /**
   * Get a device from the configuration
   *
   * @param name the name of the device
   * @return the device
   */
  @Override
  public Object getDevice(String name) {
    return devices.get(name);
  }

  /**
   * Called when the command is created and registers it with the network tables
   *
   * @param builder - The sendable builder
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    log(logPrefix + ": Initializing sendables.");
    networkValues.initSendables(builder, this.getClass().getSimpleName());
  }

  /**
   * Called every time the scheduler runs while the robot is enabled. Used to update display values
   * and draw motor graphics.
   */
  @Override
  public void periodic() {
    DashBoard.notifyListeners();
    for (Object it : devices.values()) {
      if (it instanceof GenericFunctionalMotor) {
        ((GenericFunctionalMotor) it).periodicUpdate();
      }
      if (it instanceof SmartMechanism) {
        ((SmartMechanism) it).updateTelemetry();
      }
    }
  }

  /**
   * Called every time the scheduler runs while the robot is in simulation mode. Used to update
   * simulation models.
   */
  @Override
  public void simulationPeriodic() {
    for (Object it : devices.values()) {
      if (it instanceof GenericFunctionalMotor) {
        ((GenericFunctionalMotor) it).simulationUpdate();
      }
      if (it instanceof SmartMechanism) {
        ((SmartMechanism) it).simIterate();
      }
    }
  }

  /**
   * Get the display values helper associated with the subsystem.
   *
   * @return the DisplayValuesHelper instance for managing display values
   */
  @Override
  public DisplayValuesHelper getDisplayValuesHelper() {
    return DashBoard;
  }

  /**
   * Sets the display state of the subsystem.
   *
   * @param display a boolean indicating whether to enable or disable the display
   */
  public void setDisplay(boolean display) {
    if (display) DashBoard.makeDisplayed();
  }

  public static Object setHeight(Distance of) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setHeight'");
  }
}
