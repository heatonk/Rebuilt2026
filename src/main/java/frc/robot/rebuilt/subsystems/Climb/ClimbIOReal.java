// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Climb;

import edu.wpi.first.units.measure.Distance;
import java.util.Map;
import yams.mechanisms.positional.Elevator;

/** Add your docs here. */
public class ClimbIOReal implements ClimbIO {
  private static Elevator climber;

  protected Map<String, Object> devices;

  public ClimbIOReal(Map<String, Object> devices) {
    this.devices = devices;

    climber = (Elevator) devices.get("lifter");
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {

    inputs.climbHeight = climber.getHeight();
  }

  public void idle() {
    climber.getMotorController().setDutyCycle(0);
  }

  public void setHeight(Distance height) {
    climber.getMotorController().setPosition(height);
  }

  public void runClimb(double speed) {
    climber.getMotorController().setDutyCycle(speed);
  }
}
