// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.commands.IntakeCommands;
import frc.robot.rebuilt.commands.IntakeCommands.IntakeState;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.sensors.Controller;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends GenericSubsystem {
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new Intake and selects the IO */
  public Intake() {
    super("intake.json");
    if (RobotBase.isSimulation()) {
      io = new IntakeIOSim(devices);
    } else {
      io = new IntakeIOReal(devices);
    }
  }

  public void runSpintake(double speed) {
    io.runSpintake(speed);
  }

  public void runSpintakes(double outerSpeed, double innerSpeed) {
    io.runSpintakes(innerSpeed, outerSpeed);
  }

  /** Creates a command that runs the spintake at the given speed and stops when done */
  public Command spintakeCommand(double speed) {
    return Commands.run(
            () -> {
              runSpintake(speed);
            })
        .finallyDo(
            () -> {
              runSpintake(0);
            });
  }

  public Command setDesiredHopperAngle(Angle angle) {
    return io.setHopperAngle(angle);
  }

  public Angle getHopperAngle() {
    return inputs.hopperAngleActual;
  }

  public boolean isRetracted() {
    return io.isRetracted();
  }

  public boolean isDeployed() {
    return io.isDeployed();
  }

  public void runHopper(double speed) {
    io.runHopper(speed);
  }
  /** Configures test controller bindings for the spintake, hopper control, and sysid */
  public void configTestController(Controller controller) {
    controller.createRightBumper().whileTrue(spintakeCommand(0.5));
    controller.createYButton().whileTrue(getHopperSysIdCommand());
    controller.setRightYAxis(controller.createRightYAxis());
    Trigger rightYAxis = new Trigger(() -> controller.getRightYAxis() > 0.01);
    rightYAxis.whileTrue(Commands.run(() -> runHopper(controller.getRightYAxis())));
  }
  /** Updates intake inputs from the io periodically and logs them each robot cycle */
  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public boolean isRequested(IntakeState state) {
    return inputs.stateRequested == state;
  }

  public boolean isCurrent(IntakeState state) {
    return inputs.stateCurrent == state;
  }

  public boolean isNearTrench() {
    return isCurrent(IntakeState.DEPLOYING) && io.isNearTrench();
  }

  public void setCurrentState(IntakeState state) {
    inputs.stateCurrent = state;
  }

  public IntakeState getCurrentState() {
    return inputs.stateCurrent;
  }

  public boolean isHopperStalling() {
    return io.isHopperStalling();
  }

  public Command getHopperSysIdCommand() {
    return io.getHopperSysIdCommand();
  }

  public Command getHopperCharacterizationCommand() {
    return io.getHopperCharacterizationCommand(this);
  }

  public void setRequestedState(IntakeState state) {
    inputs.stateRequested = state;
  }

  public void setHopperDeployed() {
    io.setHopperPosition(Degrees.of(0));
    setRequestedState(IntakeCommands.IntakeState.DEPLOYED);
  }

  public void setHopperRetracted() {
    io.setHopperPosition(Degrees.of(120));
    setRequestedState(IntakeCommands.IntakeState.RETRACTED);
  }

  public boolean isHopperMoving() {
    return io.isHopperMoving();
  }

  public void setHopperPosition(Angle angle) {
    io.setHopperPosition(angle);
  }

  public boolean isHopperAtGoal() {
    return inputs.hopperAtGoal;
  }

  /**
   * Returns the field-relative 3D pose of the hopper arm for AdvantageScope visualization. Log key:
   * {@code Intake/ComponentPoses}.
   *
   * <p>The hopper pivot is modelled as mounted on the side of the robot. It rotates about the
   * robot-local X axis (roll) as the arm deploys/retracts. The mount offset below is a placeholder
   * — adjust the X/Y/Z values to match the actual hopper pivot position from intake.json.
   */
  @AutoLogOutput(key = "Intake/ComponentPoses")
  public Pose3d[] getComponentPoses() {
    Pose3d robotPose = Rebuilt.drivetrain.getPoseEstimator().getCurrentPose3d();
    // Hopper pivot: offset 0.35 m to the left (+Y) and 0.3 m up (+Z) from robot center.
    // Rolls about the robot-local X axis by the hopper angle (0° = deployed, 120° = retracted).
    Pose3d hopperPose =
        robotPose.transformBy(
            new Transform3d(
                new Translation3d(0.0, 0.35, 0.3),
                new Rotation3d(inputs.hopperAngleActual.in(Radians), 0, 0)));
    return new Pose3d[] {hopperPose};
  }
}
