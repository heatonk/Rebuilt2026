// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.frc5010.common.drive.swerve.akit;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Robot;
import org.frc5010.common.drive.swerve.AkitSwerveConfig;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;

  /** Grow-only array: never shrunk to minimise GC pressure from per-cycle allocation */
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[0];

  public Module(
      ModuleIO io,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Disconnected turn encoder on module " + Integer.toString(index) + ".",
            AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount =
        Math.min(
            inputs.odometryDrivePositionsRad.length,
            inputs.odometryTurnPositions.length); // All signals are sampled together
    // Grow the array only when needed; never shrink to avoid per-cycle GC pressure
    if (odometryPositions.length != sampleCount) {
      odometryPositions = new SwerveModulePosition[sampleCount];
      for (int i = 0; i < sampleCount; i++) {
        odometryPositions[i] = new SwerveModulePosition();
      }
    }
    for (int i = 0; i < sampleCount; i++) {
      odometryPositions[i].distanceMeters =
          inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
      odometryPositions[i].angle = inputs.odometryTurnPositions[i];
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(Robot.isSimulation() ? inputs.turnAbsolutePosition : inputs.turnPosition);

    // Apply setpoints
    io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to rotation angles. */
  public void runCharacterization(double output, AkitSwerveConfig config) {
    io.setDriveOpenLoop(output);
    // CLR - This sets the turn positions to rotate the robot
    io.setTurnPosition(
        new Rotation2d(
                config.getModuleConstants(index).LocationX,
                config.getModuleConstants(index).LocationY)
            .plus(Rotation2d.kCCW_Pi_2));
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runSteerCharacterization(double output) {
    io.setDriveOpenLoop(0);
    io.setTurnOpenLoop(output);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    if (Robot.isSimulation()) {
      return inputs.turnAbsolutePosition;
    }
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * constants.WheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * constants.WheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module drive velocity in rad/sec. */
  public double getDriveFFCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }

  /** Returns the module drive acceleration in m/s², converted from rad/s² using wheel radius. */
  public double getDriveAccelerationMetersPerSecSquared() {
    return inputs.driveAccelerationRadPerSecSquared * constants.WheelRadius;
  }

  /** Returns the current drive acceleration state (velocity and heading) for chassis kinematics. */
  public SwerveModuleState getAccelerationState() {
    return new SwerveModuleState(getDriveAccelerationMetersPerSecSquared(), getAngle());
  }

  /** Returns the module steer velocity in rad/sec. */
  public double getSteerFFCharacterizationVelocity() {
    return inputs.turnVelocityRadPerSec;
  }
}
