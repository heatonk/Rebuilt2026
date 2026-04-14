// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.swerve;

import org.frc5010.common.drive.swerve.akit.Module;
import swervelib.SwerveModule;

/**
 * Mutable container for swerve module telemetry — reused each cycle to avoid per-call allocation.
 * Use {@link #update(Module)} / {@link #update(SwerveModule)} to refresh values in-place.
 */
public class GenericSwerveModuleInfo {
  public double steerAbsoluteDegrees;
  public double steerRelativeDegrees;
  public double driveRelativePositionMeters;
  public double driveVelocityMetersPerSecond;
  public double steerVelocityDegreesPerSecond;
  public double expectedSteerDegrees;

  public GenericSwerveModuleInfo() {}

  /** Update all fields from a YAGSL {@link SwerveModule} — zero allocation after first call. */
  public void update(SwerveModule module) {
    steerAbsoluteDegrees = module.getAbsolutePosition();
    steerRelativeDegrees = module.getRelativePosition();
    driveRelativePositionMeters = module.getDriveMotor().getPosition();
    driveVelocityMetersPerSecond = module.getDriveMotor().getVelocity();
    steerVelocityDegreesPerSecond = module.getAngleMotor().getVelocity();
    expectedSteerDegrees = module.getState().angle.getDegrees();
  }

  /**
   * Update all fields from an AKit {@link Module} — zero allocation after first call. Reads {@code
   * inputs} values directly to avoid the allocating {@link Module#getPosition()} call.
   */
  public void update(Module module) {
    double angleDeg = module.getAngle().getDegrees();
    steerAbsoluteDegrees = angleDeg;
    steerRelativeDegrees = angleDeg;
    driveRelativePositionMeters = module.getPositionMeters();
    driveVelocityMetersPerSecond = module.getVelocityMetersPerSec();
    steerVelocityDegreesPerSecond = 0.0;
    expectedSteerDegrees = angleDeg;
  }
}
