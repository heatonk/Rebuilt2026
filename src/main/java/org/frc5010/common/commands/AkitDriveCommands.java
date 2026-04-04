// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.frc5010.common.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.drive.swerve.GenericSwerveDrivetrain;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;

/**
 * Factory class for creating swerve drive related commands including manual drive control,
 * characterization routines, and PID tuning commands for drive and steer motors.
 */
public class AkitDriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  private static final double PID_TUNING_VOLTAGE = 2.0; // Volts - voltage to apply during tuning
  private static final double PID_TUNING_DELAY = 1.0; // Secs - initial delay before measurements

  private AkitDriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   *
   * @param swerveDrive the swerve drivetrain subsystem to control
   * @param drive the swerve drive implementation
   * @param xSupplier supplier for the forward/backward joystick input
   * @param ySupplier supplier for the left/right joystick input
   * @param omegaSupplier supplier for the rotation joystick input
   * @return a command that continuously reads joystick inputs and drives the robot
   */
  public static Command joystickDrive(
      GenericSwerveDrivetrain swerveDrive,
      AkitSwerveDrive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        swerveDrive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   *
   * @param swerveDrive the swerve drivetrain subsystem to control
   * @param drive the swerve drive implementation
   * @param xSupplier supplier for the forward/backward joystick input
   * @param ySupplier supplier for the left/right joystick input
   * @param rotationSupplier supplier for the target rotation angle
   * @return a command that drives the robot while maintaining a target orientation
   */
  public static Command joystickDriveAtAngle(
      GenericSwerveDrivetrain swerveDrive,
      AkitSwerveDrive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            swerveDrive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   *
   * @param subsystem the swerve drivetrain subsystem to characterize
   * @param characterizer consumer that accepts voltage values to apply to drive motors
   * @param velocitySupplier supplier that returns the current velocity for measurement
   * @return a command that performs feedforward characterization and logs results
   */
  public static Command feedforwardCharacterization(
      GenericSubsystem subsystem,
      Consumer<Voltage> characterizer,
      Supplier<Double> velocitySupplier) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  characterizer.accept(Volts.of(0.0));
                },
                subsystem)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  characterizer.accept(Volts.of(voltage));
                  velocitySamples.add(velocitySupplier.get());
                  voltageSamples.add(voltage);
                },
                subsystem)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                  SmartDashboard.putNumber("Characterization/Feedforward/kS", kS);
                  SmartDashboard.putNumber("Characterization/Feedforward/kV", kV);
                }));
  }

  /**
   * Measures the robot's wheel radius by spinning in a circle.
   *
   * @param swerveDrive the swerve drivetrain subsystem to characterize
   * @param drive the swerve drive implementation
   * @return a command that measures wheel radius and logs the results to SmartDashboard
   */
  public static Command wheelRadiusCharacterization(
      GenericSwerveDrivetrain swerveDrive, AkitSwerveDrive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                swerveDrive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * drive.getConfig().DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                      SmartDashboard.putNumber("Characterization/Wheel Radius", wheelRadius);
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  /**
   * Measures the proportional gain (P value) of the drive motor PID controllers.
   *
   * <p>This command applies a range of target velocity setpoints to the drive motors and measures
   * the resulting velocity errors to calculate a more robust P value for closed-loop control.
   *
   * <p>The P value is calculated using: kP = Applied Voltage / Average Velocity Error across
   * multiple setpoints
   *
   * @param swerveDrive the swerve drivetrain subsystem to tune
   * @param drive the swerve drive implementation
   * @return a command that measures velocity errors across multiple setpoints and logs suggested kP
   *     values
   */
  public static Command drivePIDTuning(GenericSwerveDrivetrain swerveDrive, AkitSwerveDrive drive) {
    final double[] TARGET_VELOCITIES = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0}; // Range of velocities in m/s
    List<Double> allVelocityErrors = new LinkedList<>();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(allVelocityErrors::clear),

        // Allow modules to orient once
        Commands.run(() -> drive.runCharacterization(0.0), swerveDrive)
            .withTimeout(PID_TUNING_DELAY),

        // Test each target velocity
        Commands.sequence(
            // Create a sequence of commands, one for each target velocity
            java.util.Arrays.stream(TARGET_VELOCITIES)
                .boxed()
                .map(
                    targetVelocity ->
                        Commands.sequence(
                            // Transition to new setpoint
                            Commands.runOnce(
                                () -> {
                                  ChassisSpeeds speeds =
                                      new ChassisSpeeds(targetVelocity, 0.0, 0.0);
                                  drive.runVelocity(speeds);
                                }),

                            // Wait for module to settle
                            Commands.waitSeconds(0.5),

                            // Gather velocity error data
                            Commands.run(
                                    () -> {
                                      ChassisSpeeds speeds =
                                          new ChassisSpeeds(targetVelocity, 0.0, 0.0);
                                      drive.runVelocity(speeds);

                                      // Calculate velocity error: difference between setpoint
                                      // and actual velocity
                                      double avgVelocity = 0.0;
                                      for (int i = 0; i < 4; i++) {
                                        avgVelocity +=
                                            drive.getModulesInfo()[i]
                                                .driveVelocityMetersPerSecond();
                                      }
                                      avgVelocity /= 4.0;
                                      double error = Math.abs(targetVelocity - avgVelocity);
                                      allVelocityErrors.add(error);
                                    },
                                    swerveDrive)
                                .withTimeout(1.5)))
                .toArray(Command[]::new)),

        // When finished, calculate and print results
        Commands.runOnce(
            () -> {
              if (allVelocityErrors.isEmpty()) {
                System.out.println("No velocity error data collected.");
                return;
              }

              // Calculate average velocity error across all test points
              double avgError = 0.0;
              for (double error : allVelocityErrors) {
                avgError += error;
              }
              avgError /= allVelocityErrors.size();

              // Calculate P value: P = Voltage / Error
              // This represents the proportional gain needed to achieve the targets with
              // applied voltage
              double kP = avgError > 0.001 ? PID_TUNING_VOLTAGE / avgError : 0.0;

              NumberFormat formatter = new DecimalFormat("#0.00000");
              System.out.println("********** Drive PID P Value Tuning Results **********");
              System.out.println(
                  "\tTarget Velocities: " + java.util.Arrays.toString(TARGET_VELOCITIES) + " m/s");
              System.out.println("\tNumber of Samples: " + allVelocityErrors.size());
              System.out.println(
                  "\tAverage Velocity Error: " + formatter.format(avgError) + " m/s");
              System.out.println("\tSuggested kP: " + formatter.format(kP));
              System.out.println("*******************************************************");
              SmartDashboard.putNumber("Characterization/Drive/P_Value", kP);
              SmartDashboard.putNumber("Characterization/Drive/Avg_Velocity_Error", avgError);
              SmartDashboard.putNumber(
                  "Characterization/Drive/Sample_Count", allVelocityErrors.size());
            }));
  }

  /**
   * Measures the proportional gain (P value) of the steer motor PID controllers.
   *
   * <p>This command applies a range of target angle setpoints to the steer motors and measures the
   * resulting angle errors to calculate a more robust P value for closed-loop control.
   *
   * <p>The P value is calculated using: kP = Applied Voltage / Average Angle Error across multiple
   * setpoints
   *
   * @param swerveDrive the swerve drivetrain subsystem to tune
   * @param drive the swerve drive implementation
   * @return a command that measures angle errors across multiple setpoints and logs suggested kP
   *     values
   */
  public static Command steerPIDTuning(GenericSwerveDrivetrain swerveDrive, AkitSwerveDrive drive) {
    final double[] TARGET_ANGLES = {
      15.0, 30.0, 45.0, 60.0, 75.0, 90.0
    }; // Range of angles in degrees
    List<Double> allAngleErrors = new LinkedList<>();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(allAngleErrors::clear),

        // Test each target angle
        Commands.sequence(
            // Create a sequence of commands, one for each target angle
            java.util.Arrays.stream(TARGET_ANGLES)
                .boxed()
                .map(
                    targetDegrees ->
                        Commands.sequence(
                            // Transition to new setpoint
                            Commands.runOnce(
                                () -> {
                                  // Command all modules to the target angle
                                  ChassisSpeeds speeds =
                                      new ChassisSpeeds(0, 0, Math.toRadians(targetDegrees));

                                  drive.runVelocity(speeds);
                                }),

                            // Wait for modules to reach target angle
                            Commands.waitSeconds(1.0),

                            // Measure angle error after settling
                            Commands.runOnce(
                                () -> {
                                  double avgAngleError = 0.0;
                                  for (int i = 0; i < 4; i++) {
                                    double currentAngle =
                                        drive.getModulesInfo()[i].steerAbsoluteDegrees();
                                    double error = Math.abs(targetDegrees - currentAngle);
                                    // Handle angle wrapping (shortest path)
                                    if (error > 180.0) {
                                      error = 360.0 - error;
                                    }
                                    avgAngleError += error;
                                  }
                                  avgAngleError /= 4.0;
                                  avgAngleError = Math.toRadians(avgAngleError);
                                  allAngleErrors.add(avgAngleError);
                                })))
                .toArray(Command[]::new)),

        // When finished, calculate and print results
        Commands.runOnce(
            () -> {
              if (allAngleErrors.isEmpty()) {
                System.out.println("No angle error data collected.");
                return;
              }

              // Calculate average angle error across all test points
              double avgError = 0.0;
              for (double error : allAngleErrors) {
                avgError += error;
              }
              avgError /= allAngleErrors.size();

              // Calculate P value: P = Voltage / Error
              double kP = avgError > 0.001 ? PID_TUNING_VOLTAGE / avgError : 0.0;

              NumberFormat formatter = new DecimalFormat("#0.00000");
              System.out.println("********** Steer PID P Value Tuning Results **********");
              System.out.println(
                  "\tTarget Angles: " + java.util.Arrays.toString(TARGET_ANGLES) + " degrees");
              System.out.println("\tNumber of Samples: " + allAngleErrors.size());
              System.out.println(
                  "\tAverage Angle Error: "
                      + formatter.format(Math.toDegrees(avgError))
                      + " degrees");
              System.out.println("\tSuggested kP: " + formatter.format(kP));
              System.out.println("*******************************************************");
              SmartDashboard.putNumber("Characterization/Steer/P_Value", kP);
              SmartDashboard.putNumber(
                  "Characterization/Steer/Avg_Angle_Error", Math.toDegrees(avgError));
              SmartDashboard.putNumber(
                  "Characterization/Steer/Sample_Count", allAngleErrors.size());
            }));
  }
}
