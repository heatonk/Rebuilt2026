// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.sensors.encoder.GenericEncoder;
import yams.motorcontrollers.SmartMotorController;

/** Add your docs here. */
public class SystemIdentification {
  /** Tracks the voltage being applied to a motor */
  private static final MutVoltage m_appliedVoltage = new MutVoltage(0, 0, Volts);
  /** Tracks the distance travelled of a position motor */
  private static final MutAngle m_distance = new MutAngle(0, 0, Rotations);
  /** Tracks the velocity of a positional motor */
  private static final MutAngularVelocity m_velocity =
      new MutAngularVelocity(0, 9, RotationsPerSecond);
  /** Tracks the rotations of an angular motor */
  private static final MutAngle m_anglePosition = new MutAngle(0, 0, Degrees);
  /** Tracks the velocity of an angular motor */
  private static final MutAngularVelocity m_angVelocity =
      new MutAngularVelocity(0, 0, DegreesPerSecond);

  public static SysIdRoutine rpmSysIdRoutine(
      GenericMotorController motor,
      GenericEncoder encoder,
      String motorName,
      SubsystemBase subsystemBase) {

    return new SysIdRoutine(
        new Config(Volts.of(1).div(Seconds.of(1)), Volts.of(1), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            (Voltage voltage) -> motor.set(voltage.in(Volts) / RobotController.getBatteryVoltage()),
            log -> {
              log.motor(motorName)
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          motor.get() * RobotController.getBatteryVoltage(), Volts))
                  .angularPosition(m_distance.mut_replace(encoder.getPosition(), Rotations))
                  .angularVelocity(
                      m_velocity.mut_replace(encoder.getVelocity(), RotationsPerSecond));
            },
            subsystemBase));
  }

  public static SysIdRoutine rpmSysIdRoutine(
      SmartMotorController motor, String motorName, SubsystemBase subsystemBase) {

    return new SysIdRoutine(
        new Config(Volts.of(1).div(Seconds.of(1)), Volts.of(4), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            (Voltage voltage) ->
                motor.setDutyCycle(voltage.in(Volts) / RobotController.getBatteryVoltage()),
            log -> {
              motor.updateTelemetry();
              motor.simIterate();
              log.motor(motorName)
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          motor.getDutyCycle() * RobotController.getBatteryVoltage(), Volts))
                  .angularPosition(m_distance.mut_replace(motor.getMechanismPosition()))
                  .angularVelocity(m_velocity.mut_replace(motor.getMechanismVelocity()));
            },
            subsystemBase));
  }

  public static SysIdRoutine angleSysIdRoutine(
      SmartMotorController motor, String motorName, SubsystemBase subsystemBase) {

    return new SysIdRoutine(
        new Config(Volts.of(1).div(Seconds.of(1)), Volts.of(4), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            (Voltage voltage) ->
                motor.setDutyCycle(voltage.in(Volts) / RobotController.getBatteryVoltage()),
            log -> {
              motor.updateTelemetry();
              motor.simIterate();
              log.motor(motorName)
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          motor.getDutyCycle() * RobotController.getBatteryVoltage(), Volts))
                  .angularPosition(m_distance.mut_replace(motor.getMechanismPosition()))
                  .angularVelocity(m_velocity.mut_replace(motor.getMechanismVelocity()));
            },
            subsystemBase));
  }

  public static SysIdRoutine angleSysIdRoutine(
      GenericMotorController motor,
      GenericEncoder encoder,
      String motorName,
      SubsystemBase subsystemBase) {

    return new SysIdRoutine(
        new Config(),
        new SysIdRoutine.Mechanism(
            (Voltage voltage) -> motor.set(voltage.in(Volts) / RobotController.getBatteryVoltage()),
            log -> {
              log.motor(motorName)
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          motor.get() * RobotController.getBatteryVoltage(), Volts))
                  .angularPosition(m_anglePosition.mut_replace(encoder.getPosition(), Degrees))
                  .angularVelocity(
                      m_angVelocity.mut_replace(encoder.getVelocity(), DegreesPerSecond));
            },
            subsystemBase));
  }

  public static Command getSysIdQuasistatic(
      SysIdRoutine routine, SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public static Command getSysIdQuasistaticForward(SysIdRoutine routine) {
    return getSysIdQuasistatic(routine, SysIdRoutine.Direction.kForward);
  }

  public static Command getSysIdQuasistaticBackward(SysIdRoutine routine) {
    return getSysIdQuasistatic(routine, SysIdRoutine.Direction.kReverse);
  }

  public static Command getSysIdDynamic(SysIdRoutine routine, SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  public static Command getSysIdDynamicForward(SysIdRoutine routine) {
    return getSysIdDynamic(routine, SysIdRoutine.Direction.kForward);
  }

  public static Command getSysIdDynamicBackward(SysIdRoutine routine) {
    return getSysIdDynamic(routine, SysIdRoutine.Direction.kReverse);
  }

  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  /**
   * Measures the velocity feedforward constants for the motors.
   *
   * <p>This command should only be used in voltage control mode.
   *
   * @param subsystem the swerve drivetrain subsystem to characterize
   * @param characterizer consumer that accepts voltage values to apply to motors
   * @param velocitySupplier supplier that returns the current velocity for measurement
   * @return a command that performs feedforward characterization and logs results
   */
  public static Command feedforwardCharacterization(
      GenericSubsystem subsystem,
      Consumer<Voltage> characterizer,
      Supplier<Double> velocitySupplier) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    List<Double> timeSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
              timeSamples.clear();
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
                  timeSamples.add(timer.get());
                },
                subsystem)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  double[] coefficients =
                      computeFeedforwardCoefficients(velocitySamples, voltageSamples, timeSamples);
                  double kS = coefficients[0];
                  double kV = coefficients[1];
                  double kA = coefficients[2];

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                  System.out.println("\tkA: " + formatter.format(kA));
                  SmartDashboard.putNumber("Characterization/Feedforward/kS", kS);
                  SmartDashboard.putNumber("Characterization/Feedforward/kV", kV);
                  SmartDashboard.putNumber("Characterization/Feedforward/kA", kA);
                }));
  }

  static double[] computeFeedforwardCoefficients(
      List<Double> velocitySamples, List<Double> voltageSamples, List<Double> timeSamples) {
    int n = Math.min(velocitySamples.size(), voltageSamples.size());
    if (n == 0) {
      return new double[] {0.0, 0.0, 0.0};
    }

    double sumX = 0.0;
    double sumY = 0.0;
    double sumXY = 0.0;
    double sumX2 = 0.0;
    for (int i = 0; i < n; i++) {
      double velocity = velocitySamples.get(i);
      double voltage = voltageSamples.get(i);
      sumX += velocity;
      sumY += voltage;
      sumXY += velocity * voltage;
      sumX2 += velocity * velocity;
    }
    double denom = (n * sumX2 - sumX * sumX);
    double kS = denom != 0.0 ? (sumY * sumX2 - sumX * sumXY) / denom : 0.0;
    double kV = denom != 0.0 ? (n * sumXY - sumX * sumY) / denom : 0.0;
    double kA = 0.0;

    if (timeSamples == null || timeSamples.size() < n) {
      return new double[] {kS, kV, kA};
    }

    int m = 0;
    double s00 = 0.0;
    double s01 = 0.0;
    double s02 = 0.0;
    double s11 = 0.0;
    double s12 = 0.0;
    double s22 = 0.0;
    double t0 = 0.0;
    double t1 = 0.0;
    double t2 = 0.0;
    for (int i = 1; i < n; i++) {
      double dt = timeSamples.get(i) - timeSamples.get(i - 1);
      if (dt <= 1e-6) {
        continue;
      }
      double velocity = velocitySamples.get(i);
      double accel = (velocity - velocitySamples.get(i - 1)) / dt;
      double voltage = voltageSamples.get(i);
      s00 += 1.0;
      s01 += velocity;
      s02 += accel;
      s11 += velocity * velocity;
      s12 += velocity * accel;
      s22 += accel * accel;
      t0 += voltage;
      t1 += velocity * voltage;
      t2 += accel * voltage;
      m++;
    }

    if (m >= 3) {
      double[][] a = {
        {s00, s01, s02},
        {s01, s11, s12},
        {s02, s12, s22}
      };
      double[] b = {t0, t1, t2};
      double[] solution = solve3x3(a, b);
      if (solution != null) {
        kS = solution[0];
        kV = solution[1];
        kA = solution[2];
      }
    }

    return new double[] {kS, kV, kA};
  }

  private static double[] solve3x3(double[][] a, double[] b) {
    double[][] m = {
      {a[0][0], a[0][1], a[0][2], b[0]},
      {a[1][0], a[1][1], a[1][2], b[1]},
      {a[2][0], a[2][1], a[2][2], b[2]}
    };

    for (int i = 0; i < 3; i++) {
      int pivot = i;
      for (int r = i + 1; r < 3; r++) {
        if (Math.abs(m[r][i]) > Math.abs(m[pivot][i])) {
          pivot = r;
        }
      }
      if (Math.abs(m[pivot][i]) < 1e-9) {
        return null;
      }
      if (pivot != i) {
        double[] temp = m[i];
        m[i] = m[pivot];
        m[pivot] = temp;
      }

      double divisor = m[i][i];
      for (int c = i; c < 4; c++) {
        m[i][c] /= divisor;
      }

      for (int r = 0; r < 3; r++) {
        if (r == i) {
          continue;
        }
        double factor = m[r][i];
        for (int c = i; c < 4; c++) {
          m[r][c] -= factor * m[i][c];
        }
      }
    }

    return new double[] {m[0][3], m[1][3], m[2][3]};
  }

  public static Command getSysIdFullCommand(
      SysIdRoutine routine, double quasistaticTimeout, double dynamicTimeout, double delay) {
    return getSysIdQuasistaticForward(routine)
        .withTimeout(quasistaticTimeout)
        .andThen(Commands.waitSeconds(delay))
        .andThen(getSysIdQuasistaticBackward(routine).withTimeout(quasistaticTimeout))
        .andThen(Commands.waitSeconds(delay))
        .andThen(getSysIdDynamicForward(routine).withTimeout(dynamicTimeout))
        .andThen(Commands.waitSeconds(delay))
        .andThen(getSysIdDynamicBackward(routine).withTimeout(dynamicTimeout));
  }

  public static Command getSysIdFullCommand(
      SysIdRoutine routine,
      double quasistaticTimeout,
      double dynamicTimeout,
      double delay,
      BooleanSupplier isAtMax,
      BooleanSupplier isAtMin,
      Runnable stopMotor) {
    return getSysIdQuasistaticForward(routine)
        .until(isAtMax)
        .finallyDo(stopMotor)
        .withTimeout(quasistaticTimeout)
        .andThen(Commands.waitSeconds(delay))
        .andThen(
            getSysIdQuasistaticBackward(routine)
                .until(isAtMin)
                .finallyDo(stopMotor)
                .withTimeout(quasistaticTimeout))
        .andThen(Commands.waitSeconds(delay))
        .andThen(
            getSysIdDynamicForward(routine)
                .until(isAtMax)
                .finallyDo(stopMotor)
                .withTimeout(dynamicTimeout))
        .andThen(Commands.waitSeconds(delay))
        .andThen(
            getSysIdDynamicBackward(routine)
                .until(isAtMin)
                .finallyDo(stopMotor)
                .withTimeout(dynamicTimeout));
  }
}
