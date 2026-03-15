// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors;

import edu.wpi.first.wpilibj.util.Color;
import org.frc5010.common.motors.MotorConstants.Motor;
import org.frc5010.common.motors.function.DriveTrainMotor;
import org.frc5010.common.motors.function.FollowerMotor;
import org.frc5010.common.motors.hardware.GenericRevBrushlessMotor;
import org.frc5010.common.motors.hardware.GenericTalonFXMotor;
import org.frc5010.common.motors.hardware.GenericThriftyNovaMotor;

/** Add your docs here. */
public class MotorFactory {
  protected static int simEncoderPort = 10;
  protected static int visualColorIndex = 0;
  protected static Color[] visualColors =
      new Color[] {
        Color.kRed,
        Color.kOrange,
        Color.kYellow,
        Color.kGreen,
        Color.kBlue,
        Color.kPurple,
        Color.kCyan,
        Color.kMagenta,
        Color.kViolet,
        Color.kPink,
        Color.kWhite,
        Color.kBrown,
        Color.kDarkRed,
        Color.kDarkOrange,
        Color.kYellowGreen,
        Color.kDarkGreen,
        Color.kDarkBlue,
        Color.kDarkViolet,
        Color.kDarkCyan,
        Color.kDarkMagenta,
        Color.kDarkSalmon,
        Color.kGray
      };

  public static int getNextSimEncoderPort() {
    return simEncoderPort++;
  }

  public static Color getNextVisualColor() {
    if (visualColorIndex >= visualColors.length) {
      visualColorIndex = 0;
    }
    return visualColors[visualColorIndex++];
  }

  public static GenericMotorController Spark(int canId, Motor config) {
    switch (config) {
      case KrakenX44:
      case KrakenX60:
      case NeoVortex:
        throw new IllegalArgumentException("Sparks can not use " + config + " config");
      default:
    }
    return new GenericRevBrushlessMotor(canId, config, true);
  }

  public static GenericMotorController SparkFlex(int canId, Motor config) {
    switch (config) {
      case KrakenX44:
      case KrakenX60:
      case Neo550:
      case Neo:
        throw new IllegalArgumentException("Sparks can not use " + config + " config");
      default:
    }
    return new GenericRevBrushlessMotor(canId, config, false);
  }

  public static GenericMotorController Thrifty(int canId, Motor config) {
    switch (config) {
      case KrakenX44:
      case KrakenX60:
      case NeoVortex:
        throw new IllegalArgumentException("Thrifty can not use " + config + " config");
      default:
    }
    return new GenericThriftyNovaMotor(canId, config);
  }

  public static GenericMotorController TalonFX(int canId, Motor config) {
    switch (config) {
      case KrakenX60:
        return new GenericTalonFXMotor(canId, config);
      case KrakenX44:
        return new GenericTalonFXMotor(canId, config);
      case KrakenX60Foc:
        return new GenericTalonFXMotor(canId, config);
      default:
        throw new IllegalArgumentException("TalonFX can not use " + config + " config");
    }
  }

  public static GenericMotorController DriveTrainMotor(GenericMotorController motor, String name) {
    return new DriveTrainMotor(motor, name);
  }

  public static GenericMotorController FollowMotor(
      GenericMotorController motor, GenericMotorController leader) {
    return new FollowerMotor(motor, leader, "");
  }
}
