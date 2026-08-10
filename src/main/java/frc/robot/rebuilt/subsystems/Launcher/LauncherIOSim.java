// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.FieldConstants;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.commands.IndexerCommands.IndexerState;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import frc.robot.rebuilt.subsystems.intake.IntakeIOSim;
import org.littletonrobotics.junction.Logger;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceProjectile;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

/** Simulation IO layer for the Launcher. Extends the real layer for shared plumbing. */
public class LauncherIOSim extends LauncherIOReal {
  protected GamePieceProjectile gamePieceProjectile;

  private static final Translation3d FLYWHEEL_RELATIVE_POSITION =
      new Translation3d(
          Inches.of(Constants.Launcher.FlyWheel.ROBOT_TO_MOTOR_X_IN).in(Meters),
          Inches.of(Constants.Launcher.FlyWheel.ROBOT_TO_MOTOR_Y_IN).in(Meters),
          Inches.of(Constants.Launcher.FlyWheel.ROBOT_TO_MOTOR_Z_IN).in(Meters));

  public LauncherIOSim(SubsystemBase parent) {
    super(parent);
    if (IntakeIOSim.intakeSimulation != null) {
      IntakeIOSim.intakeSimulation.addGamePiecesToIntake(8);
    }
  }

  @Override
  public void configureShotCalculator(ShotCalculator shotCalculator) {
    super.configureShotCalculator(shotCalculator);
    double wheelRadiusMeters = Inches.of(Constants.Launcher.FlyWheel.RADIUS_INCHES).in(Meters);
    double minFlywheelRadPerSec =
        RPM.of(Constants.Launcher.FlyWheel.LOWER_SOFT_LIMIT_RPM).in(RadiansPerSecond);
    double maxFlywheelRadPerSec =
        RPM.of(Constants.Launcher.FlyWheel.UPPER_SOFT_LIMIT_RPM).in(RadiansPerSecond);
    Rotation2d minHoodAngle = Rotation2d.fromDegrees(Constants.Launcher.Hood.LOWER_SOFT_LIMIT_DEG);
    Rotation2d maxHoodAngle = Rotation2d.fromDegrees(Constants.Launcher.Hood.UPPER_SOFT_LIMIT_DEG);
    Rotation2d hoodStep = Rotation2d.fromDegrees(0.5);

    double launchHeight = FLYWHEEL_RELATIVE_POSITION.getZ();
    double targetHeight = FieldConstants.Hub.height;
    ShotCalculator.BallisticConfig config =
        new ShotCalculator.BallisticConfig(
            1.0,
            6.0,
            0.1,
            minHoodAngle,
            maxHoodAngle,
            hoodStep,
            minFlywheelRadPerSec,
            maxFlywheelRadPerSec,
            wheelRadiusMeters,
            launchHeight,
            targetHeight,
            0.0,
            9.80665,
            Math.toRadians(90.0));

    shotCalculator.setBallisticConfig(config);
    ShotCalculator.ShotTables simTables = ShotCalculator.createBallisticTables(config);
    shotCalculator.setShotTables(simTables);
    shotCalculator.setShuttleShotTables(ShotCalculator.copyShotTables(simTables));
  }

  @Override
  public void updateSimulation(Launcher launcher, Indexer indexer) {
    if (IntakeIOSim.intakeSimulation == null) {
      return;
    }
    int amount = IntakeIOSim.intakeSimulation.getGamePiecesAmount();
    if (Math.random() > 0.5 && amount > 0) {
      if ((indexer.isCurrent(IndexerState.FEED) && launcher.isShooting())
          || (indexer.isCurrent(IndexerState.FORCE))) {
        if (IntakeIOSim.intakeSimulation.obtainGamePieceFromIntake()) {
          Pose2d worldPose = Rebuilt.drivetrain.getPoseEstimator().getCurrentPose();
          gamePieceProjectile =
              new RebuiltFuelOnFly(
                      worldPose.getTranslation(),
                      FLYWHEEL_RELATIVE_POSITION.toTranslation2d(),
                      Rebuilt.drivetrain.getFieldVelocity(),
                      Rotation2d.fromDegrees(
                          worldPose
                              .getRotation()
                              .getMeasure()
                              .plus(getTurretAngle())
                              .in(Degrees)),
                      Meters.of(FLYWHEEL_RELATIVE_POSITION.getZ()),
                      getFlyWheelExitSpeed(getFlywheelSpeed()),
                      Degrees.of(90.0).minus(getHoodAngle()))
                  .withProjectileTrajectoryDisplayCallBack(
                      (pose3ds) -> {
                        Logger.recordOutput(
                            "Launcher/GamePieceTrajectory", pose3ds.toArray(Pose3d[]::new));
                      });
          SimulatedArena.getInstance().addGamePieceProjectile(gamePieceProjectile);
        }
      }
    }
  }
}
