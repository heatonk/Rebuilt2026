// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.rebuilt.FieldConstants;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.commands.IndexerCommands.IndexerState;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import frc.robot.rebuilt.subsystems.intake.IntakeIOSim;
import org.littletonrobotics.junction.Logger;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceProjectile;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

/** Add your docs here. */
public class LauncherIOSim extends LauncherIOReal {
  protected GamePieceProjectile gamePieceProjectile;

  public LauncherIOSim(SubsystemBase parent) {
    super(parent);
    if (IntakeIOSim.intakeSimulation != null) {
      IntakeIOSim.intakeSimulation.addGamePiecesToIntake(8);
    }
    // TODO: real swerve — maple-sim intake is a no-op without a real drivetrain sim.
  }

  @Override
  /** Configures the shot calculator and calculates measurements for parts of the lancher */
  public void configureShotCalculator(ShotCalculator shotCalculator) {
    super.configureShotCalculator(shotCalculator);
    double circumferenceMeters = flyWheel.getShooterConfig().getCircumference().in(Meters);
    double wheelRadiusMeters = circumferenceMeters / (2.0 * Math.PI);
    double minFlywheelRadPerSec =
        flyWheel.getShooterConfig().getLowerSoftLimit().orElse(RPM.of(0.0)).in(RadiansPerSecond);
    double maxFlywheelRadPerSec =
        flyWheel.getShooterConfig().getUpperSoftLimit().orElse(RPM.of(5000.0)).in(RadiansPerSecond);
    /** Reads the hood angle limits */
    Rotation2d minHoodAngle =
        Rotation2d.fromDegrees(
            hood.getMotorController().getConfig().getMechanismLowerLimit().get().in(Degrees));
    Rotation2d maxHoodAngle =
        Rotation2d.fromDegrees(
            hood.getMotorController().getConfig().getMechanismUpperLimit().get().in(Degrees));
    Rotation2d hoodStep = Rotation2d.fromDegrees(0.5);

    double launchHeight = flyWheel.getRelativeMechanismPosition().getZ();
    double targetHeight = FieldConstants.Hub.height;
    /** Creates ballistic configuration for the shot calculator */
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
    // Update simulated mechanism states here
    // We should simulate a shot rate of about 10-15 gamepieces per second
    // Every other time this is called, determine a randome number and if > 0.5, shoot a gamepiece.
    // This would mean we try to shoot 25 times per second, and on average shoot about 12-13
    // gamepieces per second.
    if (Math.random() > 0.5 && amount > 0) {
      if ((indexer.isCurrent(IndexerState.FEED) && launcher.isShooting())
          || (indexer.isCurrent(IndexerState.FORCE))) {
        if (IntakeIOSim.intakeSimulation.obtainGamePieceFromIntake()) {
          Pose2d worldPose = Rebuilt.drivetrain.getPoseEstimator().getCurrentPose();
          gamePieceProjectile =
              new RebuiltFuelOnFly(
                      worldPose.getTranslation(),
                      flyWheel.getRelativeMechanismPosition().toTranslation2d(),
                      Rebuilt.drivetrain.getFieldVelocity(),
                      Rotation2d.fromDegrees(
                          worldPose.getRotation().getMeasure().plus(turret.getAngle()).in(Degrees)),
                      flyWheel.getRelativeMechanismPosition().getMeasureZ(),
                      getFlyWheelExitSpeed(flyWheel.getSpeed()),
                      Degrees.of(90.0).minus(hood.getAngle()))
                  .withProjectileTrajectoryDisplayCallBack(
                      (pose3ds) -> {
                        Logger.recordOutput(
                            "Launcher/GamePieceTrajectory", pose3ds.toArray(Pose3d[]::new));
                      });
          SimulatedArena.getInstance().addGamePieceProjectile(gamePieceProjectile);
          // Create a new gamepiece on-the-fly and add it to the field simulation
        }
      }
    }
  }
}
