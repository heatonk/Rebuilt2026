package frc.robot.rebuilt.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;

import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.subsystems.drive.StubDrivetrain;
import java.util.Map;
import java.util.Optional;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

/** Simulates the implimentation of IntakeIO */
public class IntakeIOSim extends IntakeIOReal {
  public static IntakeSimulation intakeSimulation;
  private AbstractDriveTrainSimulation driveTrainSimulation;
  private GamePieceOnFieldSimulation gamePiece;
  /** Initializes the mapleSim intake simulation */
  public IntakeIOSim(Map<String, Object> devices) {
    super(devices);
    Optional<AbstractDriveTrainSimulation> driveSim = StubDrivetrain.getMapleSimDrive();
    if (driveSim.isPresent()) {
      driveTrainSimulation = driveSim.get();
      intakeSimulation =
          IntakeSimulation.OverTheBumperIntake(
              "Fuel",
              driveTrainSimulation,
              Inches.of(27.25),
              Inches.of(11.25),
              IntakeSimulation.IntakeSide.FRONT,
              80);
    }
    // TODO: real swerve — without a maple-sim drivetrain, intake sim is a no-op.
  }
  /** Runs the intake motor and updates the state of the intake simulation */
  @Override
  public void runSpintake(double speed) {
    super.runSpintake(speed);
    if (intakeSimulation == null) {
      return;
    }
    if (speed > 0) {
      intakeSimulation.startIntake();
    } else {
      intakeSimulation.stopIntake();
    }
  }
  /** manages simulated collection of game pieces and updates intake inputs */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    super.updateInputs(inputs);
    if (intakeSimulation == null) {
      return;
    }
    if (inputs.speed < 0) {
      if (intakeSimulation.obtainGamePieceFromIntake()) {
        gamePiece =
            new RebuiltFuelOnField(
                Rebuilt.drivetrain.getPoseEstimator().getCurrentPose().getTranslation());
        SimulatedArena.getInstance().addGamePiece(gamePiece);
      }
    }
    inputs.simulatedGamepieces = intakeSimulation.getGamePiecesAmount();
  }
}
