package frc.robot.rebuilt.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rebuilt.commands.IntakeCommands;
import org.frc5010.common.arch.GenericSubsystem;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public IntakeCommands.IntakeState stateRequested = IntakeCommands.IntakeState.UNKNOWN;
    public IntakeCommands.IntakeState stateCurrent = IntakeCommands.IntakeState.UNKNOWN;
    public double speed = 0.0;
    public Angle hopperAngleActual = Degrees.of(0.0);
    public double hopperAngleDegrees = 0.0;
    public double hopperVelocityDegreesPerSecond = 0.0;
    public double hopperAmps = 0;
    public boolean hopperMoving = false;
    public boolean hopperStalling = false;
    public boolean hopperHardStopDetected = false;

    public Angle hopperAngleDesired = Degrees.of(0);
    public double hopperAngleError = 0.0;
    public boolean hopperAtGoal = true;
    public boolean hopperZeroed = false;
    public int simulatedGamepieces = 0;
  }

  public void runSpintake(double speed);

  public void runSpintakes(double outerSpeed, double innerSpeed);

  public Command setHopperAngle(Angle angle);

  public void setHopperPosition(Angle angle);

  public boolean isHopperMoving();

  public boolean isRetracted();

  public boolean isDeployed();

  public boolean isHopperStalling();

  public boolean isHopperAtLocation(Angle location);

  public void runHopper(double speed);

  public boolean isNearTrench();

  public Command getHopperSysIdCommand();

  public Command getHopperCharacterizationCommand(GenericSubsystem intake);

  public default void updateInputs(IntakeIOInputs inputs) {}
}
