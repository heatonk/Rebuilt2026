package frc.robot.rebuilt.subsystems.Climb;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import frc.robot.rebuilt.commands.ClimbCommands;
import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Launcher subsystem. */
public interface ClimbIO {

  @AutoLog
  public static class ClimbIOInputs {
    public Distance climbHeight = Inches.of(0);
    public ClimbCommands.ClimbState stateRequested = ClimbCommands.ClimbState.DISABLED;
    public ClimbCommands.ClimbState stateCurrent = ClimbCommands.ClimbState.DISABLED;
  }

  public void runClimb(double speed);

  public void idle();

  public void setHeight(Distance height);

  public default void updateInputs(ClimbIOInputs inputs) {}
}
