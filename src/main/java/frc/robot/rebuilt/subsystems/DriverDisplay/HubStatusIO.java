package frc.robot.rebuilt.subsystems.DriverDisplay;

import edu.wpi.first.units.measure.Time;
import frc.robot.rebuilt.HubTracker.Shift;
import org.littletonrobotics.junction.AutoLog;

public interface HubStatusIO {
  @AutoLog
  public static class HubStatusIOInputs {
    public boolean activeNow = false;
    Shift currentShift;
    Time timeRemainingInCurrentShift;
    Shift nextShift;
    boolean isActiveNext;
    String autoWinner;
    double matchTime;
  }

  public void updateInputs(HubStatusIOInputs Inputs);
}
