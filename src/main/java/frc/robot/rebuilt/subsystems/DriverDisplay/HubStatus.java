package frc.robot.rebuilt.subsystems.DriverDisplay;

import edu.wpi.first.units.measure.Time;
// added import
import frc.robot.rebuilt.HubTracker.Shift;
import org.frc5010.common.arch.GenericSubsystem;
import org.littletonrobotics.junction.Logger;

public class HubStatus extends GenericSubsystem {
  private HubStatusIO io = new HubStatusIOImpl();
  private HubStatusIOInputsAutoLogged inputs = new HubStatusIOInputsAutoLogged();
  Shift currentShift;
  Time timeRemainingInCurrentShift;
  Shift nextShift;
  boolean isActiveNext;
  String autoWinner;
  double matchTime;

  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);

    Logger.processInputs("HubStatus", inputs);
  }
}
