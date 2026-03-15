package frc.robot.rebuilt.subsystems.DriverDisplay;

import static edu.wpi.first.units.Units.Seconds;

import frc.robot.rebuilt.HubTracker;
import frc.robot.rebuilt.HubTracker.Shift;

public class HubStatusIOImpl implements HubStatusIO {

  @Override
  public void updateInputs(HubStatusIOInputs inputs) {
    inputs.activeNow = HubTracker.isActive();

    inputs.currentShift = HubTracker.getCurrentShift().orElse(Shift.AUTO);
    inputs.timeRemainingInCurrentShift =
        HubTracker.timeRemainingInCurrentShift().orElse(Seconds.of(0));
    inputs.nextShift = HubTracker.getNextShift().orElse(Shift.AUTO);
    inputs.isActiveNext = HubTracker.isActiveNext();
    inputs.autoWinner = HubTracker.getAutoWinner().map(it -> it.toString()).orElse("NA");
    inputs.matchTime = HubTracker.getMatchTime();
  }
}
