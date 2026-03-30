package frc.robot.rebuilt.subsystems.DriverDisplay;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// added import
import frc.robot.rebuilt.HubTracker.Shift;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.sensors.Controller;
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

  public void configureButtonBindings(Controller driver, Controller operator) {
    Trigger rumble = new Trigger(() -> timeRemainingInCurrentShift.lte(Seconds.of(3)));
    rumble
        .and(new Trigger(() -> timeRemainingInCurrentShift.gte(Seconds.of(2.5))))
        .whileTrue(Commands.runOnce(() -> driver.setRumble(0.5)))
        .onFalse(Commands.runOnce(() -> driver.setRumble(0)));
  }
}
