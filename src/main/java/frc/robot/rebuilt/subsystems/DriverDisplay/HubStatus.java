package frc.robot.rebuilt.subsystems.DriverDisplay;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.sensors.Controller;
import org.littletonrobotics.junction.Logger;

public class HubStatus extends GenericSubsystem {
  private HubStatusIO io = new HubStatusIOImpl();
  private HubStatusIOInputsAutoLogged inputs = new HubStatusIOInputsAutoLogged();

  public void configureButtonBindings(Controller driver, Controller operator) {
    Trigger rumble = new Trigger(() -> inputs.timeRemainingInCurrentShift.lte(Seconds.of(3)));
    rumble.and(() -> inputs.timeRemainingInCurrentShift.gt(Seconds.of(2.5)));
    rumble
        .onTrue(Commands.runOnce(() -> driver.setRumble(0.5)))
        .onFalse(Commands.runOnce(() -> driver.setRumble(0)));
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);

    Logger.processInputs("HubStatus", inputs);
  }
}
