package frc.robot.rebuilt.subsystems.DriverDisplay;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.rebuilt.util.Controller;
import org.littletonrobotics.junction.Logger;

public class HubStatus extends SubsystemBase {
  private HubStatusIO io = new HubStatusIOImpl();
  private HubStatusIOInputsAutoLogged inputs = new HubStatusIOInputsAutoLogged();

  public void configureButtonBindings(Controller driver, Controller operator) {
    // Trigger rumble = new Trigger(() -> inputs.timeRemainingInCurrentShift.lte(Seconds.of(3)));
    // rumble.and(() -> inputs.timeRemainingInCurrentShift.gt(Seconds.of(2.5)));
    // rumble
    //     .onTrue(Commands.runOnce(() -> driver.setRumble(0.5)))
    //     .onFalse(Commands.runOnce(() -> driver.setRumble(0)));
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);

    Logger.processInputs("HubStatus", inputs);
  }
}
