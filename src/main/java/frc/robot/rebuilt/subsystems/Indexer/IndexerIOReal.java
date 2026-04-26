package frc.robot.rebuilt.subsystems.Indexer;

import static edu.wpi.first.units.Units.Amps;

import java.util.Map;
import org.frc5010.common.motors.function.PercentControlMotor;
import yams.mechanisms.velocity.FlyWheel;

/** Implements the hardware Indexer IO */
public class IndexerIOReal implements IndexerIO {
  protected Map<String, Object> devices;
  private PercentControlMotor spindexer;
  // private PercentControlMotor transferFront, transferBack;
  private FlyWheel transferFront;

  public IndexerIOReal(Map<String, Object> devices) {
    spindexer = (PercentControlMotor) devices.get("spindexer");
    transferFront = (FlyWheel) devices.get("transfer");
    spindexer.setCurrentLimit(Amps.of(120));
    // transferFront = (PercentControlMotor) devicess.get("transfer_front");
    // transferBack = (PercentControlMotor) devices.get("transfer_back");
    // transferFront.invert(true);
    // transferFront.setFollow(transferBack, false);
    this.devices = devices;
  }
  /** Updates indexer input values with current motor speed */
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.spindexerSpeed = spindexer.get();
    inputs.transferFrontSpeed = transferFront.getMotor().getDutyCycle();
    // inputs.transferFrontSpeed = transferFront.get();
    // inputs.transferBackSpeed = transferBack.get();
  }
  /** Sets the spindexer motor speed */
  @Override
  public void runSpindexer(double speed) {
    spindexer.set(speed);
  }
  /** Sets the front transfer motor speed */
  @Override
  public void runTransferFront(double speed) {
    // transferFront.set(speed);
    transferFront.getMotor().setDutyCycle(speed);
  }

  // @Override
  // public void runTransferBack(double speed) {
  //   transferBack.set(speed);
  // }
}
