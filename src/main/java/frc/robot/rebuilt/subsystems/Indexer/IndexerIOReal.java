package frc.robot.rebuilt.subsystems.Indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.rebuilt.Constants;

/** Implements the hardware Indexer IO. */
public class IndexerIOReal implements IndexerIO {
  protected TalonFX spindexer;
  protected TalonFX transferLeader;
  protected TalonFX transferFollower;

  private final DutyCycleOut spindexerDutyCycle = new DutyCycleOut(0);
  private final DutyCycleOut transferDutyCycle = new DutyCycleOut(0);

  public IndexerIOReal(SubsystemBase parent) {
    spindexer = buildSpindexer();
    transferLeader = buildTransferLeader();
    transferFollower = buildTransferFollower(transferLeader);
  }

  private static TalonFX buildSpindexer() {
    TalonFX motor = new TalonFX(Constants.Indexer.Spindexer.CAN_ID);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = Constants.Indexer.Spindexer.CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor.getConfigurator().apply(config);
    return motor;
  }

  private static TalonFX buildTransferLeader() {
    TalonFX motor = new TalonFX(Constants.Indexer.Transfer.CAN_ID);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        Constants.Indexer.Transfer.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    motor.getConfigurator().apply(config);
    return motor;
  }

  private static TalonFX buildTransferFollower(TalonFX leader) {
    TalonFX motor = new TalonFX(Constants.Indexer.Transfer.FOLLOWER_CAN_ID);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor.getConfigurator().apply(config);
    motor.setControl(
        new Follower(
            leader.getDeviceID(),
            Constants.Indexer.Transfer.FOLLOWER_INVERTED
                ? MotorAlignmentValue.Opposed
                : MotorAlignmentValue.Aligned));
    return motor;
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.spindexerSpeed = spindexer.getDutyCycle().getValueAsDouble();
    inputs.transferFrontSpeed = transferLeader.getDutyCycle().getValueAsDouble();
  }

  @Override
  public void runSpindexer(double speed) {
    spindexer.setControl(spindexerDutyCycle.withOutput(speed));
  }

  @Override
  public void runTransferFront(double speed) {
    transferLeader.setControl(transferDutyCycle.withOutput(speed));
  }
}
