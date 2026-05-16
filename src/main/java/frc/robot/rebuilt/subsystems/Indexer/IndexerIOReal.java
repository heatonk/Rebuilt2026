package frc.robot.rebuilt.subsystems.Indexer;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.rebuilt.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.SmartMotorFactory;

/** Implements the hardware Indexer IO */
public class IndexerIOReal implements IndexerIO {
  protected TalonFX spindexer;
  protected FlyWheel transferFront;

  public IndexerIOReal(SubsystemBase parent) {
    spindexer = buildSpindexer();
    transferFront = buildTransfer(parent);
  }

  private static TalonFX buildSpindexer() {
    TalonFX motor = new TalonFX(Constants.Indexer.Spindexer.CAN_ID);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = Constants.Indexer.Spindexer.CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor.getConfigurator().apply(config);
    return motor;
  }

  private static FlyWheel buildTransfer(SubsystemBase parent) {
    TalonFX leader = new TalonFX(Constants.Indexer.Transfer.CAN_ID);
    TalonFX follower = new TalonFX(Constants.Indexer.Transfer.FOLLOWER_CAN_ID);
    DCMotor motorSim = DCMotor.getKrakenX60(2);

    @SuppressWarnings("unchecked")
    Pair<Object, Boolean>[] followers =
        new Pair[] {new Pair<>(follower, Constants.Indexer.Transfer.FOLLOWER_INVERTED)};

    SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(parent)
            .withFeedforward(
                new SimpleMotorFeedforward(
                    Constants.Indexer.Transfer.KS,
                    Constants.Indexer.Transfer.KV,
                    Constants.Indexer.Transfer.KA))
            .withSimFeedforward(
                new SimpleMotorFeedforward(
                    Constants.Indexer.Transfer.KS,
                    Constants.Indexer.Transfer.KV,
                    Constants.Indexer.Transfer.KA))
            .withClosedLoopController(
                Constants.Indexer.Transfer.KP,
                Constants.Indexer.Transfer.KI,
                Constants.Indexer.Transfer.KD)
            .withSimClosedLoopController(
                Constants.Indexer.Transfer.KP,
                Constants.Indexer.Transfer.KI,
                Constants.Indexer.Transfer.KD)
            .withGearing(
                new MechanismGearing(GearBox.fromStages(Constants.Indexer.Transfer.GEAR_STAGES)))
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("transferMotor", TelemetryVerbosity.LOW)
            .withMotorInverted(Constants.Indexer.Transfer.INVERTED)
            .withFollowers(followers);

    SmartMotorController smartMotor =
        SmartMotorFactory.create(leader, motorSim, motorConfig)
            .orElseThrow(() -> new RuntimeException("Failed to build transfer SmartMotorController"));

    Distance radius = Meters.of(Constants.Indexer.Transfer.RADIUS_M);
    Mass mass = Kilograms.of(Constants.Indexer.Transfer.MASS_KG);

    FlyWheelConfig transferConfig =
        new FlyWheelConfig(smartMotor)
            .withDiameter(radius.times(2.0))
            .withMass(mass)
            .withUpperSoftLimit(
                DegreesPerSecond.of(Constants.Indexer.Transfer.UPPER_SOFT_LIMIT_RPM * 6.0))
            .withLowerSoftLimit(
                DegreesPerSecond.of(Constants.Indexer.Transfer.LOWER_SOFT_LIMIT_RPM * 6.0))
            .withSpeedometerSimulation(
                DegreesPerSecond.of(Constants.Indexer.Transfer.UPPER_SOFT_LIMIT_RPM * 6.0))
            .withTelemetry("transfer", TelemetryVerbosity.LOW);
    transferConfig.withMOI(radius, mass);
    return new FlyWheel(transferConfig);
  }

  /** Updates indexer input values with current motor speed */
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.spindexerSpeed = spindexer.get();
    inputs.transferFrontSpeed = transferFront.getMotor().getDutyCycle();
  }

  /** Sets the spindexer motor speed */
  @Override
  public void runSpindexer(double speed) {
    spindexer.set(speed);
  }

  /** Sets the front transfer motor speed */
  @Override
  public void runTransferFront(double speed) {
    transferFront.getMotor().setDutyCycle(speed);
  }
}
