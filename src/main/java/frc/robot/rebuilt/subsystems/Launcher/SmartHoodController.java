package frc.robot.rebuilt.subsystems.Launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

/**
 * Hood controller that uses TorqueCurrentFOC.
 */
public class SmartHoodController {

  public record HoodTarget(double positionMechRot) {}

  private static final double SIGNAL_UPDATE_FREQUENCY_HZ = 250.0;

  private final TalonFX talonFX;
  private final SmartHoodConfig config;

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<AngularAcceleration> accelerationSignal;
  private final StatusSignal<Current> torqueCurrentSignal;

  private final PositionTorqueCurrentFOC trackingRequest =
      new PositionTorqueCurrentFOC(0).withSlot(0);

  private final AtomicReference<HoodTarget> target =
      new AtomicReference<>(new HoodTarget(0));

  private volatile boolean enabled = false;
  private volatile boolean forceDisabled = false;

  public SmartHoodController(SmartHoodConfig config) {
    this.config = config;
    this.talonFX = config.getTalonFX();

    TalonFXConfiguration fxConfig = new TalonFXConfiguration();
    talonFX.getConfigurator().refresh(fxConfig);

    fxConfig.Slot0.kP = config.getKP();
    fxConfig.Slot0.kI = config.getKI();
    fxConfig.Slot0.kD = config.getKD();
    fxConfig.Slot0.kS = 0; // Handled externally via feedforward
    fxConfig.Slot0.kV = config.getKV();
    fxConfig.Slot0.kA = config.getKA();

    fxConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.getPeakTorqueCurrentAmps();
    fxConfig.TorqueCurrent.PeakReverseTorqueCurrent = -config.getPeakTorqueCurrentAmps();
    fxConfig.CurrentLimits.StatorCurrentLimitEnable = false;

    fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.getUpperLimitRotations();
    fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.getLowerLimitRotations();

    talonFX.getConfigurator().apply(fxConfig);

    positionSignal = talonFX.getPosition();
    velocitySignal = talonFX.getVelocity();
    accelerationSignal = talonFX.getAcceleration();
    torqueCurrentSignal = talonFX.getTorqueCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        SIGNAL_UPDATE_FREQUENCY_HZ,
        positionSignal,
        velocitySignal,
        accelerationSignal,
        torqueCurrentSignal);
    ParentDevice.optimizeBusUtilizationForAll(talonFX);

    if (config.getYamsController() != null) {
      config.getYamsController().stopClosedLoopController();
    }
  }

  public void setTarget(Angle position) {
    double positionMechRot = position.in(edu.wpi.first.units.Units.Rotations);
    positionMechRot =
        MathUtil.clamp(
            positionMechRot, config.getLowerLimitRotations(), config.getUpperLimitRotations());
    target.set(new HoodTarget(positionMechRot));
    enabled = true;

    Logger.recordOutput("SmartHood/GoalPosition", positionMechRot);
  }

  public void step(double dtSeconds) {
    if (!enabled || forceDisabled) {
      return;
    }

    HoodTarget currentTarget = target.get();
    double actualPositionMechRot = talonFX.getPosition().getValueAsDouble();
    double positionError = Math.abs(currentTarget.positionMechRot() - actualPositionMechRot);

    double signedError = currentTarget.positionMechRot() - actualPositionMechRot;
    
    // kS acts against friction, pushing towards target.
    double ksFeedforward;
    if (positionError < (0.25 / 360.0)) { // Small deadband 0.25 degrees
      ksFeedforward = 0.0;
    } else {
      ksFeedforward = Math.signum(signedError) * config.getKS();
    }

    // kG gravity feedforward, assumes horizontal is 0, so scales by cos(angle). 
    // If arm is measured such that 0 is horizontal:
    double angleRad = actualPositionMechRot * 2.0 * Math.PI;
    double kgFeedforward = config.getKG() * Math.cos(angleRad);
    
    double totalFeedforward = ksFeedforward + kgFeedforward;

    talonFX.setControl(
        trackingRequest
            .withPosition(currentTarget.positionMechRot())
            .withFeedForward(totalFeedforward));

    Logger.recordOutput("SmartHood/PositionErrorRot", positionError);
    Logger.recordOutput("SmartHood/ActualPositionMechRot", actualPositionMechRot);
    Logger.recordOutput("SmartHood/TargetPositionMechRot", currentTarget.positionMechRot());
  }

  public double getActualPositionMechRot() {
    return talonFX.getPosition().getValueAsDouble();
  }

  public double getGoalPositionMechRot() {
    return target.get().positionMechRot();
  }

  public void stop() {
    enabled = false;
    double actualMechRot = talonFX.getPosition().getValueAsDouble();
    target.set(new HoodTarget(actualMechRot));
  }

  public TalonFX getTalonFX() {
    return talonFX;
  }

  public SmartHoodConfig getConfig() {
    return config;
  }
}
