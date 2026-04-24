package frc.robot.rebuilt.subsystems.Launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
 * Flywheel controller that uses TorqueCurrentFOC.
 */
public class SmartFlywheelController {

  public record FlywheelTarget(double velocityRadPerSec) {}

  private static final double SIGNAL_UPDATE_FREQUENCY_HZ = 250.0;

  private final TalonFX talonFX;
  private final SmartFlywheelConfig config;
  private final String name;

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<AngularAcceleration> accelerationSignal;
  private final StatusSignal<Current> torqueCurrentSignal;

  private final VelocityTorqueCurrentFOC trackingRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);

  private final AtomicReference<FlywheelTarget> target =
      new AtomicReference<>(new FlywheelTarget(0));

  private volatile boolean enabled = false;
  private volatile boolean forceDisabled = false;

  public SmartFlywheelController(SmartFlywheelConfig config, String name) {
    this.config = config;
    this.talonFX = config.getTalonFX();
    this.name = name;

    TalonFXConfiguration fxConfig = new TalonFXConfiguration();
    talonFX.getConfigurator().refresh(fxConfig);

    fxConfig.Slot0.kP = config.getKP();
    fxConfig.Slot0.kI = config.getKI();
    fxConfig.Slot0.kD = config.getKD();
    fxConfig.Slot0.kS = 0; // kS is injected via Feedforward in step() based on velocity sign
    fxConfig.Slot0.kV = config.getKV();
    fxConfig.Slot0.kA = config.getKA();

    fxConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.getPeakTorqueCurrentAmps();
    fxConfig.TorqueCurrent.PeakReverseTorqueCurrent = -config.getPeakTorqueCurrentAmps();
    fxConfig.CurrentLimits.StatorCurrentLimitEnable = false;

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

  public void setTarget(AngularVelocity velocity) {
    double velocityRadPerSec = velocity.in(edu.wpi.first.units.Units.RadiansPerSecond);
    target.set(new FlywheelTarget(velocityRadPerSec));
    enabled = true;

    Logger.recordOutput("SmartFlywheel_" + name + "/GoalVelocityRadPerSec", velocityRadPerSec);
  }

  public void step(double dtSeconds) {
    if (!enabled || forceDisabled) {
      return;
    }

    FlywheelTarget currentTarget = target.get();
    double actualVelocityRadPerSec = getActualVelocityRadPerSec();
    double velocityError = currentTarget.velocityRadPerSec() - actualVelocityRadPerSec;

    // We can inject kS here to avoid chattering
    double ksFeedforward;
    if (Math.abs(currentTarget.velocityRadPerSec()) < 0.1) {
      ksFeedforward = 0.0;
    } else {
      ksFeedforward = Math.signum(currentTarget.velocityRadPerSec()) * config.getKS();
    }

    double velocityRotPerSec = currentTarget.velocityRadPerSec() / (2.0 * Math.PI);
    
    talonFX.setControl(
        trackingRequest
            .withVelocity(velocityRotPerSec)
            .withFeedForward(ksFeedforward));

    Logger.recordOutput("SmartFlywheel_" + name + "/VelocityErrorRadPerSec", velocityError);
    Logger.recordOutput("SmartFlywheel_" + name + "/ActualVelocityRadPerSec", actualVelocityRadPerSec);
    Logger.recordOutput("SmartFlywheel_" + name + "/TargetVelocityRadPerSec", currentTarget.velocityRadPerSec());
  }

  public double getActualVelocityRadPerSec() {
    return talonFX.getVelocity().getValueAsDouble() * 2.0 * Math.PI;
  }

  public double getGoalVelocityRadPerSec() {
    return target.get().velocityRadPerSec();
  }

  public void stop() {
    enabled = false;
    target.set(new FlywheelTarget(0));
    talonFX.setControl(new com.ctre.phoenix6.controls.NeutralOut());
  }

  public TalonFX getTalonFX() {
    return talonFX;
  }

  public SmartFlywheelConfig getConfig() {
    return config;
  }
}
