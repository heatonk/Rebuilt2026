package frc.robot.rebuilt.subsystems.Launcher;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

/**
 * 2-state turret controller that switches between SEEKING and TRACKING modes.
 *
 * <p><b>SEEKING</b>: Uses {@link MotionMagicTorqueCurrentFOC} (TalonFX onboard profiling) for safe,
 * profiled travel over large distances. Prevents overshoot and respects motion constraints.
 *
 * <p><b>TRACKING</b>: Uses {@link PositionTorqueCurrentFOC} with explicit velocity and acceleration
 * feedforward to smoothly track a moving target at close range.
 *
 * <p>State transitions use hysteresis to prevent jitter:
 *
 * <ul>
 *   <li>SEEKING → TRACKING when |error| &lt; seekingThreshold
 *   <li>TRACKING → SEEKING when |error| &gt; seekingThreshold + hysteresisBuffer
 * </ul>
 *
 * <p>The {@link #step(double)} method is called at 200 Hz via Notifier, while {@link #setTarget} is
 * called from the 20 ms robot loop. Thread safety is ensured via {@link AtomicReference} over an
 * immutable {@link TurretTarget} record.
 */
public class SmartTurretController {

  /** The two operational states of the turret controller. */
  public enum TurretState {
    SEEKING,
    TRACKING
  }

  /** Immutable target data published from the 20 ms loop and consumed by the 200 Hz step loop. */
  public record TurretTarget(
      double positionMechRot, double velocityRadPerSec, double accelerationRadPerSecSq) {}

  private final TalonFX talonFX;
  private final SmartTurretConfig config;

  // Pre-allocated control requests (reused each cycle to avoid allocation).
  private final MotionMagicTorqueCurrentFOC seekingRequest =
      new MotionMagicTorqueCurrentFOC(0).withSlot(0);
  private final PositionTorqueCurrentFOC trackingRequest =
      new PositionTorqueCurrentFOC(0).withSlot(1);

  // Thread-safe target from 20 ms loop to 200 Hz step loop.
  private final AtomicReference<TurretTarget> target =
      new AtomicReference<>(new TurretTarget(0, 0, 0));

  private volatile TurretState currentState = TurretState.SEEKING;
  private volatile boolean enabled = false;

  /**
   * Constructs a SmartTurretController and configures the TalonFX with two PID slots, MotionMagic
   * parameters, and torque current limits.
   *
   * <p>Uses read-modify-write on the TalonFX configuration to preserve YAMS-set fields (inversion,
   * neutral mode, sensor ratio, etc.).
   */
  public SmartTurretController(SmartTurretConfig config) {
    this.config = config;
    this.talonFX = config.getTalonFX();

    // Read existing config to preserve YAMS-applied settings.
    TalonFXConfiguration fxConfig = new TalonFXConfiguration();
    talonFX.getConfigurator().refresh(fxConfig);

    // Slot0: Seeking (MotionMagicTorqueCurrentFOC)
    fxConfig.Slot0.kP = config.getSeekingKP();
    fxConfig.Slot0.kI = config.getSeekingKI();
    fxConfig.Slot0.kD = config.getSeekingKD();
    fxConfig.Slot0.kS = config.getKS();
    fxConfig.Slot0.kV = config.getKV();
    fxConfig.Slot0.kA = config.getKA();

    // Slot1: Tracking (PositionTorqueCurrentFOC)
    // kV and kA are set to 0 in the slot — we supply these externally via withFeedForward()
    // to avoid double-counting since the TalonFX's internal kV/kA multiply the reference
    // derivative which is zero for a static position command.
    fxConfig.Slot1.kP = config.getTrackingKP();
    fxConfig.Slot1.kI = config.getTrackingKI();
    fxConfig.Slot1.kD = config.getTrackingKD();
    fxConfig.Slot1.kS = 0;
    fxConfig.Slot1.kV = 0;
    fxConfig.Slot1.kA = 0;

    // MotionMagic cruise velocity and acceleration (mechanism rot/s and rot/s^2).
    fxConfig.MotionMagic.MotionMagicCruiseVelocity = config.getMaxVelocityMechRotPerSec();
    fxConfig.MotionMagic.MotionMagicAcceleration = config.getMaxAccelMechRotPerSecSq();
    fxConfig.MotionMagic.MotionMagicJerk = 0; // Unlimited jerk (no S-curve).

    // Torque current peak limits.
    fxConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.getPeakTorqueCurrentAmps();
    fxConfig.TorqueCurrent.PeakReverseTorqueCurrent = -config.getPeakTorqueCurrentAmps();

    // Firmware-level soft limits — last line of defense against hard stop contact.
    fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.getUpperLimitRotations();
    fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.getLowerLimitRotations();

    talonFX.getConfigurator().apply(fxConfig);

    // Stop the YAMS SmartMotorController's background closed-loop Notifier. YAMS runs its own
    // 20ms update loop that continuously sends position setpoints to the TalonFX. Since
    // SmartTurretController now owns this motor via direct TorqueCurrentFOC commands, that loop
    // must be permanently disabled to prevent interference.
    if (config.getYamsController() != null) {
      config.getYamsController().stopClosedLoopController();
    }
  }

  /**
   * Sets the turret target from the 20 ms robot loop.
   *
   * @param position desired turret mechanism angle
   * @param velocityRadPerSec angular velocity feedforward in rad/s (mechanism units)
   * @param accelerationRadPerSecSq angular acceleration feedforward in rad/s^2 (mechanism units)
   */
  public void setTarget(Angle position, double velocityRadPerSec, double accelerationRadPerSecSq) {
    double positionMechRot = position.in(edu.wpi.first.units.Units.Rotations);
    positionMechRot =
        MathUtil.clamp(
            positionMechRot, config.getLowerLimitRotations(), config.getUpperLimitRotations());
    target.set(new TurretTarget(positionMechRot, velocityRadPerSec, accelerationRadPerSecSq));
    enabled = true;

    Logger.recordOutput("SmartTurret/GoalPosition", positionMechRot);
  }

  /**
   * Steps the controller at 200 Hz. Determines the current state and applies the appropriate
   * control mode to the TalonFX.
   *
   * @param dtSeconds the time step (unused in onboard-profiling mode, retained for interface
   *     compatibility)
   */
  public void step(double dtSeconds) {
    if (!enabled) {
      return;
    }

    TurretTarget currentTarget = target.get();
    double actualPositionMechRot = talonFX.getPosition().getValueAsDouble();
    double positionError = Math.abs(currentTarget.positionMechRot() - actualPositionMechRot);

    // State transition with hysteresis.
    if (currentState == TurretState.SEEKING) {
      if (positionError < config.getSeekingThresholdRotations()) {
        currentState = TurretState.TRACKING;
      }
    } else { // TRACKING
      if (positionError
          > config.getSeekingThresholdRotations() + config.getHysteresisBufferRotations()) {
        currentState = TurretState.SEEKING;
      }
    }

    // Determine position-dependent kS.
    double directionSign = Math.signum(currentTarget.positionMechRot() - actualPositionMechRot);
    double effectiveKS = lookupKS(actualPositionMechRot, directionSign);

    switch (currentState) {
      case SEEKING:
        double seekingFF = effectiveKS * directionSign;
        seekingFF = applyFeedforwardSafetyPadding(actualPositionMechRot, seekingFF);
        talonFX.setControl(
            seekingRequest
                .withPosition(currentTarget.positionMechRot())
                .withFeedForward(seekingFF));
        break;

      case TRACKING:
        double velMechRotPerSec = currentTarget.velocityRadPerSec() / (2.0 * Math.PI);
        double accelMechRotPerSecSq = currentTarget.accelerationRadPerSecSq() / (2.0 * Math.PI);

        // Compute explicit feedforward in Amps:
        // kS for static friction, kV for velocity, kA for acceleration.
        double velocitySign = Math.signum(velMechRotPerSec);
        double ffAmps =
            effectiveKS * (Math.abs(velMechRotPerSec) > 1e-3 ? velocitySign : directionSign)
                + config.getKV() * velMechRotPerSec
                + config.getKA() * accelMechRotPerSecSq;
        ffAmps = applyFeedforwardSafetyPadding(actualPositionMechRot, ffAmps);

        talonFX.setControl(
            trackingRequest.withPosition(currentTarget.positionMechRot()).withFeedForward(ffAmps));
        break;
    }

    // Logging.
    Logger.recordOutput("SmartTurret/State", currentState.name());
    Logger.recordOutput("SmartTurret/PositionErrorRot", positionError);
    Logger.recordOutput("SmartTurret/ActualPositionMechRot", actualPositionMechRot);
    Logger.recordOutput("SmartTurret/TargetPositionMechRot", currentTarget.positionMechRot());
  }

  /**
   * Looks up the effective kS for the given turret position and direction of movement. Falls back
   * to the constant kS from config if no position-dependent map is available.
   */
  private double lookupKS(double positionMechRot, double directionSign) {
    InterpolatingDoubleTreeMap map =
        directionSign >= 0 ? config.getKsMapPositive() : config.getKsMapNegative();
    if (map != null) {
      return map.get(positionMechRot);
    }
    return config.getKS();
  }

  /**
   * Scales feedforward toward zero when the turret is near a soft limit to prevent hard stop
   * contact. The feedforward is linearly ramped down within the padding zone.
   */
  private double applyFeedforwardSafetyPadding(double positionMechRot, double feedforward) {
    double padding = config.getFeedforwardPaddingRotations();
    double upper = config.getUpperLimitRotations();
    double lower = config.getLowerLimitRotations();

    if (feedforward > 0 && positionMechRot > (upper - padding)) {
      double scale = MathUtil.clamp((upper - positionMechRot) / padding, 0.0, 1.0);
      return feedforward * scale;
    }
    if (feedforward < 0 && positionMechRot < (lower + padding)) {
      double scale = MathUtil.clamp((positionMechRot - lower) / padding, 0.0, 1.0);
      return feedforward * scale;
    }
    return feedforward;
  }

  /**
   * Estimates the time for the turret to arrive at the given goal position from its current state.
   *
   * <p>If the error is within the tracking threshold, returns a small constant. Otherwise, uses the
   * closed-form trapezoidal settling time for the seeking portion plus tracking settle time.
   *
   * @param goalPositionMechRot goal position in mechanism rotations
   * @return estimated time to arrival in seconds
   */
  public double getEstimatedTimeToArrival(double goalPositionMechRot) {
    double actualMechRot = talonFX.getPosition().getValueAsDouble();
    double errorRad = Math.abs(goalPositionMechRot - actualMechRot) * 2.0 * Math.PI;
    double velocityRadPerSec = getActualVelocityRadPerSec();
    double vMaxRad = config.getMaxVelocityMechRotPerSec() * 2.0 * Math.PI;
    double aMaxRad = config.getMaxAccelMechRotPerSecSq() * 2.0 * Math.PI;
    double seekingThresholdRad = config.getSeekingThresholdRotations() * 2.0 * Math.PI;

    // Small constant for tracking-mode settling time.
    double trackingSettleTime = 0.05;

    if (errorRad <= seekingThresholdRad) {
      return trackingSettleTime;
    }

    // Seeking time for the distance beyond the tracking threshold.
    double seekError = errorRad - seekingThresholdRad;
    double seekTime =
        TurretControlPhysics.trapezoidSettlingTime(seekError, velocityRadPerSec, vMaxRad, aMaxRad);
    return seekTime + trackingSettleTime;
  }

  /**
   * Returns the actual motor encoder velocity in mechanism rad/s.
   *
   * @return actual motor velocity in rad/s (mechanism units)
   */
  public double getActualVelocityRadPerSec() {
    return talonFX.getVelocity().getValueAsDouble() * 2.0 * Math.PI;
  }

  /**
   * Returns the actual motor encoder position in mechanism rotations.
   *
   * @return actual motor position in mechanism rotations
   */
  public double getActualPositionMechRot() {
    return talonFX.getPosition().getValueAsDouble();
  }

  /**
   * Returns the current goal position in mechanism rotations.
   *
   * @return goal mechanism position in rotations
   */
  public double getGoalPositionMechRot() {
    return target.get().positionMechRot();
  }

  /**
   * Returns the current turret state (SEEKING or TRACKING).
   *
   * @return current {@link TurretState}
   */
  public TurretState getCurrentTurretState() {
    return currentState;
  }

  /** Stops the turret by disabling the controller. The TalonFX will hold its last command. */
  public void stop() {
    enabled = false;
    double actualMechRot = talonFX.getPosition().getValueAsDouble();
    target.set(new TurretTarget(actualMechRot, 0, 0));
    currentState = TurretState.SEEKING;
  }

  /**
   * Resets the controller state. Use during robot init before encoder readings are available.
   *
   * @param currentPositionMechRot mechanism position in rotations to seed
   * @param currentVelocityMechRotPerSec mechanism velocity in rot/s to seed (unused, retained for
   *     API compatibility)
   */
  public void reset(double currentPositionMechRot, double currentVelocityMechRotPerSec) {
    target.set(new TurretTarget(currentPositionMechRot, 0, 0));
    currentState = TurretState.SEEKING;
    enabled = false;
  }

  /**
   * Returns the underlying TalonFX for use by tuning commands.
   *
   * @return the TalonFX hardware reference
   */
  public TalonFX getTalonFX() {
    return talonFX;
  }

  /**
   * Returns the configuration used to construct this controller.
   *
   * @return the {@link SmartTurretConfig}
   */
  public SmartTurretConfig getConfig() {
    return config;
  }
}
