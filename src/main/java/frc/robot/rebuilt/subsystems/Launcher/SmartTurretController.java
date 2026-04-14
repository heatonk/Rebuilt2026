package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
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
 * 2-state turret controller that switches between SEEKING and TRACKING modes.
 *
 * <p><b>SEEKING</b>: Uses {@link MotionMagicExpoTorqueCurrentFOC} (TalonFX exponential profiling)
 * for smooth, physically-optimal travel over large distances. The exponential profile adapts to the
 * mechanism's motor model (kV/kA) for crisp arrivals without overshoot. Velocity is capped by
 * {@code MotionMagicCruiseVelocity}.
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

  /** Signal update frequency for turret telemetry on CANivore (Hz). */
  private static final double SIGNAL_UPDATE_FREQUENCY_HZ = 250.0;

  private final TalonFX talonFX;
  private final SmartTurretConfig config;

  // High-frequency status signals — cached references for refreshAll() in characterization.
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<AngularAcceleration> accelerationSignal;
  private final StatusSignal<Current> torqueCurrentSignal;

  // Pre-allocated control requests (reused each cycle to avoid allocation).
  private final MotionMagicExpoTorqueCurrentFOC seekingRequest =
      new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);
  private final PositionTorqueCurrentFOC trackingRequest =
      new PositionTorqueCurrentFOC(0).withSlot(1);

  // Thread-safe target from 20 ms loop to 200 Hz step loop.
  private final AtomicReference<TurretTarget> target =
      new AtomicReference<>(new TurretTarget(0, 0, 0));

  private volatile TurretState currentState = TurretState.SEEKING;
  private volatile boolean enabled = false;
  private volatile boolean forceDisabled = false;

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

    // Slot0: Seeking (MotionMagicExpoTorqueCurrentFOC)
    fxConfig.Slot0.kP = config.getSeekingKP();
    fxConfig.Slot0.kI = config.getSeekingKI();
    fxConfig.Slot0.kD = config.getSeekingKD();
    fxConfig.Slot0.kS = config.getKS();
    fxConfig.Slot0.kV = config.getKV();
    fxConfig.Slot0.kA = config.getKA();

    // Slot1: Tracking (PositionTorqueCurrentFOC)
    // kV and kA run at firmware frequency (1 kHz) for the best temporal alignment with PID.
    // The Velocity field on the control request provides the reference for kV.
    // kS is set to 0 in the slot because it is position-dependent and injected externally
    // via withFeedForward() each cycle. This prevents chattering from velocity-sign-based kS
    // flipping at near-zero velocity.
    fxConfig.Slot1.kP = config.getTrackingKP();
    fxConfig.Slot1.kI = config.getTrackingKI();
    fxConfig.Slot1.kD = config.getTrackingKD();
    fxConfig.Slot1.kS = 0;
    fxConfig.Slot1.kV = config.getKV();
    fxConfig.Slot1.kA = config.getKA();

    // MotionMagicExpo: voltage-domain plant model for the exponential velocity profile.
    // These are ALWAYS in Volts (V/rps and V/rps²) regardless of control output type.
    // They define the mechanism's physical limits so the expo profile shapes appropriately.
    // CruiseVelocity provides an additional hard cap on profile velocity.
    fxConfig.MotionMagic.MotionMagicExpo_kV = config.getExpoKV();
    fxConfig.MotionMagic.MotionMagicExpo_kA = config.getExpoKA();
    fxConfig.MotionMagic.MotionMagicCruiseVelocity = config.getMaxVelocityMechRotPerSec();

    // Torque current peak limits.
    fxConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.getPeakTorqueCurrentAmps();
    fxConfig.TorqueCurrent.PeakReverseTorqueCurrent = -config.getPeakTorqueCurrentAmps();
    fxConfig.CurrentLimits.StatorCurrentLimitEnable = false;

    // Firmware-level soft limits — last line of defense against hard stop contact.
    fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.getUpperLimitRotations();
    fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.getLowerLimitRotations();

    talonFX.getConfigurator().apply(fxConfig);

    // Cache high-frequency status signals for position, velocity, and torque current.
    // Set them to 250 Hz on CANivore for smooth characterization data.
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
    if (!enabled || forceDisabled) {
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

    switch (currentState) {
      case SEEKING:
        talonFX.setControl(seekingRequest.withPosition(currentTarget.positionMechRot()));
        break;

      case TRACKING:
        // Compute position-error-directed kS feedforward externally.
        // Unlike firmware kS (which keys on velocity sign and chatters at zero),
        // this keys on position-error sign so it always pushes toward the target.
        // Inside the deadband, kS is zeroed to let the turret settle without oscillation.
        double signedError = currentTarget.positionMechRot() - actualPositionMechRot;
        double ksFeedforward;
        if (positionError < config.getTrackingDeadbandRotations()) {
          ksFeedforward = 0.0;
        } else {
          ksFeedforward = Math.signum(signedError) * config.getKS();
        }

        talonFX.setControl(
            trackingRequest
                .withPosition(currentTarget.positionMechRot())
                .withVelocity(RadiansPerSecond.of(currentTarget.velocityRadPerSec))
                .withFeedForward(ksFeedforward));

        break;
    }

    // Logging.
    Logger.recordOutput("SmartTurret/State", currentState.name());
    Logger.recordOutput("SmartTurret/PositionErrorRot", positionError);
    Logger.recordOutput("SmartTurret/ActualPositionMechRot", actualPositionMechRot);
    Logger.recordOutput("SmartTurret/TargetPositionMechRot", currentTarget.positionMechRot());
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
   * Returns the high-frequency position status signal (250 Hz on CANivore). Use with {@link
   * BaseStatusSignal#refreshAll} for latency-compensated reads.
   */
  public StatusSignal<Angle> getPositionSignal() {
    return positionSignal;
  }

  /**
   * Returns the high-frequency velocity status signal (250 Hz on CANivore). Use with {@link
   * BaseStatusSignal#refreshAll} for latency-compensated reads.
   */
  public StatusSignal<AngularVelocity> getVelocitySignal() {
    return velocitySignal;
  }

  /**
   * Returns the high-frequency torque current status signal (250 Hz on CANivore). Use with {@link
   * BaseStatusSignal#refreshAll} for latency-compensated reads.
   */
  public StatusSignal<Current> getTorqueCurrentSignal() {
    return torqueCurrentSignal;
  }

  /**
   * Returns the high-frequency acceleration status signal (250 Hz on CANivore). Use with {@link
   * BaseStatusSignal#refreshAll} for latency-compensated reads.
   */
  public StatusSignal<AngularAcceleration> getAccelerationSignal() {
    return accelerationSignal;
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
