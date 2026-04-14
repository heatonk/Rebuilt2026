package frc.robot.rebuilt.subsystems.Launcher;

import com.ctre.phoenix6.hardware.TalonFX;
import yams.motorcontrollers.SmartMotorController;

/**
 * Configuration for the {@link SmartTurretController} 2-state turret control system.
 *
 * <p>All slot feedforward values (kS, kV, kA) are in <b>Amps</b> for use with TorqueCurrentFOC
 * control. The expo profile parameters (expoKV, expoKA) are always in <b>Volts</b> regardless of
 * control mode. All positional values are in <b>mechanism rotations</b> (post-gear-reduction).
 */
public class SmartTurretConfig {

  // Hardware
  private final TalonFX talonFX;
  /**
   * YAMS SmartMotorController — its closed-loop Notifier will be stopped when SmartTurretController
   * takes over.
   */
  private final SmartMotorController yamsController;

  private final double gearRatio;

  // Motion constraints (mechanism rot/s and rot/s^2)
  private final double maxVelocityMechRotPerSec;
  private final double maxAccelMechRotPerSecSq;

  // Seeking PID (Slot0 — MotionMagicTorqueCurrentFOC)
  private final double seekingKP;
  private final double seekingKI;
  private final double seekingKD;

  // Tracking PID (Slot1 — PositionTorqueCurrentFOC)
  private final double trackingKP;
  private final double trackingKI;
  private final double trackingKD;

  // Feedforward in Amps (TorqueCurrentFOC units)
  private final double kS;
  private final double kV;
  private final double kA;

  // MotionMagicExpo plant model — ALWAYS in Volts regardless of control mode.
  // These define the mechanism's voltage-domain model for the expo velocity profile.
  private final double expoKV; // V/(mechanism rot/s)
  private final double expoKA; // V/(mechanism rot/s^2)

  // State transition thresholds (mechanism rotations)
  private final double seekingThresholdRotations;
  private final double hysteresisBufferRotations;

  // Soft limits (mechanism rotations)
  private final double lowerLimitRotations;
  private final double upperLimitRotations;

  // Peak torque current limit (Amps)
  private final double peakTorqueCurrentAmps;

  // Feedforward safety padding near limits (mechanism rotations)
  private final double feedforwardPaddingRotations;

  // Tracking-mode kS deadband (mechanism rotations).
  // When |positionError| < this value, external kS feedforward is zeroed to prevent chattering.
  private final double trackingDeadbandRotations;

  private SmartTurretConfig(Builder builder) {
    this.talonFX = builder.talonFX;
    this.yamsController = builder.yamsController;
    this.gearRatio = builder.gearRatio;
    this.maxVelocityMechRotPerSec = builder.maxVelocityMechRotPerSec;
    this.maxAccelMechRotPerSecSq = builder.maxAccelMechRotPerSecSq;
    this.seekingKP = builder.seekingKP;
    this.seekingKI = builder.seekingKI;
    this.seekingKD = builder.seekingKD;
    this.trackingKP = builder.trackingKP;
    this.trackingKI = builder.trackingKI;
    this.trackingKD = builder.trackingKD;
    this.kS = builder.kS;
    this.kV = builder.kV;
    this.kA = builder.kA;
    this.expoKV = builder.expoKV;
    this.expoKA = builder.expoKA;
    this.seekingThresholdRotations = builder.seekingThresholdRotations;
    this.hysteresisBufferRotations = builder.hysteresisBufferRotations;
    this.lowerLimitRotations = builder.lowerLimitRotations;
    this.upperLimitRotations = builder.upperLimitRotations;
    this.peakTorqueCurrentAmps = builder.peakTorqueCurrentAmps;
    this.feedforwardPaddingRotations = builder.feedforwardPaddingRotations;
    this.trackingDeadbandRotations = builder.trackingDeadbandRotations;
  }

  public TalonFX getTalonFX() {
    return talonFX;
  }

  /** Returns the YAMS SmartMotorController, or {@code null} if not configured. */
  public SmartMotorController getYamsController() {
    return yamsController;
  }

  public double getGearRatio() {
    return gearRatio;
  }

  public double getMaxVelocityMechRotPerSec() {
    return maxVelocityMechRotPerSec;
  }

  public double getMaxAccelMechRotPerSecSq() {
    return maxAccelMechRotPerSecSq;
  }

  public double getSeekingKP() {
    return seekingKP;
  }

  public double getSeekingKI() {
    return seekingKI;
  }

  public double getSeekingKD() {
    return seekingKD;
  }

  public double getTrackingKP() {
    return trackingKP;
  }

  public double getTrackingKI() {
    return trackingKI;
  }

  public double getTrackingKD() {
    return trackingKD;
  }

  public double getKS() {
    return kS;
  }

  public double getKV() {
    return kV;
  }

  public double getKA() {
    return kA;
  }

  /** MotionMagicExpo kV in V/(mechanism rot/s). Always in Volts regardless of control mode. */
  public double getExpoKV() {
    return expoKV;
  }

  /** MotionMagicExpo kA in V/(mechanism rot/s^2). Always in Volts regardless of control mode. */
  public double getExpoKA() {
    return expoKA;
  }

  public double getSeekingThresholdRotations() {
    return seekingThresholdRotations;
  }

  public double getHysteresisBufferRotations() {
    return hysteresisBufferRotations;
  }

  public double getLowerLimitRotations() {
    return lowerLimitRotations;
  }

  public double getUpperLimitRotations() {
    return upperLimitRotations;
  }

  public double getPeakTorqueCurrentAmps() {
    return peakTorqueCurrentAmps;
  }

  public double getFeedforwardPaddingRotations() {
    return feedforwardPaddingRotations;
  }

  public double getTrackingDeadbandRotations() {
    return trackingDeadbandRotations;
  }

  public static class Builder {
    private TalonFX talonFX;
    private SmartMotorController yamsController = null;
    private double gearRatio = 30.0;
    private double maxVelocityMechRotPerSec = 3.0;
    private double maxAccelMechRotPerSecSq = 2.78;
    private double seekingKP = 225;
    private double seekingKI = 0;
    private double seekingKD = 50;
    private double trackingKP = 225;
    private double trackingKI = 0;
    private double trackingKD = 50;
    private double kS = 10.0;
    private double kV = 0.0;
    private double kA = 5.0;
    // Expo defaults: computed from motion constraints in build() if not explicitly set.
    // -1 signals "auto-compute from maxVelocity/maxAccel".
    private double expoKV = -1;
    private double expoKA = -1;
    private double seekingThresholdRotations = 10.0 / 360.0;
    private double hysteresisBufferRotations = 3.0 / 360.0;
    private double lowerLimitRotations = -150.0 / 360.0;
    private double upperLimitRotations = 150.0 / 360.0;
    private double peakTorqueCurrentAmps = 240.0;
    private double feedforwardPaddingRotations = 10.0 / 360.0; // 10 degrees
    private double trackingDeadbandRotations = 0.25 / 360.0; // 0.25 degrees

    public Builder withTalonFX(TalonFX talonFX) {
      this.talonFX = talonFX;
      return this;
    }

    /**
     * Provides the YAMS {@link SmartMotorController} so {@link SmartTurretController} can stop its
     * background closed-loop Notifier thread on construction, preventing interference with our
     * direct TorqueCurrentFOC control.
     */
    public Builder withYAMSController(SmartMotorController yamsController) {
      this.yamsController = yamsController;
      return this;
    }

    public Builder withGearRatio(double gearRatio) {
      this.gearRatio = gearRatio;
      return this;
    }

    public Builder withMotionConstraints(
        double maxVelocityMechRotPerSec, double maxAccelMechRotPerSecSq) {
      this.maxVelocityMechRotPerSec = maxVelocityMechRotPerSec;
      this.maxAccelMechRotPerSecSq = maxAccelMechRotPerSecSq;
      return this;
    }

    public Builder withSeekingPID(double kP, double kI, double kD) {
      this.seekingKP = kP;
      this.seekingKI = kI;
      this.seekingKD = kD;
      return this;
    }

    public Builder withTrackingPID(double kP, double kI, double kD) {
      this.trackingKP = kP;
      this.trackingKI = kI;
      this.trackingKD = kD;
      return this;
    }

    public Builder withFeedforward(double kS, double kV, double kA) {
      this.kS = kS;
      this.kV = kV;
      this.kA = kA;
      return this;
    }

    /**
     * Sets the MotionMagicExpo plant model parameters in Volts. These are always in V/rps and
     * V/rps^2 regardless of control mode (even when using TorqueCurrentFOC).
     *
     * <p>If not called, defaults are auto-computed from motion constraints as {@code 12.0 /
     * maxVelocity} and {@code 12.0 / maxAcceleration}.
     */
    public Builder withExpoConstraints(double expoKV, double expoKA) {
      this.expoKV = expoKV;
      this.expoKA = expoKA;
      return this;
    }

    public Builder withSeekingThreshold(double seekingThresholdRotations) {
      this.seekingThresholdRotations = seekingThresholdRotations;
      return this;
    }

    public Builder withHysteresisBuffer(double hysteresisBufferRotations) {
      this.hysteresisBufferRotations = hysteresisBufferRotations;
      return this;
    }

    public Builder withSoftLimits(double lowerLimitRotations, double upperLimitRotations) {
      this.lowerLimitRotations = lowerLimitRotations;
      this.upperLimitRotations = upperLimitRotations;
      return this;
    }

    public Builder withPeakTorqueCurrent(double peakAmps) {
      this.peakTorqueCurrentAmps = peakAmps;
      return this;
    }

    public Builder withFeedforwardPadding(double feedforwardPaddingRotations) {
      this.feedforwardPaddingRotations = feedforwardPaddingRotations;
      return this;
    }

    public Builder withTrackingDeadband(double trackingDeadbandRotations) {
      this.trackingDeadbandRotations = trackingDeadbandRotations;
      return this;
    }

    public SmartTurretConfig build() {
      if (talonFX == null) {
        throw new IllegalStateException("TalonFX must be set");
      }
      // Auto-compute expo plant model from motion constraints if not explicitly set.
      // V_max / kV_expo = max achievable velocity; V_max / kA_expo = max achievable acceleration.
      if (expoKV < 0) {
        expoKV = 12.0 / maxVelocityMechRotPerSec;
      }
      if (expoKA < 0) {
        expoKA = 12.0 / maxAccelMechRotPerSecSq;
      }
      return new SmartTurretConfig(this);
    }
  }
}
