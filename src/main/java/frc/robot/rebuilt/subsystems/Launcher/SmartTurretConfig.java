package frc.robot.rebuilt.subsystems.Launcher;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Configuration for the {@link SmartTurretController} 2-state turret control system.
 *
 * <p>All feedforward values (kS, kV, kA) are in <b>Amps</b> for use with TorqueCurrentFOC control.
 * All positional values are in <b>mechanism rotations</b> (post-gear-reduction).
 */
public class SmartTurretConfig {

  // Hardware
  private final TalonFX talonFX;
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

  // State transition thresholds (mechanism rotations)
  private final double seekingThresholdRotations;
  private final double hysteresisBufferRotations;

  // Soft limits (mechanism rotations)
  private final double lowerLimitRotations;
  private final double upperLimitRotations;

  // Position-dependent kS maps (nullable — if null, uses constant kS)
  private final InterpolatingDoubleTreeMap ksMapPositive;
  private final InterpolatingDoubleTreeMap ksMapNegative;

  // Peak torque current limit (Amps)
  private final double peakTorqueCurrentAmps;

  // Feedforward safety padding near limits (mechanism rotations)
  private final double feedforwardPaddingRotations;

  private SmartTurretConfig(Builder builder) {
    this.talonFX = builder.talonFX;
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
    this.seekingThresholdRotations = builder.seekingThresholdRotations;
    this.hysteresisBufferRotations = builder.hysteresisBufferRotations;
    this.lowerLimitRotations = builder.lowerLimitRotations;
    this.upperLimitRotations = builder.upperLimitRotations;
    this.ksMapPositive = builder.ksMapPositive;
    this.ksMapNegative = builder.ksMapNegative;
    this.peakTorqueCurrentAmps = builder.peakTorqueCurrentAmps;
    this.feedforwardPaddingRotations = builder.feedforwardPaddingRotations;
  }

  public TalonFX getTalonFX() {
    return talonFX;
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

  public InterpolatingDoubleTreeMap getKsMapPositive() {
    return ksMapPositive;
  }

  public InterpolatingDoubleTreeMap getKsMapNegative() {
    return ksMapNegative;
  }

  public double getPeakTorqueCurrentAmps() {
    return peakTorqueCurrentAmps;
  }

  public double getFeedforwardPaddingRotations() {
    return feedforwardPaddingRotations;
  }

  public static class Builder {
    private TalonFX talonFX;
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
    private double seekingThresholdRotations = 10.0 / 360.0;
    private double hysteresisBufferRotations = 3.0 / 360.0;
    private double lowerLimitRotations = -150.0 / 360.0;
    private double upperLimitRotations = 150.0 / 360.0;
    private InterpolatingDoubleTreeMap ksMapPositive;
    private InterpolatingDoubleTreeMap ksMapNegative;
    private double peakTorqueCurrentAmps = 40.0;
    private double feedforwardPaddingRotations = 10.0 / 360.0; // 10 degrees

    public Builder withTalonFX(TalonFX talonFX) {
      this.talonFX = talonFX;
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

    public Builder withKsMaps(
        InterpolatingDoubleTreeMap positive, InterpolatingDoubleTreeMap negative) {
      this.ksMapPositive = positive;
      this.ksMapNegative = negative;
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

    public SmartTurretConfig build() {
      if (talonFX == null) {
        throw new IllegalStateException("TalonFX must be set");
      }
      return new SmartTurretConfig(this);
    }
  }
}
