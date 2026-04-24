package frc.robot.rebuilt.subsystems.Launcher;

import com.ctre.phoenix6.hardware.TalonFX;
import yams.motorcontrollers.SmartMotorController;

/**
 * Configuration for the {@link SmartFlywheelController} TorqueCurrentFOC control system.
 *
 * <p>All slot feedforward values (kS, kV, kA) are in <b>Amps</b> for use with TorqueCurrentFOC
 * control.
 */
public class SmartFlywheelConfig {

  // Hardware
  private final TalonFX talonFX;
  private final SmartMotorController yamsController;

  private final double gearRatio;

  // PID
  private final double kP;
  private final double kI;
  private final double kD;

  // Feedforward in Amps (TorqueCurrentFOC units)
  private final double kS;
  private final double kV;
  private final double kA;

  // Peak torque current limit (Amps)
  private final double peakTorqueCurrentAmps;

  private SmartFlywheelConfig(Builder builder) {
    this.talonFX = builder.talonFX;
    this.yamsController = builder.yamsController;
    this.gearRatio = builder.gearRatio;
    this.kP = builder.kP;
    this.kI = builder.kI;
    this.kD = builder.kD;
    this.kS = builder.kS;
    this.kV = builder.kV;
    this.kA = builder.kA;
    this.peakTorqueCurrentAmps = builder.peakTorqueCurrentAmps;
  }

  public TalonFX getTalonFX() {
    return talonFX;
  }

  public SmartMotorController getYamsController() {
    return yamsController;
  }

  public double getGearRatio() {
    return gearRatio;
  }

  public double getKP() {
    return kP;
  }

  public double getKI() {
    return kI;
  }

  public double getKD() {
    return kD;
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

  public double getPeakTorqueCurrentAmps() {
    return peakTorqueCurrentAmps;
  }

  public static class Builder {
    private TalonFX talonFX;
    private SmartMotorController yamsController = null;
    private double gearRatio = 1.0;
    private double kP = 0.5;
    private double kI = 0;
    private double kD = 0;
    private double kS = 0.5;
    private double kV = 0.1;
    private double kA = 0.0;
    private double peakTorqueCurrentAmps = 120.0;

    public Builder withTalonFX(TalonFX talonFX) {
      this.talonFX = talonFX;
      return this;
    }

    public Builder withYAMSController(SmartMotorController yamsController) {
      this.yamsController = yamsController;
      return this;
    }

    public Builder withGearRatio(double gearRatio) {
      this.gearRatio = gearRatio;
      return this;
    }

    public Builder withPID(double kP, double kI, double kD) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      return this;
    }

    public Builder withFeedforward(double kS, double kV, double kA) {
      this.kS = kS;
      this.kV = kV;
      this.kA = kA;
      return this;
    }

    public Builder withPeakTorqueCurrent(double peakAmps) {
      this.peakTorqueCurrentAmps = peakAmps;
      return this;
    }

    public SmartFlywheelConfig build() {
      if (talonFX == null) {
        throw new IllegalStateException("TalonFX must be set");
      }
      return new SmartFlywheelConfig(this);
    }
  }
}
