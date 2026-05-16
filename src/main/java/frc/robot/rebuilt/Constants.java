package frc.robot.rebuilt;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class Constants {
  public static final String INDEXER = Indexer.class.getSimpleName();
  public static final String INTAKE = Intake.class.getSimpleName();
  public static final String LAUNCHER = Launcher.class.getSimpleName();

  public static class Launcher {
    public static final double SHOOTER_TOLERANCE_RPM = 50.0;
    public static final double HOOD_ANGLE_TOLERANCE_DEGREES = 3.5;
    public static final double TURRET_ANGLE_TOLERANCE_DEGREES = 5.0;
    public static final Angle HOPPER_EXTENSION_BUFFER_BEFORE_AIM = Degrees.of(10);
    public static final double HOOD_STALL_CURRENT_THRESHOLD = 20.0;

    public static final double HOOD_LEGACY_START_ANGLE_DEGREES = 30.0;
    public static final double HOOD_CORRECTED_START_ANGLE_DEGREES = 12.723;
    public static final double HOOD_CORRECTED_END_ANGLE_DEGREES = 45.723;
    public static final double HOOD_CALIBRATION_OFFSET_DEGREES =
        HOOD_CORRECTED_START_ANGLE_DEGREES - HOOD_LEGACY_START_ANGLE_DEGREES;

    public static double offsetLegacyHoodAngleDegrees(double legacyAngleDegrees) {
      return legacyAngleDegrees + HOOD_CALIBRATION_OFFSET_DEGREES;
    }

    public static final Angle LOW_HOOD_ANGLE = Degrees.of(offsetLegacyHoodAngleDegrees(31.0));
    public static final AngularVelocity LOW_FLYWHEEL_RPM = RotationsPerSecond.of(1);

    public static final Angle HUB_HOOD_ANGLE = LOW_HOOD_ANGLE;
    public static final AngularVelocity HUB_FLYWHEEL_RPM = RotationsPerSecond.of(1.25);

    public static final Angle TOWER_HOOD_ANGLE = Degrees.of(offsetLegacyHoodAngleDegrees(40.0));
    public static final AngularVelocity TOWER_FLYWHEEL_RPM = RotationsPerSecond.of(1.5);

    public static final Angle TURRET_FORWARD = Degrees.of(0);
    public static final AngularVelocity FWD_FLYWHEEL_RPM = LOW_FLYWHEEL_RPM;
    public static final Angle FWD_HOOD_ANGLE = LOW_HOOD_ANGLE;

    // Device construction parameters (transcribed from launcher/*.json under deploy/).
    public static class Turret {
      public static final int CAN_ID = 18;
      public static final String CAN_BUS = "canivore";
      public static final boolean INVERTED = true;
      public static final String GEAR_STAGES = "30:1";
      public static final double MASS_LBS = 15.0;
      public static final double RADIUS_INCHES = 5.0;
      public static final double LOWER_HARD_LIMIT_DEG = -160.0;
      public static final double UPPER_HARD_LIMIT_DEG = 160.0;
      public static final double LOWER_SOFT_LIMIT_DEG = -150.0;
      public static final double UPPER_SOFT_LIMIT_DEG = 150.0;
      public static final double STARTING_ANGLE_DEG = 0.0;
      public static final double ROBOT_TO_MOTOR_X_IN = -4.856;
      public static final double ROBOT_TO_MOTOR_Y_IN = 4.863;
      public static final double ROBOT_TO_MOTOR_Z_IN = 14.466;

      // Real hardware gains (SIMPLE control — SmartTurretController drives MotionMagic directly)
      public static final double KP = 1770.41578;
      public static final double KI = 0.0;
      public static final double KD = 1477.68211;
      public static final double KS = 12.2;
      public static final double KV = 3.06;
      public static final double KA = 1.94789461763;
      public static final double MAX_VEL_DEG_PER_SEC = 1080.0;
      public static final double MAX_ACCEL_DEG_PER_SEC_SQ = 10800.0;

      // Sim gains
      public static final double SIM_KP = 8.0;
      public static final double SIM_KI = 0.0;
      public static final double SIM_KD = 8.0;
      public static final double SIM_KS = 0.030215;
      public static final double SIM_KV = 0.00087341;
      public static final double SIM_KA = 0.98956;

      // SmartTurretController fallback feedforward (used when yams config doesn't provide one)
      public static final double SMART_FALLBACK_KS = 12.12;
      public static final double SMART_FALLBACK_KV = 3.06;
      public static final double SMART_FALLBACK_KA = 2.0;
      // SmartTurretController PID
      public static final double SMART_SEEKING_KP = 1050.0;
      public static final double SMART_SEEKING_KI = 0.0;
      public static final double SMART_SEEKING_KD = 144.886;
      public static final double SMART_TRACKING_KP = 1050.0;
      public static final double SMART_TRACKING_KI = 0.0;
      public static final double SMART_TRACKING_KD = 144.886;
    }

    public static class Hood {
      public static final int CAN_ID = 19;
      public static final boolean INVERTED = false;
      public static final boolean USE_TORQUE_CURRENT_FOC = true;
      public static final double CURRENT_LIMIT_AMPS = 60.0;
      public static final String GEAR_STAGES = "1015:33";
      public static final double LENGTH_INCHES = 9.466;
      public static final double MASS_LBS = 0.25;
      public static final double LOWER_HARD_LIMIT_DEG = 12.723;
      public static final double UPPER_HARD_LIMIT_DEG = 45.723;
      public static final double LOWER_SOFT_LIMIT_DEG = 12.723;
      public static final double UPPER_SOFT_LIMIT_DEG = 45.723;
      public static final double STARTING_ANGLE_DEG = 12.723;
      public static final double HORIZONTAL_ZERO_DEG = 5.3;

      // Real hardware gains (PROFILED control)
      public static final double KP = 1100.0;
      public static final double KI = 0.0;
      public static final double KD = 60.0;
      public static final double KS = 0.0;
      public static final double KV = 0.0;
      public static final double KA = 0.0972;
      public static final double KG = 4.357;
      public static final double MAX_VEL_DEG_PER_SEC = 1080.0;
      public static final double MAX_ACCEL_DEG_PER_SEC_SQ = 5000.0;

      // Sim gains
      public static final double SIM_KP = 9.0;
      public static final double SIM_KI = 0.0;
      public static final double SIM_KD = 0.0;
      public static final double SIM_KS = 0.0;
      public static final double SIM_KV = 0.0;
      public static final double SIM_KA = 0.0;
      public static final double SIM_KG = 0.0;
      public static final double SIM_MAX_VEL_DEG_PER_SEC = 180.0;
      public static final double SIM_MAX_ACCEL_DEG_PER_SEC_SQ = 90.0;
    }

    public static class FlyWheel {
      public static final int CAN_ID = 16;
      public static final int FOLLOWER_CAN_ID = 17;
      public static final boolean INVERTED = true;
      public static final boolean FOLLOWER_INVERTED = true;
      public static final double CURRENT_LIMIT_AMPS = 80.0;
      public static final String GEAR_STAGES = "18:1";
      public static final double MASS_LBS = 5.1;
      public static final double RADIUS_INCHES = 1.975;
      public static final double LOWER_SOFT_LIMIT_RPM = 0.0;
      public static final double UPPER_SOFT_LIMIT_RPM = 5000.0;
      public static final double ROBOT_TO_MOTOR_X_IN = -5.872;
      public static final double ROBOT_TO_MOTOR_Y_IN = 4.8;
      public static final double ROBOT_TO_MOTOR_Z_IN = 14.466;

      // Real hardware gains
      public static final double KP = 5.0;
      public static final double KI = 0.0;
      public static final double KD = 0.0;
      public static final double KS = 0.066854;
      public static final double KV = 2.1;
      public static final double KA = 0.38848;

      // Sim gains
      public static final double SIM_KP = 2.0;
      public static final double SIM_KI = 0.0;
      public static final double SIM_KD = 0.0;
      public static final double SIM_KS = 0.066854;
      public static final double SIM_KV = 2.1962;
      public static final double SIM_KA = 0.38848;
    }

    public static class CrtEncoders {
      public static final int ENCODER_40_CAN_ID = 21;
      public static final int ENCODER_36_CAN_ID = 22;
      public static final String CAN_BUS = "canivore";
      public static final double ENCODER_40_OFFSET_ROT = -0.46923828125;
      public static final double ENCODER_36_OFFSET_ROT = 0.129638671875;
      public static final double SIM_ENCODER_40_VALUE = 0.391;
      public static final double SIM_ENCODER_36_VALUE = 0.274;
    }
  }

  public static class Indexer {
    public static final double SPINDEXER_SPEED = 0.90;
    public static final double TRANSFER_SPEED = 1.0;
    public static final double TRANSFER_CHURN = 0.25;

    // Device construction parameters (transcribed from indexer/*.json under deploy/).
    public static class Spindexer {
      public static final int CAN_ID = 9;
      public static final double CURRENT_LIMIT_AMPS = 120.0;
    }

    public static class Transfer {
      public static final int CAN_ID = 10;
      public static final int FOLLOWER_CAN_ID = 11;
      public static final boolean INVERTED = true;
      public static final boolean FOLLOWER_INVERTED = false;
      public static final String GEAR_STAGES = "1:1";
      public static final double MASS_KG = 2.0;
      public static final double RADIUS_M = 0.05;
      public static final double LOWER_SOFT_LIMIT_RPM = 0.0;
      public static final double UPPER_SOFT_LIMIT_RPM = 5000.0;

      // Closed-loop gains exist in JSON but are unused (transfer runs in duty-cycle).
      public static final double KP = 4.0;
      public static final double KI = 0.0;
      public static final double KD = 0.0;
      public static final double KS = 0.0;
      public static final double KV = 0.0;
      public static final double KA = 0.0;
    }
  }

  public static class Intake {
    public static final double HOPPER_GO_OUT = -0.3;
    public static final double HOPPER_GO_IN = 0.2;
    public static final double INTAKE_IN = 1.0;
    public static final double INTAKE_INNER_IN = 0.3;
    public static final double INTAKE_AUTO = 1.0;
    public static final double INTAKE_DEADZONE = 0.25;
    public static final double INTAKE_CHURN = -0.25;
    public static final double INTAKE_MAX_IN = 0.9;
    public static final double INTAKE_MAX_OUT = -0.9;
    public static final double HOPPER_ANGLE_TOLERANCE = 3;
    public static final double HOPPER_STALL_TIME = 0.3;
    public static final Angle HOPPER_RETRACTED_ANGLE = Degrees.of(120);
    public static final Angle HOPPER_DEPLOYED_ANGLE = Degrees.of(0);
    public static final Angle HOPPER_ANGLED = Degrees.of(45);
    public static final double HOPPER_STALL_CURRENT_THRESHOLD = 80.0;
    public static final double HOPPER_MOVING_VELOCITY_THRESHOLD = 1.0;
    public static final double HOPPER_DEPLOY_STOP_REZERO_MAX_ANGLE = 20.0;
    public static final double HOPPER_AUTO_REZERO_THRESHOLD =
        -1.0; // degrees — if hopper goes past 0, auto-rezero
    public static final double HOPPER_FIRST_DEPLOY_DUTY =
        -0.2; // duty cycle for first-deploy zeroing nudge
    public static final double HOPPER_DEPLOY_NUDGE_DUTY =
        -0.35; // duty cycle for normal deploy nudge after PID

    // Device construction parameters (transcribed from intake/*.json under deploy/).
    public static class Hopper {
      public static final int CAN_ID = 15;
      public static final int FOLLOWER_CAN_ID = 14;
      public static final boolean INVERTED = true;
      public static final boolean FOLLOWER_INVERTED = true;
      public static final double CURRENT_LIMIT_AMPS = 60.0;
      public static final String GEAR_STAGES = "24:1";
      public static final double LENGTH_METERS = 0.234;
      public static final double MASS_LBS = 1.0;
      public static final double LOWER_HARD_LIMIT_DEG = 0.0;
      public static final double UPPER_HARD_LIMIT_DEG = 125.0;
      public static final double LOWER_SOFT_LIMIT_DEG = 0.0;
      public static final double UPPER_SOFT_LIMIT_DEG = 120.0;
      public static final double STARTING_ANGLE_DEG = 125.0;
      public static final double HORIZONTAL_ZERO_DEG = 0.0;

      // Real hardware gains (PROFILED control)
      public static final double KP = 40.0;
      public static final double KI = 0.0;
      public static final double KD = 5.0;
      public static final double KS = 0.5992224858009301;
      public static final double KV = 0.010060789918368191;
      public static final double KA = 0.0;
      public static final double KG = 0.65;
      public static final double MAX_VEL_DEG_PER_SEC = 1080.0;
      public static final double MAX_ACCEL_DEG_PER_SEC_SQ = 5000.0;

      // Sim gains
      public static final double SIM_KP = 10.0;
      public static final double SIM_KI = 0.0;
      public static final double SIM_KD = 0.0;
      public static final double SIM_KS = 0.048722;
      public static final double SIM_KV = 0.005486;
      public static final double SIM_KA = 0.0;
      public static final double SIM_KG = 0.04055;
      public static final double SIM_MAX_VEL_DEG_PER_SEC = 360.0;
      public static final double SIM_MAX_ACCEL_DEG_PER_SEC_SQ = 180.0;
    }

    public static class SpintakeInner {
      public static final int CAN_ID = 12;
      public static final boolean INVERTED = true;
      public static final String GEAR_STAGES = "11:36";
      public static final double MASS_KG = 2.0;
      public static final double RADIUS_M = 0.05;
      public static final double LOWER_SOFT_LIMIT_RPM = 0.0;
      public static final double UPPER_SOFT_LIMIT_RPM = 5000.0;

      public static final double KP = 4.0;
      public static final double KI = 0.0;
      public static final double KD = 0.0;
      public static final double KS = 0.0;
      public static final double KV = 0.0;
      public static final double KA = 0.0;
    }

    public static class SpintakeOuter {
      public static final int CAN_ID = 13;
      public static final boolean INVERTED = true;
      public static final String GEAR_STAGES = "11:36";
      public static final double MASS_KG = 2.0;
      public static final double RADIUS_M = 0.05;
      public static final double LOWER_SOFT_LIMIT_RPM = 0.0;
      public static final double UPPER_SOFT_LIMIT_RPM = 5000.0;

      public static final double KP = 4.0;
      public static final double KI = 0.0;
      public static final double KD = 0.0;
      public static final double KS = 0.0;
      public static final double KV = 0.0;
      public static final double KA = 0.0;
    }
  }
}
