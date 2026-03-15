package frc.robot.rebuilt;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.rebuilt.subsystems.Climb.Climb;

public class Constants {
  public static final String INDEXER = Indexer.class.getSimpleName();
  public static final String CLIMB = Climb.class.getSimpleName();
  public static final String INTAKE = Intake.class.getSimpleName();
  public static final String LAUNCHER = Launcher.class.getSimpleName();
  /** Defines the maxinum distance for the climb */
  public static class ClimbConstants {
    public static final Distance MAX = Inch.of(27);
  }

  public static class Launcher {
    public static final double SHOOTER_TOLERANCE_RPM = 50.0;
    public static final double HOOD_ANGLE_TOLERANCE_DEGREES = 1.0;
    public static final double TURRET_ANGLE_TOLERANCE_DEGREES = 1.5;

    public static final Angle LOW_HOOD_ANGLE = Degrees.of(31);
    public static final AngularVelocity LOW_FLYWHEEL_RPM = RotationsPerSecond.of(1);

    public static final Angle HUB_HOOD_ANGLE = LOW_HOOD_ANGLE;
    public static final AngularVelocity HUB_FLYWHEEL_RPM = RotationsPerSecond.of(1.25);

    public static final Angle TOWER_HOOD_ANGLE = Degrees.of(40);
    public static final AngularVelocity TOWER_FLYWHEEL_RPM = RotationsPerSecond.of(1.5);

    public static final Angle TURRET_FORWARD = Degrees.of(0);
    public static final AngularVelocity FWD_FLYWHEEL_RPM = LOW_FLYWHEEL_RPM;
    public static final Angle FWD_HOOD_ANGLE = LOW_HOOD_ANGLE;
  }

  public static class Indexer {
    public static final double SPINDEXER_SPEED = 0.7;
    public static final double TRANSFER_SPEED = 0.7;
    public static final double TRANSFER_CHURN = 0.25;
  }

  public static class Intake {
    public static final double HOPPER_GO_OUT = -0.3;
    public static final double HOPPER_GO_IN = 0.2;
    public static final double HOPPER_OUT = 0.00;
    public static final double INTAKE_IN = 0.0;
    public static final double INTAKE_OUT = -0.0;
    public static final double INTAKE_MAX_IN = 0.9;
    public static final double INTAKE_MAX_OUT = -0.9;
    public static final double HOPPER_STALL_CURRENT_THRESHOLD = 5.0;
  }
}
