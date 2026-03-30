package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

/**
 * RoboRIO-side trapezoidal profile controller for the turret.
 *
 * <p>Generates intermediate position setpoints from a trapezoidal velocity profile and sends them
 * to the TalonFX via {@link PositionVoltage}. This replaces the TalonFX-internal MotionMagic
 * profile so that the aiming solver's kinematic feedforward velocity can be integrated as the
 * profile's goal velocity rather than fighting an internal zero-velocity-targeting profile.
 *
 * <p>The {@link #step(double)} method is intended to be called at a high frequency (e.g. 200 Hz via
 * {@code addPeriodic}) while the goal is updated from the normal 20 ms robot loop via {@link
 * #setGoal(Angle, double)}.
 */
public class TurretProfileController {
  private final TalonFX talonFX;
  private final TrapezoidProfile.Constraints constraints;

  // Profile state in mechanism rotations (NOT motor rotations).
  private TrapezoidProfile.State currentState;
  private final AtomicReference<TrapezoidProfile.State> goalState;

  // Soft limits in mechanism rotations.
  private final double lowerLimitRotations;
  private final double upperLimitRotations;

  private final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(true);
  private boolean enabled = false;

  /**
   * @param talonFX the raw TalonFX hardware reference
   * @param maxVelocityMechRotPerSec maximum mechanism velocity in rot/s
   * @param maxAccelMechRotPerSecSq maximum mechanism acceleration in rot/s²
   * @param lowerLimitRotations lower soft limit in mechanism rotations
   * @param upperLimitRotations upper soft limit in mechanism rotations
   */
  public TurretProfileController(
      TalonFX talonFX,
      double maxVelocityMechRotPerSec,
      double maxAccelMechRotPerSecSq,
      double lowerLimitRotations,
      double upperLimitRotations) {
    this.talonFX = talonFX;
    this.constraints =
        new TrapezoidProfile.Constraints(maxVelocityMechRotPerSec, maxAccelMechRotPerSecSq);
    this.lowerLimitRotations = lowerLimitRotations;
    this.upperLimitRotations = upperLimitRotations;
    this.currentState = new TrapezoidProfile.State(0, 0);
    this.goalState = new AtomicReference<>(new TrapezoidProfile.State(0, 0));
  }

  /**
   * Sets the profile goal.
   *
   * <p>Called from the 20 ms aiming loop. The feedforward is expressed in mechanism rad/s and
   * converted to mechanism rot/s internally to match the profile's units.
   *
   * @param position desired turret mechanism angle
   * @param feedforwardRadPerSec angular velocity feedforward in rad/s (mechanism units)
   */
  public void setGoal(Angle position, double feedforwardRadPerSec) {
    double positionMechRot = position.in(edu.wpi.first.units.Units.Rotations);
    // Clamp to soft limits.
    positionMechRot = MathUtil.clamp(positionMechRot, lowerLimitRotations, upperLimitRotations);
    // Convert feedforward from rad/s to mechanism rot/s.
    double feedforwardMechRotPerSec = feedforwardRadPerSec / (2.0 * Math.PI);
    goalState.set(new TrapezoidProfile.State(positionMechRot, feedforwardMechRotPerSec));
    enabled = true;
    Logger.recordOutput("TurretProfile/End Goal Position", position);
    Logger.recordOutput(
        "TurretProfile/End Goal Velocity", RadiansPerSecond.of(feedforwardRadPerSec));
  }

  /**
   * Steps the profile controller.
   *
   * <p>When no goal is active (robot disabled or between commands), {@code currentState} is
   * continuously synced from the TalonFX encoder so the profile always starts from reality when a
   * goal is next set. When a goal is active, the trapezoidal profile is advanced by {@code
   * dtSeconds} and the resulting intermediate position is sent to the TalonFX via {@link
   * PositionVoltage}.
   *
   * <p>This method is called at a fixed high frequency (e.g. every 5 ms via Notifier).
   *
   * @param dtSeconds the time step in seconds
   */
  public void step(double dtSeconds) {
    if (!enabled) {
      // Sync profile state to actual motor position/velocity so there is no jump when a goal is
      // first set. The TalonFX signals are updated independently; reading the cached value here is
      // sufficient since positional accuracy during idle is not critical.
      double actualMechRot = talonFX.getPosition().getValueAsDouble();
      double actualMechRotPerSec = talonFX.getVelocity().getValueAsDouble();
      currentState = new TrapezoidProfile.State(actualMechRot, actualMechRotPerSec);
      return;
    }

    TrapezoidProfile.State goal = goalState.get();
    TrapezoidProfile profile = new TrapezoidProfile(constraints);
    currentState = profile.calculate(dtSeconds, currentState, goal);

    talonFX.setControl(
        positionRequest.withPosition(currentState.position).withVelocity(currentState.velocity));

    // Log profile state for tuning and diagnostics.
    Logger.recordOutput("TurretProfile/CurrentPositionMechRot", currentState.position);
    Logger.recordOutput("TurretProfile/CurrentVelocityMechRotPerSec", currentState.velocity);
    Logger.recordOutput("TurretProfile/GoalPositionMechRot", goal.position);
    Logger.recordOutput("TurretProfile/GoalVelocityMechRotPerSec", goal.velocity);
  }

  /**
   * Returns the current profile state in mechanism units.
   *
   * @return the current {@link TrapezoidProfile.State} (position in mechanism rot, velocity in
   *     mechanism rot/s)
   */
  public TrapezoidProfile.State getCurrentState() {
    return currentState;
  }

  /**
   * Returns the current profile velocity in mechanism rad/s.
   *
   * @return current velocity in rad/s
   */
  public double getCurrentVelocityRadPerSec() {
    return currentState.velocity * 2.0 * Math.PI;
  }

  /**
   * Computes the estimated time for the turret to travel from its current profile state to the
   * given goal using the closed-form analytical trapezoidal formula.
   *
   * @param goalPositionMechRot goal position in mechanism rotations
   * @return estimated settling time in seconds
   */
  public double getSettlingTime(double goalPositionMechRot) {
    double errorRad = (goalPositionMechRot - currentState.position) * 2.0 * Math.PI;
    double velocityRadPerSec = currentState.velocity * 2.0 * Math.PI;
    double vMaxRad = constraints.maxVelocity * 2.0 * Math.PI;
    double aMaxRad = constraints.maxAcceleration * 2.0 * Math.PI;
    return TurretControlPhysics.trapezoidSettlingTime(
        Math.abs(errorRad), velocityRadPerSec, vMaxRad, aMaxRad);
  }

  /**
   * Computes settling time from angle error and current profile velocity in radians using the
   * closed-form analytical trapezoidal formula.
   *
   * @param angleErrorRad absolute angle error in radians
   * @param currentVelocityRadPerSec current turret velocity in rad/s (positive = toward target)
   * @return estimated settling time in seconds
   */
  public double getSettlingTimeRadians(double angleErrorRad, double currentVelocityRadPerSec) {
    double vMaxRad = constraints.maxVelocity * 2.0 * Math.PI;
    double aMaxRad = constraints.maxAcceleration * 2.0 * Math.PI;
    return TurretControlPhysics.trapezoidSettlingTime(
        Math.abs(angleErrorRad), currentVelocityRadPerSec, vMaxRad, aMaxRad);
  }

  /** Stops the turret by holding it at the current actual motor position with zero velocity. */
  public void stop() {
    double actualMechRot = talonFX.getPosition().getValueAsDouble();
    goalState.set(new TrapezoidProfile.State(actualMechRot, 0));
    currentState = new TrapezoidProfile.State(actualMechRot, 0);
    enabled = false;
  }

  /**
   * Explicitly resets the profile state. Prefer calling {@link #stop()} for normal use; reserve
   * this for cases where you need to seed a specific position (e.g. during robot init before the
   * first encoder reading is available).
   *
   * @param currentPositionMechRot mechanism position in rotations to seed
   * @param currentVelocityMechRotPerSec mechanism velocity in rot/s to seed
   */
  public void reset(double currentPositionMechRot, double currentVelocityMechRotPerSec) {
    currentState = new TrapezoidProfile.State(currentPositionMechRot, currentVelocityMechRotPerSec);
    goalState.set(new TrapezoidProfile.State(currentPositionMechRot, 0));
    enabled = false;
  }

  /**
   * Returns the profile constraints.
   *
   * @return the trapezoidal profile constraints (maxVelocity and maxAcceleration in mechanism
   *     rot/s)
   */
  public TrapezoidProfile.Constraints getConstraints() {
    return constraints;
  }

  /**
   * Returns the current goal position in mechanism rotations. This is the final target angle last
   * set via {@link #setGoal}, not the intermediate profile setpoint currently being sent to the
   * motor.
   *
   * @return goal mechanism position in rotations
   */
  public double getGoalPositionMechRot() {
    return goalState.get().position;
  }
}
