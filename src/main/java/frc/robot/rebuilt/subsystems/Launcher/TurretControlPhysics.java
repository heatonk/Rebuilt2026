package frc.robot.rebuilt.subsystems.Launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BiFunction;
import java.util.function.DoubleFunction;

public class TurretControlPhysics {
  /** Defines configuration parameters and physical constraints for turret aiming calculations */
  private final Translation2d turretOffsetRobotFrame;

  private final Rotation2d minTurretAngle;
  private final Rotation2d maxTurretAngle;
  private final Rotation2d feedforwardPaddingAngle;
  private final double settlingTimeGain; // Alpha filter gain (0.0 to 1.0)

  private static final double DERIVATIVE_PROBE_TIME_DELTA = 0.005; // 5ms
  private static final int MAX_SOLVER_ITERATIONS = 4;
  private static final double CONVERGENCE_THRESHOLD_SECONDS = 0.001;

  private final DoubleFunction<Double> timeOfFlightFunction;
  private final BiFunction<Double, Double, Double> settlingTimeFunction;
  private final double minEffectiveRangeMeters;
  private final double maxEffectiveRangeMeters;

  /**
   * Builds a settling-time function from trapezoidal motion-profile constraints.
   *
   * <p>Given angle error {@code |Δθ|} in radians the function returns the time (seconds) for the
   * turret to reach its goal under a trapezoidal velocity profile with peak velocity {@code vMax}
   * (rad/s) and peak acceleration {@code aMax} (rad/s²).
   *
   * <ul>
   *   <li>Triangle phase: {@code t = 2 * sqrt(|Δθ| / aMax)} when {@code |Δθ| < vMax²/aMax}
   *   <li>Trapezoid phase: {@code t = vMax/aMax + |Δθ|/vMax} otherwise
   * </ul>
   *
   * @param maxVelocityRadPerSec peak turret velocity (rad/s)
   * @param maxAccelRadPerSecSq peak turret acceleration (rad/s²)
   * @return a {@link DoubleFunction} mapping |angleErrorRadians| → settlingTimeSeconds
   */
  public static DoubleFunction<Double> trapezoidalSettlingTimeFunction(
      double maxVelocityRadPerSec, double maxAccelRadPerSecSq) {
    double vMax = Math.abs(maxVelocityRadPerSec);
    double aMax = Math.abs(maxAccelRadPerSecSq);
    if (vMax < 1e-6 || aMax < 1e-6) {
      return (err) -> 0.0;
    }
    double triangleThreshold = (vMax * vMax) / aMax;
    return (angleErrorRad) -> {
      double err = Math.abs(angleErrorRad);
      if (err < triangleThreshold) {
        return 2.0 * Math.sqrt(err / aMax);
      } else {
        return vMax / aMax + err / vMax;
      }
    };
  }

  /**
   * Builds a velocity-aware settling-time function from trapezoidal motion-profile constraints.
   *
   * <p>Unlike {@link #trapezoidalSettlingTimeFunction}, this version accounts for the turret's
   * current velocity when estimating time-to-arrival. Uses a closed-form analytical formula — O(1)
   * cost with a single sqrt — derived from the three-phase trapezoidal profile:
   *
   * <ul>
   *   <li>Moving toward target without overshoot: normal accel-cruise-decel formula
   *   <li>Moving toward target but overshooting: decelerate past goal, then come back from rest
   *   <li>Moving away from target: decelerate to stop (going backward), then move forward
   * </ul>
   *
   * <p>The returned estimate is accurate to within the settlingTimeGain that the caller applies;
   * the overshoot case uses a symmetric-return path which very slightly undershoots reality.
   *
   * @param maxVelocityRadPerSec peak turret velocity (rad/s)
   * @param maxAccelRadPerSecSq peak turret acceleration (rad/s²)
   * @return a {@link BiFunction} mapping (|angleErrorRadians|, currentVelocityRadPerSec) →
   *     settlingTimeSeconds
   */
  public static BiFunction<Double, Double, Double> velocityAwareSettlingTimeFunction(
      double maxVelocityRadPerSec, double maxAccelRadPerSecSq) {
    double vMax = Math.abs(maxVelocityRadPerSec);
    double aMax = Math.abs(maxAccelRadPerSecSq);
    if (vMax < 1e-6 || aMax < 1e-6) {
      return (err, vel) -> 0.0;
    }
    return (angleErrorRad, currentVelocityRadPerSec) ->
        trapezoidSettlingTime(Math.abs(angleErrorRad), currentVelocityRadPerSec, vMax, aMax);
  }

  /**
   * Builds a settling-time function that accounts for the 2-state SmartTurretController behavior.
   *
   * <p>When the turret is already within the seeking threshold (close to target), only a small
   * constant tracking settle time is needed. When farther away, the trapezoidal settling time is
   * used for the seeking portion beyond the threshold, plus the tracking settle time.
   *
   * @param seekingThresholdRad the SEEKING -> TRACKING transition threshold in radians
   * @param trackingSettleTimeSeconds constant settle time for the tracking-mode PID (~0.05s)
   * @param maxVelocityRadPerSec peak turret velocity (rad/s)
   * @param maxAccelRadPerSecSq peak turret acceleration (rad/s^2)
   * @return a {@link BiFunction} mapping (|angleErrorRadians|, currentVelocityRadPerSec) ->
   *     settlingTimeSeconds
   */
  public static BiFunction<Double, Double, Double> twoStateSettlingTimeFunction(
      double seekingThresholdRad,
      double trackingSettleTimeSeconds,
      double maxVelocityRadPerSec,
      double maxAccelRadPerSecSq) {
    double vMax = Math.abs(maxVelocityRadPerSec);
    double aMax = Math.abs(maxAccelRadPerSecSq);
    if (vMax < 1e-6 || aMax < 1e-6) {
      return (err, vel) -> 0.0;
    }
    return (angleErrorRad, currentVelocityRadPerSec) -> {
      double absError = Math.abs(angleErrorRad);
      if (absError <= seekingThresholdRad) {
        return trackingSettleTimeSeconds;
      }
      double seekError = absError - seekingThresholdRad;
      double seekTime = trapezoidSettlingTime(seekError, currentVelocityRadPerSec, vMax, aMax);
      return seekTime + trackingSettleTimeSeconds;
    };
  }

  /**
   * Closed-form trapezoidal settling-time estimate.
   *
   * @param d absolute angle error in rad (must be ≥ 0)
   * @param v0 current velocity in rad/s (positive = toward target)
   * @param vMax peak velocity in rad/s
   * @param aMax peak acceleration in rad/s²
   * @return estimated settling time in seconds
   */
  static double trapezoidSettlingTime(double d, double v0, double vMax, double aMax) {
    if (d < 1e-9) return 0.0;
    // Clamp to reachable velocity range.
    v0 = Math.max(-vMax, Math.min(vMax, v0));

    if (v0 < 0.0) {
      // Moving away from the target. First decelerate to rest (while traveling backward),
      // then cover the original distance plus the backward overshoot from rest.
      double tDecel = -v0 / aMax; // > 0
      double dBack = v0 * v0 / (2.0 * aMax); // distance traveled backward
      return tDecel + fromRest(d + dBack, vMax, aMax);
    }

    // v0 >= 0: moving toward the target.
    double dStop = v0 * v0 / (2.0 * aMax); // braking distance from v0 to 0
    if (dStop > d) {
      // Would overshoot. Decelerate past the target, stop, then return from rest.
      double tDecel = v0 / aMax;
      double dOvershoot = dStop - d; // how far past the target before stopping
      return tDecel + fromRest(dOvershoot, vMax, aMax);
    }

    // Normal approach: won't overshoot. Check trapezoid vs triangle.
    double d1 = (vMax * vMax - v0 * v0) / (2.0 * aMax); // to accelerate from v0 to vMax
    double d3 = vMax * vMax / (2.0 * aMax); // to decelerate from vMax to 0
    if (d >= d1 + d3) {
      // Trapezoid: accelerate to vMax, cruise, decelerate to 0.
      double t1 = (vMax - v0) / aMax;
      double t2 = (d - d1 - d3) / vMax;
      double t3 = vMax / aMax;
      return t1 + t2 + t3;
    } else {
      // Triangle: accelerate to v_peak, decelerate to 0. v_peak < vMax.
      double vPeak = Math.sqrt(aMax * d + v0 * v0 / 2.0);
      return (vPeak - v0) / aMax + vPeak / aMax;
    }
  }

  /** Settling time from rest (v=0) to cover distance d under a trapezoidal profile. */
  private static double fromRest(double d, double vMax, double aMax) {
    double threshold = vMax * vMax / aMax;
    if (d < threshold) {
      return 2.0 * Math.sqrt(d / aMax); // triangle
    } else {
      return vMax / aMax + d / vMax; // trapezoid
    }
  }

  public record RobotState(Pose2d pose, ChassisSpeeds velocity, ChassisSpeeds acceleration) {}

  @FunctionalInterface
  public interface RobotPredictor {
    RobotState predict(double timeSinceStartSeconds, double lookaheadSeconds);
  }

  /**
   * @param turretOffsetRobotFrame Vector from robot center to turret center (Robot Frame).
   * @param minTurretAngle Minimum physical rotation limit (e.g. -165 deg).
   * @param maxTurretAngle Maximum physical rotation limit (e.g. +165 deg).
   * @param feedforwardPaddingAngle Buffer zone near limits where velocity is ramped down.
   * @param settlingGain Gain for settling time estimation (<1.0 underestimates to prevent
   *     oscillation).
   * @param timeOfFlightFunc Function returning projectile flight time (s) given distance (m).
   * @param settlingTimeFunc Function returning turret settling time (s) given angle error (rad) and
   *     current velocity (rad/s).
   * @param minRangeMeters Minimum effective shot range.
   * @param maxRangeMeters Maximum effective shot range.
   */
  public TurretControlPhysics(
      Translation2d turretOffsetRobotFrame,
      Rotation2d minTurretAngle,
      Rotation2d maxTurretAngle,
      Rotation2d feedforwardPaddingAngle,
      double settlingGain,
      DoubleFunction<Double> timeOfFlightFunc,
      BiFunction<Double, Double, Double> settlingTimeFunc,
      double minRangeMeters,
      double maxRangeMeters) {
    this.turretOffsetRobotFrame = turretOffsetRobotFrame;
    SmartDashboard.putNumber("Turret Offset X", turretOffsetRobotFrame.getX());
    SmartDashboard.putNumber("Turret Offset Y", turretOffsetRobotFrame.getY());
    SmartDashboard.putNumber("Turret Offset Angle", turretOffsetRobotFrame.getAngle().getDegrees());

    this.minTurretAngle = minTurretAngle;
    this.maxTurretAngle = maxTurretAngle;
    this.feedforwardPaddingAngle = feedforwardPaddingAngle;
    this.settlingTimeGain = settlingGain;
    this.timeOfFlightFunction = timeOfFlightFunc;
    this.settlingTimeFunction = settlingTimeFunc;
    this.minEffectiveRangeMeters = minRangeMeters;
    this.maxEffectiveRangeMeters = maxRangeMeters;
  }
  /** Defines possible states for the aiming status */
  public enum AimingStatus {
    READY_TO_FIRE,
    TARGET_TOO_CLOSE,
    TARGET_TOO_FAR,
    IN_DEADZONE,
    SOLVER_FAILED
  }
  /** Defines data representing for the solver result */
  public record AimingSolution(
      Translation2d virtualTargetFieldPos,
      Rotation2d turretFieldHeading,
      Rotation2d turretLocalHeading,
      double turretFeedforwardRadPerSec,
      double effectiveDistanceMeters,
      double estimatedTimeOfFlight,
      AimingStatus status,
      SolverState finalSolverState) {
    public boolean isPossible() {
      return status == AimingStatus.READY_TO_FIRE;
    }
  }

  /**
   * Solves for the optimal turret angle and feedforward velocity.
   *
   * @param targetFieldPos The field-relative position of the target.
   * @param currentTurretAngle The current robot-relative angle of the turret.
   * @param currentTurretVelocityRadPerSec The current turret angular velocity in rad/s.
   * @param predictor The prediction logic to estimate future robot states.
   * @return A complete aiming solution including setpoints and status.
   */
  public AimingSolution solve(
      Translation2d targetFieldPos,
      Rotation2d currentTurretAngle,
      double currentTurretVelocityRadPerSec,
      RobotPredictor predictor) {

    SolverState finalState =
        runNewtonSolver(
            targetFieldPos, currentTurretAngle, currentTurretVelocityRadPerSec, predictor);

    Rotation2d fieldHeading = getAngleFromVector(finalState.vectorToVirtualTarget);
    Rotation2d localHeading = fieldHeading.minus(finalState.robotStateAtFire.pose().getRotation());

    double feedforwardRadPerSec = calculateKinematicFeedforward(finalState);

    AimingStatus status = AimingStatus.READY_TO_FIRE;
    double distanceToTarget = finalState.vectorToVirtualTarget.getNorm();
    /** Checks the distance to the target for effective shooting range */
    if (distanceToTarget < minEffectiveRangeMeters) {
      status = AimingStatus.TARGET_TOO_CLOSE;
    } else if (distanceToTarget > maxEffectiveRangeMeters) {
      status = AimingStatus.TARGET_TOO_FAR;
    } else if (!finalState.hasConverged) {
      status = AimingStatus.SOLVER_FAILED;
    }

    double localHeadingRadians = MathUtil.angleModulus(localHeading.getRadians());
    double minLimitRadians = minTurretAngle.getRadians();
    double maxLimitRadians = maxTurretAngle.getRadians();

    if (localHeadingRadians < minLimitRadians || localHeadingRadians > maxLimitRadians) {
      status = AimingStatus.IN_DEADZONE;
      /**
       * If the angle is outside limits, then it clamps to a valid limit based on motion direction
       */
      if (feedforwardRadPerSec > 0.1) {
        localHeading = minTurretAngle;
      } else if (feedforwardRadPerSec < -0.1) {
        localHeading = maxTurretAngle;
      } else {
        /** Sets the turret closest angle limit if the feed forward is near zero */
        double distanceToMin =
            Math.abs(MathUtil.angleModulus(localHeadingRadians - minLimitRadians));
        double distanceToMax =
            Math.abs(MathUtil.angleModulus(localHeadingRadians - maxLimitRadians));
        localHeading = (distanceToMin < distanceToMax) ? minTurretAngle : maxTurretAngle;
      }

      feedforwardRadPerSec = 0.0;

    } else {
      feedforwardRadPerSec =
          applyFeedforwardSafetyPadding(localHeadingRadians, feedforwardRadPerSec);
    }

    return new AimingSolution(
        finalState.virtualTargetFieldPos,
        fieldHeading,
        localHeading,
        feedforwardRadPerSec,
        distanceToTarget,
        finalState.requiredTimeOfFlight,
        status,
        finalState);
  }
  /** Scales feedforward when the turret is near mechanical limits */
  private double applyFeedforwardSafetyPadding(
      double currentAngleRadians, double commandedFeedforward) {
    double minLimitRadians = minTurretAngle.getRadians();
    double maxLimitRadians = maxTurretAngle.getRadians();
    double paddingRadians = feedforwardPaddingAngle.getRadians();

    if (commandedFeedforward > 0 && currentAngleRadians > (maxLimitRadians - paddingRadians)) {
      double distanceToLimit = maxLimitRadians - currentAngleRadians;
      double scaleFactor = MathUtil.clamp(distanceToLimit / paddingRadians, 0.0, 1.0);
      return commandedFeedforward * scaleFactor;
    }

    if (commandedFeedforward < 0 && currentAngleRadians < (minLimitRadians + paddingRadians)) {
      double distanceToLimit = currentAngleRadians - minLimitRadians;
      double scaleFactor = MathUtil.clamp(distanceToLimit / paddingRadians, 0.0, 1.0);
      return commandedFeedforward * scaleFactor;
    }

    return commandedFeedforward;
  }
  /**
   * Runs the Newton solver to converge the right time of flight so the launcher can shoot when
   * moving
   */
  private SolverState runNewtonSolver(
      Translation2d targetFieldPos,
      Rotation2d currentTurretAngle,
      double currentTurretVelocityRadPerSec,
      RobotPredictor predictor) {

    double timeFlightGuess = 0.5;
    SolverState bestState = null;
    /** Computes the solver state for the current guess for the current time of flight guess */
    for (int i = 0; i < MAX_SOLVER_ITERATIONS; i++) {
      SolverState stateCurrent =
          computePhysicsState(
              timeFlightGuess,
              targetFieldPos,
              currentTurretAngle,
              currentTurretVelocityRadPerSec,
              predictor);

      if (Math.abs(stateCurrent.errorSeconds) < CONVERGENCE_THRESHOLD_SECONDS) {
        return stateCurrent.markConverged();
      }
      /** Computes the solver state for the derivative probe */
      SolverState stateProbe =
          computePhysicsState(
              timeFlightGuess + DERIVATIVE_PROBE_TIME_DELTA,
              targetFieldPos,
              currentTurretAngle,
              currentTurretVelocityRadPerSec,
              predictor);

      double slope =
          (stateProbe.errorSeconds - stateCurrent.errorSeconds) / DERIVATIVE_PROBE_TIME_DELTA;

      if (Math.abs(slope) < 1e-5) slope = Math.signum(slope) * 1e-5;

      double newGuess = timeFlightGuess - (stateCurrent.errorSeconds / slope);
      timeFlightGuess = Math.max(0.01, newGuess);

      bestState = stateCurrent;
    }
    return bestState;
  }
  /** Computes and returns a solver state */
  private SolverState computePhysicsState(
      double timeFlightGuess,
      Translation2d targetFieldPos,
      Rotation2d currentTurretAngle,
      double currentTurretVelocityRadPerSec,
      RobotPredictor predictor) {

    RobotState stateNow = predictor.predict(0.0, 0.0);

    Translation2d estimatedVirtualTarget =
        targetFieldPos.minus(
            stateNow.velocity() != null
                ? new Translation2d(
                        stateNow.velocity().vxMetersPerSecond,
                        stateNow.velocity().vyMetersPerSecond)
                    .times(timeFlightGuess)
                : new Translation2d());
    /**
     * Computes the robot heading, turret offset, and vector from the turret to the estimated target
     */
    Rotation2d robotHeadingNow = stateNow.pose().getRotation();
    Translation2d turretOffsetNow = turretOffsetRobotFrame.rotateBy(robotHeadingNow);
    Translation2d vectorToEstimatedTarget =
        estimatedVirtualTarget.minus(stateNow.pose().getTranslation().plus(turretOffsetNow));
    /** Computes the angle to the estimated target */
    Rotation2d goalAngleLocal = getAngleFromVector(vectorToEstimatedTarget).minus(robotHeadingNow);
    double angleErrorRadians =
        Math.abs(MathUtil.angleModulus(goalAngleLocal.minus(currentTurretAngle).getRadians()));

    double estimatedSettlingTime =
        settlingTimeFunction.apply(angleErrorRadians, currentTurretVelocityRadPerSec)
            * settlingTimeGain;

    // Cap prediction lookahead to prevent heading over-extrapolation during fast rotation.
    // Linear heading prediction (heading += omega * dt) becomes unreliable beyond ~0.15s
    // and causes the solved turret goal to jump discontinuously when robot angular velocity
    // changes.
    double predictedSettlingTime = Math.min(estimatedSettlingTime, 0.15);

    RobotState stateAtFire = predictor.predict(0.0, predictedSettlingTime);
    Rotation2d headingAtFire = stateAtFire.pose().getRotation();
    Translation2d turretOffsetAtFire = turretOffsetRobotFrame.rotateBy(headingAtFire);

    double robotAngularVelocity = stateAtFire.velocity().omegaRadiansPerSecond;

    Translation2d tangentialVelocity =
        new Translation2d(
            -robotAngularVelocity * turretOffsetAtFire.getY(),
            robotAngularVelocity * turretOffsetAtFire.getX());

    Translation2d robotLinearVelocity =
        new Translation2d(
            stateAtFire.velocity().vxMetersPerSecond, stateAtFire.velocity().vyMetersPerSecond);

    Translation2d inheritedMuzzleVelocity = robotLinearVelocity.plus(tangentialVelocity);

    Translation2d virtualTargetPos =
        targetFieldPos.minus(inheritedMuzzleVelocity.times(timeFlightGuess));

    Translation2d gunPositionAtFire = stateAtFire.pose().getTranslation().plus(turretOffsetAtFire);
    Translation2d vectorToVirtualTarget = virtualTargetPos.minus(gunPositionAtFire);

    double distanceToVirtualTarget = vectorToVirtualTarget.getNorm();
    double requiredTimeOfFlight = timeOfFlightFunction.apply(distanceToVirtualTarget);

    double errorSeconds = timeFlightGuess - requiredTimeOfFlight;
    /** Returns a fully constructed solver state */
    return new SolverState(
        errorSeconds,
        requiredTimeOfFlight,
        virtualTargetPos,
        vectorToVirtualTarget,
        inheritedMuzzleVelocity,
        stateAtFire,
        false);
  }
  /** Computes the kinematic feed forward caused by robot rotation and acceleration */
  private double calculateKinematicFeedforward(SolverState state) {
    RobotState robotState = state.robotStateAtFire;
    Rotation2d robotHeading = robotState.pose().getRotation();
    Translation2d turretOffsetRotated = turretOffsetRobotFrame.rotateBy(robotHeading);

    double robotOmega = robotState.velocity().omegaRadiansPerSecond;
    double robotAlpha =
        robotState.acceleration() != null ? robotState.acceleration().omegaRadiansPerSecond : 0.0;

    Translation2d accelTangential =
        new Translation2d(
            -robotAlpha * turretOffsetRotated.getY(), robotAlpha * turretOffsetRotated.getX());

    Translation2d accelCentripetal = turretOffsetRotated.times(-(robotOmega * robotOmega));
    /**
     * Converts the robot's linear acceleration into a 2D vector and defaults it to 0 if no data is
     * available
     */
    Translation2d accelRobotLinear =
        robotState.acceleration() != null
            ? new Translation2d(
                robotState.acceleration().vxMetersPerSecond,
                robotState.acceleration().vyMetersPerSecond)
            : new Translation2d();

    Translation2d accelTurretMount = accelRobotLinear.plus(accelTangential).plus(accelCentripetal);

    Translation2d velocityVirtualTargetDrift = accelTurretMount.times(-state.requiredTimeOfFlight);

    Translation2d velocityRelative =
        velocityVirtualTargetDrift.minus(state.inheritedMuzzleVelocity);

    double distanceSquared = Math.pow(state.vectorToVirtualTarget.getNorm(), 2);

    if (distanceSquared < 1e-4) return 0.0;

    double crossProduct =
        (state.vectorToVirtualTarget.getX() * velocityRelative.getY())
            - (state.vectorToVirtualTarget.getY() * velocityRelative.getX());

    double omegaFieldRelative = crossProduct / distanceSquared;

    return omegaFieldRelative - robotOmega;
  }

  private Rotation2d getAngleFromVector(Translation2d vec) {
    return new Rotation2d(vec.getX(), vec.getY());
  }
  /** Packages physics data from the newton solver into one immutable object */
  public record SolverState(
      double errorSeconds,
      double requiredTimeOfFlight,
      Translation2d virtualTargetFieldPos,
      Translation2d vectorToVirtualTarget,
      Translation2d inheritedMuzzleVelocity,
      RobotState robotStateAtFire,
      boolean hasConverged) {
    /** Returns the new solver state that has converged */
    public SolverState markConverged() {
      return new SolverState(
          errorSeconds,
          requiredTimeOfFlight,
          virtualTargetFieldPos,
          vectorToVirtualTarget,
          inheritedMuzzleVelocity,
          robotStateAtFire,
          true);
    }
  }
}
