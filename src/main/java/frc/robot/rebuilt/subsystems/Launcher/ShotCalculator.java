// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.subsystems.Launcher.TurretControlPhysics.AimingSolution;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.frc5010.common.constants.Constants;
import org.frc5010.common.utils.geometry.AllianceFlipUtil;
import org.frc5010.common.utils.geometry.GeomUtil;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
/** Calculates the turret and hood angle, and the flywheel speed for shooting */
public class ShotCalculator {
  private static ShotCalculator instance;

  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

  private Rotation2d lastTurretAngle;
  private double lastHoodAngle;
  private Rotation2d turretAngle;
  private double hoodAngle = Double.NaN;
  private double turretVelocity;
  private double hoodVelocity;
  private TurretControlPhysics turretControlPhysics;
  private Translation2d cachedTurretOffset;
  private Rotation2d minTurretAngle = Rotation2d.fromDegrees(-165.0);
  private Rotation2d maxTurretAngle = Rotation2d.fromDegrees(165.0);
  private Rotation2d feedforwardPaddingAngle = Rotation2d.fromDegrees(10.0);
  private double settlingGain = 0.00;
  // Default turret motion constraints (overridden via setTurretMotionConstraints).
  // These represent the practical maximum velocity (360 °/s) and acceleration (720 °/s²)
  // until real SysId values are provided.
  private double turretMaxVelocityRadPerSec = Math.toRadians(360.0);
  private double turretMaxAccelRadPerSecSq = Math.toRadians(720.0);
  private BiFunction<Double, Double, Double> settlingTimeFunction =
      TurretControlPhysics.velocityAwareSettlingTimeFunction(
          turretMaxVelocityRadPerSec, turretMaxAccelRadPerSecSq);
  private DoubleSupplier turretVelocitySupplier = () -> 0.0;
  private final String targetName = "Target";
  private final String lookAhead = "Lookahead";
  private final String virtualTarget = "VirtualTarget";
  private final String turret = "Turret";

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }
  /** Stores calculated shooting parameters */
  public record ShootingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed,
      Distance distanceToVirtualTarget,
      AimingSolution solution) {}

  // Cache parameters
  private ShootingParameters latestParameters = null;
  public static double flywheelMultiplier = 1.04;

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
  private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  public record ShotTables(
      Map<Double, Rotation2d> hoodAngles,
      Map<Double, Double> flywheelSpeeds,
      Map<Double, Double> timeOfFlightSeconds,
      double minDistanceMeters,
      double maxDistanceMeters,
      double phaseDelaySeconds) {
    /** returns a copy of ShotTables with updated phase delay */
    public ShotTables withPhaseDelaySeconds(double newPhaseDelaySeconds) {
      return new ShotTables(
          hoodAngles,
          flywheelSpeeds,
          timeOfFlightSeconds,
          minDistanceMeters,
          maxDistanceMeters,
          newPhaseDelaySeconds);
    }
  }

  public static void incrementFlywheelMultiplier(double amount) {
    ShotCalculator.flywheelMultiplier += amount;
  }

  public static double getFlywheelMultiplier() {
    return ShotCalculator.flywheelMultiplier;
  }
  /** Stores configuration values for generating ballistic shot tables */
  public record BallisticConfig(
      double minDistanceMeters,
      double maxDistanceMeters,
      double distanceStepMeters,
      Rotation2d minHoodAngle,
      Rotation2d maxHoodAngle,
      Rotation2d hoodAngleStep,
      double minFlywheelRadPerSec,
      double maxFlywheelRadPerSec,
      double wheelRadiusMeters,
      double launchHeightMeters,
      double targetHeightMeters,
      double phaseDelaySeconds,
      double gravityMetersPerSecondSquared,
      double hoodAngleReferenceRadians) {}

  static {
    applyShotTables(createDefaultTables());
  }
  /** Creates default hood angle, flywheel speeds, and time of light tables */
  public static ShotTables createDefaultTables() {
    final double offset = 0.5969;
    return new ShotTables(
        Map.ofEntries(
            Map.entry(2.0796297600462808, Rotation2d.fromDegrees(33.0)),
            Map.entry(2.3644814757115706, Rotation2d.fromDegrees(34.0)),
            Map.entry(2.7648598116063776, Rotation2d.fromDegrees(35.0)),
            Map.entry(3.2194779503261004, Rotation2d.fromDegrees(38.0)),
            Map.entry(3.53088512698516, Rotation2d.fromDegrees(39.0)),
            Map.entry(3.9268571046742813, Rotation2d.fromDegrees(40.0)),
            Map.entry(4.317290273504823, Rotation2d.fromDegrees(42.0)),
            Map.entry(4.540307519714445, Rotation2d.fromDegrees(43.0)),
            Map.entry(5.77893560525366, Rotation2d.fromDegrees(45.0)),
            Map.entry(6.35214199070158, Rotation2d.fromDegrees(47.0)),
            Map.entry(10.990685758149123, Rotation2d.fromDegrees(50.0)),
            Map.entry(13.024035615135324, Rotation2d.fromDegrees(55.0))),
        Map.ofEntries(
            Map.entry(2.0796297600462808, 97.0),
            Map.entry(2.3644814757115706, 103.0),
            Map.entry(2.7648598116063776, 105.0),
            Map.entry(3.2194779503261004, 107.0),
            Map.entry(3.53088512698516, 110.0),
            Map.entry(3.9268571046742813, 113.0),
            Map.entry(4.317290273504823, 115.0),
            Map.entry(4.540307519714445, 117.0),
            Map.entry(5.77893560525366, 127.0),
            Map.entry(6.35214199070158, 133.0),
            Map.entry(10.990685758149123, 155.0),
            Map.entry(13.024035615135324, 170.0)),
        Map.ofEntries(
            Map.entry(2.11, 1.04),
            Map.entry(3.92, 1.19),
            Map.entry(4.10, 1.22),
            Map.entry(5.58, 1.28)),
        0.7,
        100.0,
        0.03);
  }

  public static ShotTables createBallisticTables(BallisticConfig config) {
    if (config == null || config.wheelRadiusMeters() <= 0.0) {
      return createDefaultTables();
    }

    Map<Double, Rotation2d> hoodAngles = new TreeMap<>();
    Map<Double, Double> flywheelSpeeds = new TreeMap<>();
    Map<Double, Double> timeOfFlight = new TreeMap<>();

    double minValid = Double.POSITIVE_INFINITY;
    double maxValid = 0.0;
    double minDistance = config.minDistanceMeters();
    double maxDistance = config.maxDistanceMeters();
    double distanceStep = Math.max(config.distanceStepMeters(), 0.05);
    double angleStep = Math.max(config.hoodAngleStep().getRadians(), Math.toRadians(0.25));
    double hoodReference = config.hoodAngleReferenceRadians();
    double minAngle = hoodReference - config.maxHoodAngle().getRadians();
    double maxAngle = hoodReference - config.minHoodAngle().getRadians();
    double gravity = config.gravityMetersPerSecondSquared();
    double heightDelta = config.targetHeightMeters() - config.launchHeightMeters();

    for (double distance = minDistance; distance <= maxDistance + 1e-6; distance += distanceStep) {
      BallisticSolution solution =
          solveBallistic(
              distance,
              heightDelta,
              gravity,
              minAngle,
              maxAngle,
              angleStep,
              config.wheelRadiusMeters(),
              config.minFlywheelRadPerSec(),
              config.maxFlywheelRadPerSec());
      if (solution == null) {
        continue;
      }
      hoodAngles.put(distance, Rotation2d.fromRadians(hoodReference - solution.angleRadians()));
      flywheelSpeeds.put(distance, solution.flywheelRadPerSec());
      timeOfFlight.put(distance, solution.timeOfFlightSeconds());
      minValid = Math.min(minValid, distance);
      maxValid = Math.max(maxValid, distance);
    }

    if (hoodAngles.isEmpty()) {
      return createDefaultTables();
    }

    double minRange = Double.isFinite(minValid) ? minValid : minDistance;
    double maxRange = maxValid > 0.0 ? maxValid : maxDistance;

    return new ShotTables(
        hoodAngles, flywheelSpeeds, timeOfFlight, minRange, maxRange, config.phaseDelaySeconds());
  }

  private record BallisticSolution(
      double angleRadians, double flywheelRadPerSec, double timeOfFlightSeconds) {}

  private static BallisticSolution solveBallistic(
      double distanceMeters,
      double heightDeltaMeters,
      double gravityMetersPerSecondSquared,
      double minAngleRadians,
      double maxAngleRadians,
      double angleStepRadians,
      double wheelRadiusMeters,
      double minFlywheelRadPerSec,
      double maxFlywheelRadPerSec) {
    double bestFlywheel = Double.POSITIVE_INFINITY;
    double bestAngle = 0.0;
    double bestTime = 0.0;

    for (double angle = minAngleRadians;
        angle <= maxAngleRadians + 1e-6;
        angle += angleStepRadians) {
      double cos = Math.cos(angle);
      double tan = Math.tan(angle);
      double denominator = distanceMeters * tan - heightDeltaMeters;
      if (denominator <= 0.0 || Math.abs(cos) < 1e-6) {
        continue;
      }

      double velocitySquared =
          gravityMetersPerSecondSquared
              * distanceMeters
              * distanceMeters
              / (2.0 * cos * cos * denominator);
      if (velocitySquared <= 0.0) {
        continue;
      }

      double velocity = Math.sqrt(velocitySquared);
      double flywheelRadPerSec = velocity / wheelRadiusMeters;
      if (flywheelRadPerSec < minFlywheelRadPerSec || flywheelRadPerSec > maxFlywheelRadPerSec) {
        continue;
      }

      if (flywheelRadPerSec < bestFlywheel) {
        bestFlywheel = flywheelRadPerSec;
        bestAngle = angle;
        bestTime = distanceMeters / (velocity * cos);
      }
    }

    if (!Double.isFinite(bestFlywheel)) {
      return null;
    }

    return new BallisticSolution(bestAngle, bestFlywheel, bestTime);
  }

  /**
   * Get the interpolated hood angle (degrees) from the current lookup table for a given distance.
   *
   * @param distanceMeters distance to target in meters
   * @return hood angle in degrees from the lookup table, or NaN if unavailable
   */
  public double getLookupHoodAngleDegrees(double distanceMeters) {
    Rotation2d value = shotHoodAngleMap.get(distanceMeters);
    return value != null ? value.getDegrees() : Double.NaN;
  }

  public boolean hasValidShot() {
    if (latestParameters != null) {
      return latestParameters.solution.isPossible();
    }
    return false;
  }

  /**
   * Get the interpolated flywheel speed from the current lookup table for a given distance.
   *
   * @param distanceMeters distance to target in meters
   * @return flywheel speed from the lookup table, or NaN if unavailable
   */
  public double getLookupFlywheelSpeed(double distanceMeters) {
    Double value = shotFlywheelSpeedMap.get(distanceMeters);
    return value != null ? value : Double.NaN;
  }

  /**
   * Compute a ballistic guess for the given distance using the stored ballistic config. Returns
   * null if no ballistic config has been provided.
   *
   * @param distanceMeters distance to target in meters
   * @return a double[] of {hoodAngleDegrees, flywheelSpeed, timeOfFlightSeconds}, or null
   */
  public double[] getBallisticGuess(double distanceMeters) {
    if (ballisticConfig == null) {
      return null;
    }
    double hoodReference = ballisticConfig.hoodAngleReferenceRadians();
    double minAngle = hoodReference - ballisticConfig.maxHoodAngle().getRadians();
    double maxAngle = hoodReference - ballisticConfig.minHoodAngle().getRadians();
    double angleStep = Math.max(ballisticConfig.hoodAngleStep().getRadians(), Math.toRadians(0.25));
    double heightDelta =
        ballisticConfig.targetHeightMeters() - ballisticConfig.launchHeightMeters();
    BallisticSolution solution =
        solveBallistic(
            distanceMeters,
            heightDelta,
            ballisticConfig.gravityMetersPerSecondSquared(),
            minAngle,
            maxAngle,
            angleStep,
            ballisticConfig.wheelRadiusMeters(),
            ballisticConfig.minFlywheelRadPerSec(),
            ballisticConfig.maxFlywheelRadPerSec());
    if (solution == null) {
      return null;
    }
    double hoodAngleDegrees = Math.toDegrees(hoodReference - solution.angleRadians());
    return new double[] {
      hoodAngleDegrees, solution.flywheelRadPerSec(), solution.timeOfFlightSeconds()
    };
  }

  /**
   * Add or update a single data point in the live lookup tables.
   *
   * @param distanceMeters the distance key
   * @param hoodAngleDegrees hood angle in degrees
   * @param flywheelSpeed flywheel speed value
   * @param timeOfFlightSeconds estimated time-of-flight in seconds
   */
  public void addDataPoint(
      double distanceMeters,
      double hoodAngleDegrees,
      double flywheelSpeed,
      double timeOfFlightSeconds) {
    shotHoodAngleMap.put(distanceMeters, Rotation2d.fromDegrees(hoodAngleDegrees));
    shotFlywheelSpeedMap.put(distanceMeters, flywheelSpeed);
    timeOfFlightMap.put(distanceMeters, timeOfFlightSeconds);
    if (distanceMeters < minDistance) {
      minDistance = distanceMeters;
    }
    if (distanceMeters > maxDistance) {
      maxDistance = distanceMeters;
    }
    latestParameters = null;
    turretControlPhysics = null;
  }

  /** Store a ballistic config so that ballistic guesses can be computed on demand. */
  private BallisticConfig ballisticConfig;

  public void setBallisticConfig(BallisticConfig config) {
    this.ballisticConfig = config;
  }

  public BallisticConfig getBallisticConfig() {
    return ballisticConfig;
  }

  public void setShotTables(ShotTables tables) {
    applyShotTables(tables);
    latestParameters = null;
    turretControlPhysics = null;
  }

  private static void applyShotTables(ShotTables tables) {
    if (tables == null) {
      return;
    }
    shotHoodAngleMap.clear();
    tables.hoodAngles().forEach(shotHoodAngleMap::put);
    shotFlywheelSpeedMap.clear();
    tables.flywheelSpeeds().forEach(shotFlywheelSpeedMap::put);
    timeOfFlightMap.clear();
    tables.timeOfFlightSeconds().forEach(timeOfFlightMap::put);
    minDistance = tables.minDistanceMeters();
    maxDistance = tables.maxDistanceMeters();
    phaseDelay = tables.phaseDelaySeconds();
  }

  public void setTurretConstraints(
      Rotation2d minAngle, Rotation2d maxAngle, Rotation2d paddingAngle) {
    if (minAngle != null) {
      minTurretAngle = minAngle;
    }
    if (maxAngle != null) {
      maxTurretAngle = maxAngle;
    }
    if (paddingAngle != null) {
      feedforwardPaddingAngle = paddingAngle;
    }
    turretControlPhysics = null;
  }

  public void setSettlingTimeFunction(
      BiFunction<Double, Double, Double> function, double newSettlingGain) {
    if (function != null) {
      settlingTimeFunction = function;
    }
    settlingGain = newSettlingGain;
    turretControlPhysics = null;
  }

  /**
   * Configures the turret settling-time function using the closed-form trapezoidal motion profile.
   *
   * <p>This replaces any previously supplied {@link #setSettlingTimeFunction} with the analytical
   * result derived from the given peak velocity and acceleration. Call this once during robot init
   * after measuring the turret's actual motion profile constraints via SysId.
   *
   * @param maxVelocityRadPerSec peak turret velocity in rad/s
   * @param maxAccelRadPerSecSq peak turret acceleration in rad/s²
   * @param newSettlingGain multiplier applied to the computed time (use ≤ 1.0 to avoid oscillation)
   */
  public void setTurretMotionConstraints(
      double maxVelocityRadPerSec, double maxAccelRadPerSecSq, double newSettlingGain) {
    turretMaxVelocityRadPerSec = maxVelocityRadPerSec;
    turretMaxAccelRadPerSecSq = maxAccelRadPerSecSq;
    settlingTimeFunction =
        TurretControlPhysics.velocityAwareSettlingTimeFunction(
            turretMaxVelocityRadPerSec, turretMaxAccelRadPerSecSq);
    settlingGain = newSettlingGain;
    turretControlPhysics = null;
  }

  /**
   * Sets the supplier for the current turret angular velocity. This is used by the velocity-aware
   * settling time function to account for turret momentum when predicting time-to-arrival.
   *
   * @param supplier a {@link DoubleSupplier} returning the current turret velocity in rad/s
   */
  public void setTurretVelocitySupplier(DoubleSupplier supplier) {
    if (supplier != null) {
      turretVelocitySupplier = supplier;
    }
  }

  public ShootingParameters getParameters(
      Translation2d turretRelativePosition,
      Rotation2d turretRelativeAngle,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Translation2d> targetPositionSupplier) {
    if (latestParameters != null) {
      return latestParameters;
    }

    // Snapshot current pose, field-relative velocity, and field-relative acceleration.
    // Field velocity is used for linear extrapolation: x += vx*dt, y += vy*dt, heading += omega*dt.
    // This matches the actual drivetrain behavior since the drive code already accounts for
    // curvature (discretize) when commanding inputs to drive straight in field frame.
    Pose2d estimatedPose = robotPoseSupplier.get();
    ChassisSpeeds fieldVelocity = Rebuilt.drivetrain.getFieldVelocity();
    ChassisSpeeds fieldAcceleration = Rebuilt.drivetrain.getFieldAcceleration();

    // Apply phase delay using linear field-frame extrapolation.
    Pose2d phaseDelayedPose = linearExtrapolatePose(estimatedPose, fieldVelocity, phaseDelay);

    Translation2d target = AllianceFlipUtil.apply(targetPositionSupplier.get());
    Pose2d turretPosition =
        phaseDelayedPose.transformBy(
            new Transform2d(
                turretRelativePosition.getMeasureX(),
                turretRelativePosition.getMeasureY(),
                turretRelativeAngle));

    TurretControlPhysics physics = getTurretControlPhysics(turretRelativePosition);
    double currentTurretVelocityRadPerSec = turretVelocitySupplier.getAsDouble();
    TurretControlPhysics.AimingSolution solution =
        physics.solve(
            target,
            turretRelativeAngle,
            currentTurretVelocityRadPerSec,
            (timeSinceStartSeconds, lookaheadSeconds) -> {
              // Linear field-frame pose extrapolation: no twist, just straight-line translation
              // in the field frame plus proportional heading change.
              Pose2d predictedPose =
                  linearExtrapolatePose(phaseDelayedPose, fieldVelocity, lookaheadSeconds);
              return new TurretControlPhysics.RobotState(
                  predictedPose, fieldVelocity, fieldAcceleration);
            });

    double distanceToVirtualTarget = solution.effectiveDistanceMeters();
    Rotation2d hoodSetpoint = shotHoodAngleMap.get(distanceToVirtualTarget);
    Double flywheelSpeed = shotFlywheelSpeedMap.get(distanceToVirtualTarget);

    turretAngle = solution.turretLocalHeading();
    hoodAngle = hoodSetpoint != null ? hoodSetpoint.getRadians() : 0.0;
    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    turretVelocity =
        turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getRadians() / Constants.loopPeriodSecs);
    hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;
    latestParameters =
        new ShootingParameters(
            solution.isPossible(),
            turretAngle,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            flywheelSpeed != null ? flywheelSpeed : 0.0,
            Meters.of(distanceToVirtualTarget),
            solution);

    // Log calculated values
    Logger.recordOutput("ShotCalculator/AimingStatus", solution.status().toString());
    Logger.recordOutput(
        "ShotCalculator/TurretToTargetDistance", solution.effectiveDistanceMeters());
    Logger.recordOutput(
        "ShotCalculator/VirtualTargetFieldPosition",
        new Pose2d(solution.finalSolverState().virtualTargetFieldPos(), turretAngle));
    Logger.recordOutput("ShotCalculator/FieldVelocity", fieldVelocity);
    Logger.recordOutput("ShotCalculator/FieldAcceleration", fieldAcceleration);

    Rebuilt.drivetrain
        .getField2d()
        .getObject(targetName)
        .setPose(new Pose2d(target, target.getAngle()));
    // Lookahead visualisation uses the same linear extrapolation
    Pose2d lookaheadRobotPose =
        linearExtrapolatePose(phaseDelayedPose, fieldVelocity, solution.estimatedTimeOfFlight());
    Pose2d lookaheadTurretPose =
        lookaheadRobotPose.transformBy(
            new Transform2d(
                turretRelativePosition.getMeasureX(),
                turretRelativePosition.getMeasureY(),
                turretRelativeAngle));
    Rebuilt.drivetrain.getField2d().getObject(lookAhead).setPose(lookaheadTurretPose);
    Pose2d virtualTargetPose = new Pose2d(solution.virtualTargetFieldPos(), turretAngle);
    Rebuilt.drivetrain.getField2d().getObject(virtualTarget).setPose(virtualTargetPose);
    Rebuilt.drivetrain.getField2d().getObject(turret).setPose(turretPosition);

    return latestParameters;
  }

  /**
   * Extrapolates a robot pose forward in time using linear field-frame velocity components.
   *
   * <p>The robot's field-frame position advances by {@code vx * dt} and {@code vy * dt}. Heading
   * advances by {@code omega * dt}. This avoids the curvature error introduced by {@link
   * Pose2d#exp(Twist2d)} because the drive code already compensates for curvature when generating
   * robot-relative motor commands (via {@code ChassisSpeeds.discretize}).
   */
  private static Pose2d linearExtrapolatePose(
      Pose2d currentPose, ChassisSpeeds fieldVelocity, double dt) {
    Translation2d newTranslation =
        currentPose
            .getTranslation()
            .plus(
                new Translation2d(
                    fieldVelocity.vxMetersPerSecond * dt, fieldVelocity.vyMetersPerSecond * dt));
    Rotation2d newRotation =
        currentPose
            .getRotation()
            .plus(Rotation2d.fromRadians(fieldVelocity.omegaRadiansPerSecond * dt));
    return new Pose2d(newTranslation, newRotation);
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }

  private TurretControlPhysics getTurretControlPhysics(Translation2d turretOffset) {
    if (turretControlPhysics == null
        || cachedTurretOffset == null
        || !cachedTurretOffset.equals(turretOffset)) {
      cachedTurretOffset = turretOffset;
      turretControlPhysics =
          new TurretControlPhysics(
              turretOffset,
              minTurretAngle,
              maxTurretAngle,
              feedforwardPaddingAngle,
              settlingGain,
              this::getTimeOfFlightSeconds,
              settlingTimeFunction,
              minDistance,
              maxDistance);
    }
    return turretControlPhysics;
  }

  private double getTimeOfFlightSeconds(double distanceMeters) {
    Double time = timeOfFlightMap.get(distanceMeters);
    return time != null ? time : 0.0;
  }
}
