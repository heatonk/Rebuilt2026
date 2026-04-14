// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.auto.pathplanner.PathFinderCommand;
import org.frc5010.common.commands.DriveToPoseSupplier;
import org.frc5010.common.commands.JoystickToSwerve;
import org.frc5010.common.constants.GenericDrivetrainConstants;
import org.frc5010.common.constants.RobotConstantsDef;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.drive.swerve_utils.PathConstraints5010;
import org.frc5010.common.drive.swerve_utils.SwerveSetpointGenerator5010;
import org.frc5010.common.sensors.Controller;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Add your docs here. */
public class GenericSwerveDrivetrain extends GenericDrivetrain {

  private SwerveDriveFunctions swerveDrive = null;
  private GenericDrivetrainConstants swerveConstants;

  public GenericSwerveDrivetrain(
      LoggedMechanism2d mechVisual,
      GenericDrivetrainConstants swerveConstants,
      SwerveDriveFunctions swerveDriveFunctions) {
    super(mechVisual);
    this.swerveDrive = swerveDriveFunctions;
    this.swerveConstants = swerveConstants;

    ppRobotConfigSupplier = swerveDrive.getPPRobotConfigSupplier();
    setDrivetrainPoseEstimator(swerveDrive.initializePoseEstimator());
    initializeSimulation(swerveConstants);
    driveTrainSimulationSupplier = swerveDrive.getDriveTrainSimulationSupplier();
  }

  @Override
  public void driveWithFeedforwards(ChassisSpeeds direction, DriveFeedforwards feedforwards) {
    swerveDrive.drive(direction, feedforwards);
  }

  @Override
  public void drive(ChassisSpeeds direction) {
    swerveDrive.driveRobotRelative(direction);
  }

  @Override
  public void setAutoBuilder() {
    setupPathPlanner();
  }

  /** Setup AutoBuilder for PathPlanner. */
  public void setupPathPlanner() {
    AutoBuilder.configure(
        poseEstimator::getCurrentPose, // Robot pose supplier
        poseEstimator
            ::resetToPose, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeedsWithAngleSupplier,
        // RELATIVE ChassisSpeeds
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for
            // holonomic
            // drive trains
            new PIDConstants(4, 0, 0), // Translation PID constants
            new PIDConstants(1.0, 0, 0) // Rotation PID constants
            ),
        ppRobotConfigSupplier.get(), // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          return GenericRobot.getAlliance() == DriverStation.Alliance.Red;
        },
        this // Reference to this subsystem to set requirements
        );

    // Preload PathPlanner Path finding.
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    warmupPathfinding();
  }

  /**
   * Schedules the PathPlanner pathfinding warmup command. Safe to call during robotInit() because
   * the command is decorated with ignoringDisable(true), which lets it run while the robot is
   * disabled. Calling this early prevents the ~180 ms overrun that occurs when warmup finishes
   * mid-match the first time a PathfindingCommand is instantiated.
   */
  public void warmupPathfinding() {
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
  }

  @Override
  public void addAutoCommands(LoggedDashboardChooser<Command> selectableCommand) {
    swerveDrive.addAutoCommands(selectableCommand, this);
  }

  @Override
  public void updateGlassWidget() {
    GenericSwerveModuleInfo[] modules = swerveDrive.getModulesInfo();
    for (int moduleKey = 0; moduleKey < modules.length; moduleKey++) {
      double turningDeg = modules[moduleKey].steerRelativeDegrees;
      double absEncDeg = modules[moduleKey].steerAbsoluteDegrees;
      // This method will be called once per scheduler run
      absEncDials.get(moduleKey).setAngle(absEncDeg + 90);
      motorDials.get(moduleKey).setAngle(turningDeg + 90);
      motorDials
          .get(moduleKey)
          .setLength(0.1 * modules[moduleKey].steerVelocityDegreesPerSecond + 0.02);
      expectDials
          .get(moduleKey)
          .setLength(0.1 * modules[moduleKey].driveVelocityMetersPerSecond + 0.02);
      expectDials.get(moduleKey).setAngle(modules[moduleKey].expectedSteerDegrees + 90);
    }
  }

  public Field2d getField2d() {
    return swerveDrive.getField2d();
  }

  public GenericDrivetrainConstants getSwerveConstants() {
    return swerveConstants;
  }

  @Override
  protected ChassisSpeeds getChassisSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  @Override
  public void initGlassWidget(GenericDrivetrainConstants constants) {
    SmartDashboard.putData("Drive Visual", mechanismSimulation);
    GenericSwerveModuleInfo[] modules = swerveDrive.getModulesInfo();

    visualRoots.put(
        0,
        mechanismSimulation.getRoot(
            "frontleft",
            RobotConstantsDef.robotVisualH * constants.getWheelBase().in(Meters) / 2.0
                + RobotConstantsDef.robotVisualH / 2.0,
            RobotConstantsDef.robotVisualV * constants.getWheelBase().in(Meters) / 2.0
                + RobotConstantsDef.robotVisualV / 2.0));
    visualRoots.put(
        1,
        mechanismSimulation.getRoot(
            "frontright",
            RobotConstantsDef.robotVisualH * constants.getWheelBase().in(Meters) / 2.0
                + RobotConstantsDef.robotVisualH / 2.0,
            RobotConstantsDef.robotVisualV * -constants.getWheelBase().in(Meters) / 2.0
                + RobotConstantsDef.robotVisualV / 2.0));
    visualRoots.put(
        2,
        mechanismSimulation.getRoot(
            "backleft",
            RobotConstantsDef.robotVisualH * -constants.getWheelBase().in(Meters) / 2.0
                + RobotConstantsDef.robotVisualH / 2.0,
            RobotConstantsDef.robotVisualV * constants.getWheelBase().in(Meters) / 2.0
                + RobotConstantsDef.robotVisualV / 2.0));
    visualRoots.put(
        3,
        mechanismSimulation.getRoot(
            "backright",
            RobotConstantsDef.robotVisualH * -constants.getWheelBase().in(Meters) / 2.0
                + RobotConstantsDef.robotVisualH / 2.0,
            RobotConstantsDef.robotVisualV * -constants.getWheelBase().in(Meters) / 2.0
                + RobotConstantsDef.robotVisualV / 2.0));
    for (int i = 0; i < modules.length; i++) {
      visualRoots
          .get(i)
          .append(
              new LoggedMechanismLigament2d(i + "-vert", 0.10, 90, 6.0, new Color8Bit(50, 50, 50)));
      visualRoots
          .get(i)
          .append(
              new LoggedMechanismLigament2d(i + "-hori", 0.10, 0, 6.0, new Color8Bit(50, 50, 50)));
      motorDials.put(
          i,
          visualRoots
              .get(i)
              .append(
                  new LoggedMechanismLigament2d(
                      i + "-motor", 0.10, 90, 6.0, new Color8Bit(Color.kYellow))));
      absEncDials.put(
          i,
          visualRoots
              .get(i)
              .append(
                  new LoggedMechanismLigament2d(
                      i + "-Abs", 0.50, 90, 6, new Color8Bit(Color.kBlue))));
      expectDials.put(
          i,
          visualRoots
              .get(i)
              .append(
                  new LoggedMechanismLigament2d(
                      i + "-Exp", 0.10, 90, 6, new Color8Bit(Color.kRed))));
    }
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the maximum angular velocity of an azimuth/angle motor in the swerve module.
   *
   * @return {@link AngularVelocity} of the maximum azimuth/angle motor.
   */
  public AngularVelocity getMaximumModuleAngleVelocity() {
    return swerveDrive.getMaximumModuleAngleVelocity();
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current field-relative chassis acceleration derived from drive motor acceleration
   * signals. Returns zero if the underlying drive implementation does not support acceleration
   * signals.
   *
   * @return A ChassisSpeeds object representing field-relative acceleration (m/s² components)
   */
  public ChassisSpeeds getFieldAcceleration() {
    if (swerveDrive instanceof AkitSwerveDrive) {
      return ((AkitSwerveDrive) swerveDrive).getFieldAcceleration();
    }
    return new ChassisSpeeds();
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void setChassisSpeedsWithAngleSupplier(
      ChassisSpeeds chassisSpeeds, DriveFeedforwards moduleFeedForwards) {
    ChassisSpeeds angleSuppliedChassisSpeeds =
        new ChassisSpeeds(
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            null != angleSpeedSupplier
                ? angleSpeedSupplier.getAsDouble()
                : chassisSpeeds.omegaRadiansPerSecond);

    swerveDrive.drive(angleSuppliedChassisSpeeds, moduleFeedForwards);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(
        () -> {
          swerveDrive.driveFieldOriented(velocity.get());
        });
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(
            getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond(),
            getSwerveConstants().getkTeleDriveMaxAccelerationUnitsPerSecond(),
            getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond(),
            getSwerveConstants().getkTeleDriveMaxAngularAccelerationUnitsPerSecond());
    // PathConstraints constraints = new PathConstraints(
    // swerveDrive.getMaximumChassisVelocity(), 4.0,
    // swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose, constraints, 0.0 // Goal end velocity in meters/sec
        );
  }

  public Supplier<Command> driveToPosePrecise(Supplier<Pose2d> pose, Transform2d PathPlanOffset) {
    return driveToPosePrecise(pose, PathPlanOffset, Seconds.of(100));
  }

  public Supplier<Command> driveToPosePrecise(
      Supplier<Pose2d> pose, Transform2d PathPlanOffset, Time timeout) {
    Supplier<Pose3d> pose3D = () -> new Pose3d(pose.get());
    Supplier<DriveToPoseSupplier> finishDriving =
        () ->
            new DriveToPoseSupplier(
                    this, this::getPose, () -> pose3D.get().toPose2d(), new Transform2d(), 4.3)
                .withInitialVelocity(() -> getFieldVelocity()); // TO-DO: Change to
    // Distance-Based PID+

    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(
            getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond() * 1.0,
            3.5,
            getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond(),
            getSwerveConstants().getkTeleDriveMaxAngularAccelerationUnitsPerSecond());

    return () ->
        new PathFinderCommand(
                pose.get().transformBy(PathPlanOffset),
                constraints,
                0.0,
                this::getPose,
                this::getFieldVelocity,
                this::setChassisSpeedsWithAngleSupplier,
                new PPHolonomicDriveController( // PPHolonomicController is the built in path
                    // following controller for
                    // holonomic
                    // drive trains
                    new PIDConstants(2.0, 0, 0.0), // Translation PID constants
                    new PIDConstants(1.0, 0, 0.0) // Rotation PID constants
                    ),
                ppRobotConfigSupplier.get(),
                this)
            .beforeStarting(
                () -> {
                  poseEstimator.setTargetPoseOnField(
                      pose.get().transformBy(PathPlanOffset), "Auto Drive Pose");
                  SmartDashboard.putBoolean("Running AutoDrive", true);
                })
            .until(() -> getPose().getTranslation().getDistance(pose.get().getTranslation()) < 0.5)
            // .until(() -> RobotModel.circularObstacleWillBeAvoided(obstaclePosition,
            // poseEstimator::getCurrentPose,
            // pose.get(), unavoidableVertices, obstacleRadius, robotRadius,
            // maxRobotDimensionDeviation,
            // maxObstacleDimensionDeviation, obstacleAvoidanceResolution))
            // .until(() -> RobotModel.robotHasLinearPath(getPose(), pose.get(),
            // unavoidableVertices,
            // getSwerveConstants().getTrackWidth(), getSwerveConstants().getWheelBase()))
            .andThen(
                finishDriving
                    .get()
                    .withTimeout(timeout)
                    .beforeStarting(
                        () -> {
                          poseEstimator.setTargetPoseOnField(pose.get(), "Auto Drive Pose");
                        }))
            .finallyDo(
                () -> {
                  SmartDashboard.putBoolean("Running AutoDrive", false);
                });
  }

  public Supplier<Command> driveToPoseAuton(
      Supplier<Pose2d> pose, Transform2d PathPlanOffset, Time timeout) {
    PathConstraints constraints =
        new PathConstraints(
            getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond() * 1.0,
            8.0,
            getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond(),
            getSwerveConstants().getkTeleDriveMaxAngularAccelerationUnitsPerSecond());
    return driveToPoseAuton(pose, PathPlanOffset, timeout, constraints);
  }

  public Supplier<Command> newDriveToPoseAuton(
      Supplier<Pose2d> pose,
      Transform2d PathPlanOffset,
      Time timeout,
      double maxAcceleration,
      DoubleSupplier distance,
      DoubleSupplier stoppingDistance) {
    PathConstraints constraints =
        new PathConstraints(
            getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond() * 1.0,
            8.0,
            getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond(),
            getSwerveConstants().getkTeleDriveMaxAngularAccelerationUnitsPerSecond());
    return newDriveToPoseAuton(
        pose, PathPlanOffset, timeout, constraints, maxAcceleration, distance, stoppingDistance);
  }

  public Supplier<Command> driveToPoseAuton(
      Supplier<Pose2d> pose,
      Transform2d PathPlanOffset,
      Time timeout,
      PathConstraints driveConstraints) {
    Supplier<Pose3d> pose3D = () -> new Pose3d(pose.get());
    Supplier<DriveToPoseSupplier> finishDriving =
        () ->
            new DriveToPoseSupplier(
                    this, this::getPose, () -> pose3D.get().toPose2d(), new Transform2d(), 4.3)
                .withInitialVelocity(() -> getFieldVelocity()); // TO-DO: Change to
    // Distance-Based PID+
    // Create the constraints to use while pathfinding
    // PathConstraints constraints = new PathConstraints(
    // swerveDrive.getMaximumChassisVelocity(), 4.0,
    // swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return () ->
        new PathFinderCommand(
                pose.get().transformBy(PathPlanOffset),
                driveConstraints,
                0.0,
                this::getPose,
                this::getFieldVelocity,
                this::setChassisSpeedsWithAngleSupplier,
                new PPHolonomicDriveController( // PPHolonomicController is the built in path
                    // following controller for
                    // holonomic
                    // drive trains
                    new PIDConstants(4.0, 0, 0.0), // Translation PID constants
                    new PIDConstants(4.0, 0, 0.0) // Rotation PID constants
                    ),
                ppRobotConfigSupplier.get(),
                this)
            .beforeStarting(
                () -> {
                  poseEstimator.setTargetPoseOnField(
                      pose.get().transformBy(PathPlanOffset), "Auto Drive Pose");
                  SmartDashboard.putBoolean("Running AutoDrive", true);
                })
            .until(() -> getPose().getTranslation().getDistance(pose.get().getTranslation()) < 2.2)
            .andThen(
                finishDriving
                    .get()
                    .withTimeout(timeout)
                    .beforeStarting(
                        () -> {
                          // poseEstimator.setState(State.ENABLED_FIELD);
                          poseEstimator.setTargetPoseOnField(pose.get(), "Auto Drive Pose");
                        }))
            .finallyDo(
                () -> {
                  // poseEstimator.setState(State.ENABLED_ENV);
                  SmartDashboard.putBoolean("Running AutoDrive", false);
                });
  }

  public Supplier<Command> newDriveToPoseAuton(
      Supplier<Pose2d> pose,
      Transform2d PathPlanOffset,
      Time timeout,
      PathConstraints driveConstraints,
      double maxAcceleration,
      DoubleSupplier distance,
      DoubleSupplier stoppingDistance) {
    Supplier<Pose3d> pose3D = () -> new Pose3d(pose.get());
    Supplier<DriveToPoseSupplier> finishDriving =
        () ->
            new DriveToPoseSupplier(
                    this,
                    this::getPose,
                    () -> pose3D.get().toPose2d(),
                    new Transform2d(),
                    maxAcceleration)
                .withInitialVelocity(() -> getFieldVelocity()); // TO-DO: Change to
    // Distance-Based
    // PID+
    // Create the constraints to use while pathfinding
    // PathConstraints constraints = new PathConstraints(
    // swerveDrive.getMaximumChassisVelocity(), 4.0,
    // swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return () ->
        new PathFinderCommand(
                pose.get().transformBy(PathPlanOffset),
                driveConstraints,
                0.0,
                this::getPose,
                this::getFieldVelocity,
                this::setChassisSpeedsWithAngleSupplier,
                new PPHolonomicDriveController( // PPHolonomicController is the built in path
                    // following controller for
                    // holonomic
                    // drive trains
                    new PIDConstants(4.0, 0, 0.0), // Translation PID constants
                    new PIDConstants(4.0, 0, 0.0) // Rotation PID constants
                    ),
                ppRobotConfigSupplier.get(),
                this)
            .beforeStarting(
                () -> {
                  poseEstimator.setTargetPoseOnField(
                      pose.get().transformBy(PathPlanOffset), "Auto Drive Pose");
                  SmartDashboard.putBoolean("Running AutoDrive", true);
                })
            .until(() -> Math.abs(stoppingDistance.getAsDouble() - distance.getAsDouble()) < 0.1)
            .andThen(
                finishDriving
                    .get()
                    .withTimeout(timeout)
                    .beforeStarting(
                        () -> {
                          poseEstimator.setTargetPoseOnField(pose.get(), "Auto Drive Pose");
                        }))
            .finallyDo(
                () -> {
                  SmartDashboard.putBoolean("Running AutoDrive", false);
                });
  }

  public Supplier<Command> driveToPosePrecise(Supplier<Pose2d> pose) {
    return driveToPosePrecise(pose, new Transform2d());
  }

  public Supplier<Command> driveToPosePrecise(Pose2d pose, Transform2d PathPlanOffset) {
    return driveToPosePrecise(() -> pose, PathPlanOffset);
  }

  public ChassisSpeeds getFieldVelocitiesFromJoystick(
      DoubleSupplier xSpdFunction, DoubleSupplier ySpdFunction, DoubleSupplier turnSpdFunction) {
    double xInput = (xSpdFunction.getAsDouble());
    double yInput = (ySpdFunction.getAsDouble());

    if (Double.isNaN(xInput)
        || Double.isNaN(yInput)
        || Double.isNaN(turnSpdFunction.getAsDouble())) {
      // Input is NaN — fall through to previous-value substitution below
    }

    xInput = Double.isNaN(xInput) ? previousLeftXInput : xInput;
    yInput = Double.isNaN(yInput) ? previousLeftYInput : yInput;

    Translation2d inputTranslation = new Translation2d(xInput, yInput);
    double magnitude = inputTranslation.getNorm();
    Rotation2d angle = 0 != xInput || 0 != yInput ? inputTranslation.getAngle() : new Rotation2d();

    double curvedMagnitude = Math.pow(magnitude, 3);

    double turnSpeed = (turnSpdFunction.getAsDouble());
    turnSpeed = Double.isNaN(turnSpeed) ? previousRightXInput : turnSpeed;

    // limit power
    double xSpeed =
        curvedMagnitude
            * angle.getCos()
            * getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond();
    double ySpeed =
        curvedMagnitude
            * angle.getSin()
            * getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond();
    turnSpeed = turnSpeed * getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond();

    // convert to chassis speed class
    ChassisSpeeds chassisSpeeds;

    chassisSpeeds =
        new ChassisSpeeds(
            DriverStation.getAlliance().get() == Alliance.Red && isFieldOrientedDrive.getValue()
                ? -xSpeed
                : xSpeed,
            DriverStation.getAlliance().get() == Alliance.Red && isFieldOrientedDrive.getValue()
                ? -ySpeed
                : ySpeed,
            turnSpeed);

    previousLeftXInput = xInput;
    previousLeftYInput = yInput;
    previousRightXInput = turnSpeed;

    if (Double.isNaN(xInput)
        || Double.isNaN(yInput)
        || Double.isNaN(turnSpdFunction.getAsDouble())) {
      System.out.println("Input Problems");
    }

    return chassisSpeeds;
  }

  /**
   * Drive with {@link SwerveSetpointGenerator} from 254, implemented by PathPlanner.
   *
   * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to achieve.
   * @return {@link Command} to run.
   * @throws IOException If the PathPlanner GUI settings is invalid
   * @throws ParseException If PathPlanner GUI settings is nonexistent.
   */
  public Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
      throws IOException, ParseException {
    SwerveSetpointGenerator5010 setpointGenerator =
        new SwerveSetpointGenerator5010(
            ppRobotConfigSupplier.get(),
            swerveDrive.getMaximumModuleAngleVelocity().in(RadiansPerSecond));

    AtomicReference<SwerveSetpoint> prevSetpoint =
        new AtomicReference<>(
            new SwerveSetpoint(
                swerveDrive.getRobotVelocity(),
                swerveDrive.getStates(),
                DriveFeedforwards.zeros(swerveDrive.getModulesInfo().length)));

    AtomicReference<Double> previousTime = new AtomicReference<>();

    PathConstraints5010 constraints =
        new PathConstraints5010(
            MetersPerSecond.of(getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond()),
            MetersPerSecondPerSecond.of(
                getSwerveConstants().getkTeleDriveMaxAccelerationUnitsPerSecond()),
            (Supplier<LinearAcceleration>)
                () -> MetersPerSecondPerSecond.of(maxForwardAcceleration.get()),
            (Supplier<LinearAcceleration>)
                () -> MetersPerSecondPerSecond.of(maxBackwardAcceleration.get()),
            (Supplier<LinearAcceleration>)
                () -> MetersPerSecondPerSecond.of(maxRightAcceleration.get()),
            (Supplier<LinearAcceleration>)
                () -> MetersPerSecondPerSecond.of(maxLeftAcceleration.get()),
            (Supplier<LinearVelocity>) () -> MetersPerSecond.of(maxForwardVelocity.get()),
            (Supplier<LinearVelocity>) () -> MetersPerSecond.of(maxBackwardVelocity.get()),
            (Supplier<LinearVelocity>) () -> MetersPerSecond.of(maxLeftVelocity.get()),
            (Supplier<LinearVelocity>) () -> MetersPerSecond.of(maxRightVelocity.get()),
            RadiansPerSecond.of(
                getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond()),
            RadiansPerSecondPerSecond.of(
                getSwerveConstants().getkTeleDriveMaxAngularAccelerationUnitsPerSecond()),
            Volts.of(12),
            false);

    return startRun(
        () -> {
          previousTime.set(Timer.getFPGATimestamp());
          prevSetpoint.set(
              new SwerveSetpoint(
                  swerveDrive.getRobotVelocity(),
                  swerveDrive.getStates(),
                  DriveFeedforwards.zeros(swerveDrive.getModulesInfo().length)));
        },
        () -> {
          double newTime = Timer.getFPGATimestamp();
          SwerveSetpoint newSetpoint =
              setpointGenerator.generateSetpoint(
                  prevSetpoint.get(),
                  robotRelativeChassisSpeed.get(),
                  constraints,
                  newTime - previousTime.get());
          swerveDrive.drive(
              newSetpoint.robotRelativeSpeeds(),
              newSetpoint.moduleStates(),
              newSetpoint.feedforwards().linearForces());

          prevSetpoint.set(newSetpoint);
          previousTime.set(newTime);
        });
  }

  /**
   * Drive with 254's Setpoint generator; port written by PathPlanner.
   *
   * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
   * @return Command to drive the robot using the setpoint generator.
   */
  public Command driveWithSetpointGeneratorFieldRelative(
      Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
    try {
      return driveWithSetpointGenerator(
          () -> {
            ChassisSpeeds speeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());
            return speeds;
          });
    } catch (Exception e) {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();
  }

  public Command driveWithSetpointGeneratorOrientationConsidered(
      Supplier<ChassisSpeeds> inputSpeeds) {
    try {
      return driveWithSetpointGenerator(
          () -> {
            ChassisSpeeds speeds =
                isFieldOrientedDrive.getValue()
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(inputSpeeds.get(), getHeading())
                    : inputSpeeds.get();
            return speeds;
          });
    } catch (Exception e) {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();
  }

  public Command createDefaultCommand(Controller driverXbox) {
    DoubleSupplier leftX = () -> driverXbox.getLeftXAxis();
    DoubleSupplier leftY = () -> driverXbox.getLeftYAxis();
    DoubleSupplier rightX = () -> driverXbox.getRightXAxis();
    BooleanSupplier isFieldOriented = () -> isFieldOrientedDrive.getValue();

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
     * velocity.
     */
    // SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveDrive,
    // leftY,
    // leftX)
    // .withControllerRotationAxis(rightX)
    // .deadband(0.07)
    // .scaleTranslation(0.8)
    // .allianceRelativeControl(true);

    // return driveFieldOriented(driveAngularVelocity);
    return new JoystickToSwerve(
        this, leftY, leftX, rightX, isFieldOriented, () -> GenericRobot.getAlliance());
  }

  public Command createDefaultTestCommand(Controller driverXbox) {
    DoubleSupplier leftX = () -> driverXbox.getLeftXAxis();
    DoubleSupplier leftY = () -> driverXbox.getLeftYAxis();
    DoubleSupplier rightX = () -> driverXbox.getRightXAxis();
    BooleanSupplier isFieldOriented = () -> isFieldOrientedDrive.getValue();

    driverXbox.createAButton().whileTrue(sysIdDriveMotorCommand());
    // driverXbox.createBButton().whileTrue(sysIdAngleMotorCommand());
    // return Commands.run(() -> SwerveDriveTest.centerModules(swerveDrive), this);
    return new JoystickToSwerve(
        this, leftY, leftX, rightX, isFieldOriented, () -> GenericRobot.getAlliance());
  }

  public void stop() {
    swerveDrive.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
  }

  public Command sysIdDriveMotorCommand() {
    return swerveDrive.sysIdDriveMotorCommand(this);
  }

  public Command sysIdAngleMotorCommand() {
    return swerveDrive.sysIdAngleMotorCommand(this);
  }

  public void resetEncoders() {
    swerveDrive.resetEncoders();
  }

  public double getGyroRate() {
    return swerveDrive.getGyroRate();
  }

  public SwerveModulePosition[] getModulePositions() {
    return swerveDrive.getModulePositions();
  }

  public SwerveModuleState[] getStates() {
    return swerveDrive.getStates();
  }

  public void drive(
      ChassisSpeeds robotRelativeSpeeds, SwerveModuleState[] moduleStates, Force[] linearForces) {
    swerveDrive.drive(robotRelativeSpeeds, moduleStates, linearForces);
  }

  @Override
  public void periodic() {
    super.periodic();
    swerveDrive.periodic();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
    swerveDrive.updateSimulation();
  }
}
