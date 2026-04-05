// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.frc5010.common.drive.swerve.akit;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.frc5010.common.commands.AkitDriveCommands;
import org.frc5010.common.drive.pose.DrivePoseEstimator;
import org.frc5010.common.drive.pose.SwerveFunctionsPose;
import org.frc5010.common.drive.swerve.AkitSwerveConfig;
import org.frc5010.common.drive.swerve.GenericSwerveDrivetrain;
import org.frc5010.common.drive.swerve.GenericSwerveModuleInfo;
import org.frc5010.common.drive.swerve.SwerveDriveFunctions;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

public class AkitSwerveDrive extends SwerveDriveFunctions {
  private final AkitSwerveConfig config;
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private SysIdRoutine sysId;
  private Field2d field = new Field2d(); // For visualization in SmartDashboard
  private Supplier<RobotConfig> robotConfigSupplier = () -> null;

  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics;
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator;
  private final Consumer<Pose2d> resetSimulationPoseCallBack;

  // Pre-allocated odometry buffers — sized for max odometry samples per 20ms loop.
  // At 250 Hz odometry the theoretical max is ~5 samples; 16 provides ample headroom.
  private static final int MAX_ODOMETRY_SAMPLES = 16;
  /** Empty array sentinel for disabled-mode logging — never mutated */
  private static final SwerveModuleState[] EMPTY_MODULE_STATES = new SwerveModuleState[] {};

  private final SwerveModulePosition[][] odometryModulePositions =
      new SwerveModulePosition[MAX_ODOMETRY_SAMPLES][4];
  private final SwerveModulePosition[][] odometryModuleDeltas =
      new SwerveModulePosition[MAX_ODOMETRY_SAMPLES][4];

  {
    // Pre-populate every slot so no null checks are needed in the hot loop
    for (int s = 0; s < MAX_ODOMETRY_SAMPLES; s++) {
      for (int m = 0; m < 4; m++) {
        odometryModulePositions[s][m] = new SwerveModulePosition();
        odometryModuleDeltas[s][m] = new SwerveModulePosition();
      }
    }
  }

  public AkitSwerveDrive(
      AkitSwerveConfig config,
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Consumer<Pose2d> resetSimulationPoseCallBack) {
    this.config = config;
    kinematics = new SwerveDriveKinematics(getModuleTranslations());
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    this.gyroIO = gyroIO;
    this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
    modules[0] = new Module(flModuleIO, 0, config.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, config.FrontRight);
    modules[2] = new Module(blModuleIO, 2, config.BackLeft);
    modules[3] = new Module(brModuleIO, 3, config.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    OdometryThread.getInstance().start();

    // Should we keep this?
    // Pathfinding.setPathfinder(new LocalADStarAK());

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  public AkitSwerveConfig getConfig() {
    return config;
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", EMPTY_MODULE_STATES);
      Logger.recordOutput("SwerveStates/SetpointsOptimized", EMPTY_MODULE_STATES);
    }

    getChassisSpeeds();

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = Math.min(sampleTimestamps.length, MAX_ODOMETRY_SAMPLES);
    for (int i = 0; i < sampleCount; i++) {
      // Reuse pre-allocated position / delta arrays for this sample
      SwerveModulePosition[] modulePositions = odometryModulePositions[i];
      SwerveModulePosition[] moduleDeltas = odometryModuleDeltas[i];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        SwerveModulePosition freshPos = modules[moduleIndex].getOdometryPositions()[i];
        modulePositions[moduleIndex].distanceMeters = freshPos.distanceMeters;
        modulePositions[moduleIndex].angle = freshPos.angle;
        moduleDeltas[moduleIndex].distanceMeters =
            freshPos.distanceMeters - lastModulePositions[moduleIndex].distanceMeters;
        moduleDeltas[moduleIndex].angle = freshPos.angle;
        lastModulePositions[moduleIndex].distanceMeters = freshPos.distanceMeters;
        lastModulePositions[moduleIndex].angle = freshPos.angle;
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.CURRENT_MODE != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, config.getMaxDriveSpeed());

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output, config);
    }
  }

  /**
   * Runs the steer characterization routine on all modules. This command is used to measure the
   * feedforward constants of the steer motors.
   *
   * @param output The output to send to the modules in volts.
   */
  public void runSteerCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runSteerCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    Translation2d[] moduleTranslations = getModuleTranslations();
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    Logger.recordOutput("SwerveStates/Measured", states);
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  @Override
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds speeds = kinematics.toChassisSpeeds(getModuleStates());
    Logger.recordOutput("SwerveChassisSpeeds/Measured", speeds);
    return speeds;
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getDriveFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getDriveFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getSteerFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getSteerFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    resetSimulationPoseCallBack.accept(pose);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return config.getMaxDriveSpeed().in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / config.DRIVE_BASE_RADIUS;
  }

  @Override
  public Field2d getField2d() {
    return field;
  }

  @Override
  public DrivePoseEstimator initializePoseEstimator() {
    return new DrivePoseEstimator(new SwerveFunctionsPose(this));
  }

  @Override
  public ChassisSpeeds getRobotVelocity() {
    return getChassisSpeeds();
  }

  @Override
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation());
  }

  /**
   * Returns the robot-relative chassis acceleration derived from drive motor acceleration signals.
   * Uses the same kinematics math as velocity, but with per-module acceleration instead of
   * velocity.
   */
  public ChassisSpeeds getChassisAcceleration() {
    SwerveModuleState[] accelStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      accelStates[i] = modules[i].getAccelerationState();
    }
    return kinematics.toChassisSpeeds(accelStates);
  }

  /**
   * Returns the field-relative chassis acceleration derived from drive motor acceleration signals.
   */
  public ChassisSpeeds getFieldAcceleration() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisAcceleration(), getRotation());
  }

  @Override
  public void drive(ChassisSpeeds velocity, DriveFeedforwards feedforwards) {
    runVelocity(velocity);
  }

  @Override
  public void driveFieldOriented(ChassisSpeeds velocity) {
    runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(velocity, gyroInputs.yawPosition));
  }

  @Override
  public void driveRobotRelative(ChassisSpeeds velocity) {
    runVelocity(velocity);
  }

  @Override
  public void resetEncoders() {}

  @Override
  public double getGyroRate() {
    return Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec);
  }

  /**
   * Returns a SysIdRoutine instance configured for system identification of the drive.
   *
   * <p>If the instance variable sysId is null, a new SysIdRoutine instance is created with the
   * provided SubsystemBase and a default configuration. The default configuration includes a no-op
   * mechanism and a logger which records the state of the SysIdRoutine to the {@link Logger}.
   *
   * @param swerveSubsystem The subsystem to add to the requirements of the SysIdRoutine
   * @return A SysIdRoutine instance configured for system identification of the drive
   */
  protected SysIdRoutine getSysId(SubsystemBase swerveSubsystem) {
    if (null == sysId) {
      sysId =
          new SysIdRoutine(
              new SysIdRoutine.Config(
                  Volts.of(0.5).per(Second),
                  Volts.of(7),
                  Second.of(30),
                  (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
              new SysIdRoutine.Mechanism(
                  (voltage) -> runCharacterization(voltage.in(Volts)), null, swerveSubsystem));
    }
    return sysId;
  }

  @Override
  public Command sysIdDriveMotorCommand(SubsystemBase swerveSubsystem) {
    // Configure SysId
    return sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        .withTimeout(10)
        .andThen(Commands.waitSeconds(3))
        .andThen(sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        .withTimeout(10)
        .andThen(Commands.waitSeconds(3))
        .andThen(sysIdDynamic(SysIdRoutine.Direction.kForward))
        .withTimeout(4)
        .andThen(Commands.waitSeconds(3))
        .andThen(sysIdDynamic(SysIdRoutine.Direction.kReverse))
        .withTimeout(4);
  }

  @Override
  public Command sysIdAngleMotorCommand(SubsystemBase swerveSubsystem) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'sysIdAngleMotorCommand'");
  }

  @Override
  public void drive(
      ChassisSpeeds robotRelativeVelocity, SwerveModuleState[] states, Force[] feedforwardForces) {
    runVelocity(robotRelativeVelocity);
  }

  @Override
  public SwerveModuleState[] getStates() {
    return getModuleStates();
  }

  @Override
  public AngularVelocity getMaximumModuleAngleVelocity() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getMaximumModuleAngleVelocity'");
  }

  @Override
  public GenericSwerveModuleInfo[] getModulesInfo() {
    if (null == moduleInfos) {
      moduleInfos = new GenericSwerveModuleInfo[modules.length];
      for (int i = 0; i < modules.length; i++) {
        moduleInfos[i] = new GenericSwerveModuleInfo();
      }
    }
    for (int i = 0; i < modules.length; i++) {
      moduleInfos[i].update(modules[i]);
    }
    return moduleInfos;
  }

  /** Returns an array of module translations. */
  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(config.FrontLeft.LocationX, config.FrontLeft.LocationY),
      new Translation2d(config.FrontRight.LocationX, config.FrontRight.LocationY),
      new Translation2d(config.BackLeft.LocationX, config.BackLeft.LocationY),
      new Translation2d(config.BackRight.LocationX, config.BackRight.LocationY)
    };
  }

  /**
   * Retrieves the pose of the simulated drivetrain from the MapleSim system.
   *
   * @return The current pose of the simulated drivetrain as a {@link Pose2d}.
   */
  @Override
  public Pose2d getSimPose() {
    return driveSimulation.getSimulatedDriveTrainPose();
  }

  @Override
  public Supplier<RobotConfig> getPPRobotConfigSupplier() {
    return robotConfigSupplier;
  }

  @Override
  public void setPPRobotConfigSupplier(Supplier<RobotConfig> robotConfigSupplier) {
    this.robotConfigSupplier = robotConfigSupplier;
  }

  public void updateSimulation() {
    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }

  @Override
  public void addAutoCommands(
      LoggedDashboardChooser<Command> selectableCommand, GenericSwerveDrivetrain drivetrain) {
    selectableCommand.addOption(
        "PRO: Swerve Wheel Radius Characterization",
        AkitDriveCommands.wheelRadiusCharacterization(drivetrain, this));
    selectableCommand.addOption(
        "PRO: Swerve Drive Feedforward Characterization",
        AkitDriveCommands.feedforwardCharacterization(
            drivetrain,
            (Voltage voltage) -> runCharacterization(voltage.in(Volts)),
            () -> getDriveFFCharacterizationVelocity()));
    selectableCommand.addOption(
        "PRO: Swerve Steer Feedforward Characterization",
        AkitDriveCommands.feedforwardCharacterization(
            drivetrain,
            (Voltage voltage) -> runSteerCharacterization(voltage.in(Volts)),
            () -> getSteerFFCharacterizationVelocity()));

    selectableCommand.addOption(
        "PRO: Swerve Angle PID Tuning", AkitDriveCommands.steerPIDTuning(drivetrain, this));
    selectableCommand.addOption(
        "PRO: Swerve Drive PID Tuning", AkitDriveCommands.drivePIDTuning(drivetrain, this));
  }
}
