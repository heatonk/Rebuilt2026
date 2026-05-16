package frc.robot.rebuilt.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.FieldConstants;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.commands.IntakeCommands;
import frc.robot.rebuilt.subsystems.drive.StubDrivetrain;
import frc.robot.rebuilt.util.TorqueCurrentArmSupport;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.SmartMotorFactory;

public class IntakeIOReal implements IntakeIO {
  protected FlyWheel spintakeInner;
  protected FlyWheel spintakeOuter;
  protected Arm intakeHopper;
  private TalonFX hopperTalonFX;
  private final MotionMagicTorqueCurrentFOC hopperMotionMagicRequest =
      new MotionMagicTorqueCurrentFOC(0).withSlot(0);
  private Angle hopperAngleSetpoint = Degrees.of(0.0);
  private TorqueCurrentArmSupport.Config hopperTorqueCurrentConfig =
      TorqueCurrentArmSupport.Config.defaults(false);
  protected StubDrivetrain drivetrain;
  private IntakeCommands.IntakeState lastState = IntakeCommands.IntakeState.RETRACTED;

  public IntakeIOReal(SubsystemBase parent) {
    spintakeOuter = buildSpintakeOuter(parent);
    spintakeInner = buildSpintakeInner(parent);
    intakeHopper = buildHopper(parent);
    drivetrain = Rebuilt.drivetrain;

    intakeHopper.getMotorController().setStatorCurrentLimit(Amps.of(100));
    intakeHopper.getMotorController().setSupplyCurrentLimit(Amps.of(30));
    hopperTorqueCurrentConfig = TorqueCurrentArmSupport.Config.defaults(false);
    hopperAngleSetpoint = intakeHopper.getAngle();

    Object rawController = intakeHopper.getMotorController().getMotorController();
    if (!RobotBase.isSimulation()
        && hopperTorqueCurrentConfig.useTorqueCurrentFOC()
        && rawController instanceof TalonFX talonFX) {
      hopperTalonFX = talonFX;
      TorqueCurrentArmSupport.syncSlot0Feedforward(intakeHopper, hopperTalonFX);
    }
  }

  private static Arm buildHopper(SubsystemBase parent) {
    TalonFX leader = new TalonFX(Constants.Intake.Hopper.CAN_ID);
    TalonFX follower = new TalonFX(Constants.Intake.Hopper.FOLLOWER_CAN_ID);
    DCMotor motorSim = DCMotor.getKrakenX44(2);

    @SuppressWarnings("unchecked")
    Pair<Object, Boolean>[] followers =
        new Pair[] {new Pair<>(follower, Constants.Intake.Hopper.FOLLOWER_INVERTED)};

    SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(parent)
            .withSoftLimit(
                Degrees.of(Constants.Intake.Hopper.LOWER_SOFT_LIMIT_DEG),
                Degrees.of(Constants.Intake.Hopper.UPPER_SOFT_LIMIT_DEG))
            .withFeedforward(
                new ArmFeedforward(
                    Constants.Intake.Hopper.KS,
                    Constants.Intake.Hopper.KG,
                    Constants.Intake.Hopper.KV,
                    Constants.Intake.Hopper.KA))
            .withSimFeedforward(
                new ArmFeedforward(
                    Constants.Intake.Hopper.SIM_KS,
                    Constants.Intake.Hopper.SIM_KG,
                    Constants.Intake.Hopper.SIM_KV,
                    Constants.Intake.Hopper.SIM_KA))
            .withClosedLoopController(
                Constants.Intake.Hopper.KP,
                Constants.Intake.Hopper.KI,
                Constants.Intake.Hopper.KD,
                DegreesPerSecond.of(Constants.Intake.Hopper.MAX_VEL_DEG_PER_SEC),
                DegreesPerSecondPerSecond.of(Constants.Intake.Hopper.MAX_ACCEL_DEG_PER_SEC_SQ))
            .withSimClosedLoopController(
                Constants.Intake.Hopper.SIM_KP,
                Constants.Intake.Hopper.SIM_KI,
                Constants.Intake.Hopper.SIM_KD,
                DegreesPerSecond.of(Constants.Intake.Hopper.SIM_MAX_VEL_DEG_PER_SEC),
                DegreesPerSecondPerSecond.of(Constants.Intake.Hopper.SIM_MAX_ACCEL_DEG_PER_SEC_SQ))
            .withGearing(new MechanismGearing(GearBox.fromStages(Constants.Intake.Hopper.GEAR_STAGES)))
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("hopperMotor", TelemetryVerbosity.LOW)
            .withStatorCurrentLimit(Amps.of(Constants.Intake.Hopper.CURRENT_LIMIT_AMPS))
            .withMotorInverted(Constants.Intake.Hopper.INVERTED)
            .withFollowers(followers);

    SmartMotorController smartMotor =
        SmartMotorFactory.create(leader, motorSim, motorConfig)
            .orElseThrow(() -> new RuntimeException("Failed to build hopper SmartMotorController"));

    ArmConfig armConfig =
        new ArmConfig(smartMotor)
            .withLength(Meters.of(Constants.Intake.Hopper.LENGTH_METERS))
            .withHardLimit(
                Degrees.of(Constants.Intake.Hopper.LOWER_HARD_LIMIT_DEG),
                Degrees.of(Constants.Intake.Hopper.UPPER_HARD_LIMIT_DEG))
            .withTelemetry("hopper", TelemetryVerbosity.LOW)
            .withMass(Pounds.of(Constants.Intake.Hopper.MASS_LBS))
            .withStartingPosition(Degrees.of(Constants.Intake.Hopper.STARTING_ANGLE_DEG))
            .withHorizontalZero(Degrees.of(Constants.Intake.Hopper.HORIZONTAL_ZERO_DEG));
    return new Arm(armConfig);
  }

  private static FlyWheel buildSpintakeInner(SubsystemBase parent) {
    return buildSpintake(
        parent,
        "spintake_inner",
        Constants.Intake.SpintakeInner.CAN_ID,
        Constants.Intake.SpintakeInner.INVERTED,
        Constants.Intake.SpintakeInner.GEAR_STAGES,
        Constants.Intake.SpintakeInner.MASS_KG,
        Constants.Intake.SpintakeInner.RADIUS_M,
        Constants.Intake.SpintakeInner.LOWER_SOFT_LIMIT_RPM,
        Constants.Intake.SpintakeInner.UPPER_SOFT_LIMIT_RPM,
        Constants.Intake.SpintakeInner.KP,
        Constants.Intake.SpintakeInner.KI,
        Constants.Intake.SpintakeInner.KD,
        Constants.Intake.SpintakeInner.KS,
        Constants.Intake.SpintakeInner.KV,
        Constants.Intake.SpintakeInner.KA);
  }

  private static FlyWheel buildSpintakeOuter(SubsystemBase parent) {
    return buildSpintake(
        parent,
        "spintake_outer",
        Constants.Intake.SpintakeOuter.CAN_ID,
        Constants.Intake.SpintakeOuter.INVERTED,
        Constants.Intake.SpintakeOuter.GEAR_STAGES,
        Constants.Intake.SpintakeOuter.MASS_KG,
        Constants.Intake.SpintakeOuter.RADIUS_M,
        Constants.Intake.SpintakeOuter.LOWER_SOFT_LIMIT_RPM,
        Constants.Intake.SpintakeOuter.UPPER_SOFT_LIMIT_RPM,
        Constants.Intake.SpintakeOuter.KP,
        Constants.Intake.SpintakeOuter.KI,
        Constants.Intake.SpintakeOuter.KD,
        Constants.Intake.SpintakeOuter.KS,
        Constants.Intake.SpintakeOuter.KV,
        Constants.Intake.SpintakeOuter.KA);
  }

  private static FlyWheel buildSpintake(
      SubsystemBase parent,
      String name,
      int canId,
      boolean inverted,
      String gearStages,
      double massKg,
      double radiusM,
      double lowerSoftLimitRpm,
      double upperSoftLimitRpm,
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA) {
    TalonFX motor = new TalonFX(canId);
    DCMotor motorSim = DCMotor.getKrakenX60(1);

    SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(parent)
            .withFeedforward(new SimpleMotorFeedforward(kS, kV, kA))
            .withSimFeedforward(new SimpleMotorFeedforward(kS, kV, kA))
            .withClosedLoopController(kP, kI, kD)
            .withSimClosedLoopController(kP, kI, kD)
            .withGearing(new MechanismGearing(GearBox.fromStages(gearStages)))
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry(name + "Motor", TelemetryVerbosity.LOW)
            .withMotorInverted(inverted);

    SmartMotorController smartMotor =
        SmartMotorFactory.create(motor, motorSim, motorConfig)
            .orElseThrow(() -> new RuntimeException("Failed to build " + name + " SmartMotorController"));

    edu.wpi.first.units.measure.Distance radius = Meters.of(radiusM);
    edu.wpi.first.units.measure.Mass mass = edu.wpi.first.units.Units.Kilograms.of(massKg);

    FlyWheelConfig shooterConfig =
        new FlyWheelConfig(smartMotor)
            .withDiameter(radius.times(2.0))
            .withMass(mass)
            .withUpperSoftLimit(DegreesPerSecond.of(upperSoftLimitRpm * 6.0))
            .withLowerSoftLimit(DegreesPerSecond.of(lowerSoftLimitRpm * 6.0))
            .withSpeedometerSimulation(DegreesPerSecond.of(upperSoftLimitRpm * 6.0))
            .withTelemetry(name, TelemetryVerbosity.LOW);
    shooterConfig.withMOI(radius, mass);
    return new FlyWheel(shooterConfig);
  }

  @Override
  public void runSpintake(double speed) {
    spintakeOuter.getMotor().setDutyCycle(speed);
    spintakeInner.getMotor().setDutyCycle(Constants.Intake.INTAKE_INNER_IN);
  }

  public void runSpintakes(double outerSpeed, double innerSpeed) {
    spintakeOuter.getMotor().setDutyCycle(outerSpeed);
    spintakeInner.getMotor().setDutyCycle(innerSpeed);
  }

  public Command setHopperAngle(Angle angle) {
    return Commands.runOnce(() -> requestHopperAngle(angle));
  }

  public void setHopperPosition(Angle angle) {
    intakeHopper.getMotor().setEncoderPosition(angle);
  }

  public boolean isHopperMoving() {
    return Math.abs(
            intakeHopper.getMotorController().getMechanismVelocity().in(Degrees.per(Second)))
        > Constants.Intake.HOPPER_MOVING_VELOCITY_THRESHOLD;
  }

  public boolean isHopperStalling() {
    return Math.abs(intakeHopper.getMotor().getStatorCurrent().in(Amps))
        > Constants.Intake.HOPPER_STALL_CURRENT_THRESHOLD;
  }

  public boolean isRetracted() {
    return (intakeHopper.getAngle().gte(Constants.Intake.HOPPER_RETRACTED_ANGLE));
  }

  public boolean isDeployed() {
    return (intakeHopper.getAngle().lte(Degrees.of(2.0)));
  }

  public Command getHopperSysIdCommand() {
    return intakeHopper.sysId(Volts.of(4), Volts.of(0.5).per(Seconds), Seconds.of(8));
  }

  public Command getHopperCharacterizationCommand(SubsystemBase intake) {
    return intakeHopper.sysId(Volts.of(4), Volts.of(0.5).per(Seconds), Seconds.of(8));
  }

  public void runHopper(double speed) {
    intakeHopper.getMotorController().setDutyCycle(speed);
  }

  public boolean isNearTrench() {
    Pose2d current = drivetrain.getPoseEstimator().getCurrentPose();
    double currentX = current.getX();
    double currentY = current.getY();

    double topTrenchLeftX = FieldConstants.TrenchZoneTop.nearAllianceLeftDanger.getX();
    double topTrenchRightX = FieldConstants.TrenchZoneTop.nearAllianceRightDanger.getX();

    double topTrenchY = FieldConstants.TrenchZoneTop.nearAllianceLeftDanger.getY();

    double topOppTrenchLeftX = FieldConstants.TrenchZoneTop.oppAllianceLeftDanger.getX();
    double topOppTrenchRightX = FieldConstants.TrenchZoneTop.oppAllianceRightDanger.getX();

    double lowerTrenchLeftX = FieldConstants.TrenchZoneBottom.nearAllianceLeftDanger.getX();
    double lowerTrenchRightX = FieldConstants.TrenchZoneBottom.nearAllianceRightDanger.getX();

    double lowerTrenchY = FieldConstants.TrenchZoneBottom.oppAllianceLeftDanger.getY();

    double lowerOppTrenchLeftX = FieldConstants.TrenchZoneBottom.oppAllianceLeftDanger.getX();
    double lowerOppTrenchRightX = FieldConstants.TrenchZoneBottom.oppAllianceRightDanger.getX();

    boolean nearAllianceTop =
        ((currentX > topTrenchLeftX && currentX < topTrenchRightX) && currentY > topTrenchY);

    boolean nearOppAllianceTop =
        ((currentX > topOppTrenchLeftX && currentX < topOppTrenchRightX) && currentY > topTrenchY);

    boolean nearAllianceBottom =
        ((currentX > lowerTrenchLeftX && currentX < lowerTrenchRightX) && currentY < lowerTrenchY);

    boolean nearOppAllianceBottom =
        ((currentX > lowerOppTrenchLeftX && currentX < lowerOppTrenchRightX)
            && currentY < lowerTrenchY);

    SmartDashboard.putBoolean("Near Top Opp Alliance", nearOppAllianceTop);
    SmartDashboard.putBoolean("Near Top Alliance", nearAllianceTop);
    SmartDashboard.putBoolean("Near Bottom Opp Alliance", nearOppAllianceBottom);
    SmartDashboard.putBoolean("Near Bottom Alliance", nearAllianceBottom);

    return nearAllianceTop || nearOppAllianceTop || nearAllianceBottom || nearOppAllianceBottom;
  }

  public double getDegreesDifference(Angle angleOne, Angle angleTwo) {
    return MathUtil.inputModulus(angleOne.minus(angleTwo).in(Degrees), -180, 180);
  }

  public boolean isHopperAtLocation(Angle location) {
    return getDegreesDifference(intakeHopper.getMotorController().getMechanismPosition(), location)
        < Constants.Intake.HOPPER_ANGLE_TOLERANCE;
  }

  private void requestHopperAngle(Angle angle) {
    hopperAngleSetpoint = angle;
    intakeHopper.getMotorController().setPosition(angle);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    double hopperVelocityDegreesPerSecond =
        intakeHopper.getMotorController().getMechanismVelocity().in(Degrees.per(Second));
    double hopperAmps = intakeHopper.getMotor().getStatorCurrent().in(Amps);

    inputs.hopperAngleActual = intakeHopper.getMotorController().getMechanismPosition();
    inputs.hopperAngleDegrees = inputs.hopperAngleActual.in(Degrees);
    inputs.hopperVelocityDegreesPerSecond = hopperVelocityDegreesPerSecond;
    inputs.hopperAngleDesired =
        hopperTalonFX != null
            ? hopperAngleSetpoint
            : intakeHopper
                .getMotorController()
                .getMechanismPositionSetpoint()
                .orElse(Degrees.of(0));
    inputs.hopperAngleError = inputs.hopperAngleDesired.minus(inputs.hopperAngleActual).in(Degrees);
    inputs.hopperAtGoal =
        MathUtil.inputModulus(inputs.hopperAngleError, -180, 180)
            < Constants.Intake.HOPPER_ANGLE_TOLERANCE;
    inputs.speed = spintakeOuter.getMotor().getDutyCycle();
    inputs.hopperAmps = hopperAmps;
    inputs.hopperMoving =
        Math.abs(hopperVelocityDegreesPerSecond)
            > Constants.Intake.HOPPER_MOVING_VELOCITY_THRESHOLD;
    inputs.hopperStalling = Math.abs(hopperAmps) > Constants.Intake.HOPPER_STALL_CURRENT_THRESHOLD;
    inputs.hopperHardStopDetected = inputs.hopperStalling;

    Logger.recordOutput("Hopper Velocity", hopperVelocityDegreesPerSecond);
    Logger.recordOutput("Hopper Moving", inputs.hopperMoving);
    Logger.recordOutput("Hopper Hard Stop", inputs.hopperHardStopDetected);
  }
}
