package frc.robot.rebuilt.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.FieldConstants;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.commands.IntakeCommands;
import frc.robot.rebuilt.subsystems.drive.RebuiltDrivetrain;
import org.littletonrobotics.junction.Logger;

public class IntakeIOReal implements IntakeIO {
  private static final double HOPPER_GEAR_RATIO = 24.0;
  private static final double SPINTAKE_GEAR_RATIO = 11.0 / 36.0;

  protected TalonFX spintakeInner;
  protected TalonFX spintakeOuter;
  protected TalonFX hopperLeader;
  protected TalonFX hopperFollower;

  private final MotionMagicTorqueCurrentFOC hopperMotionMagicRequest =
      new MotionMagicTorqueCurrentFOC(0).withSlot(0);
  private final DutyCycleOut hopperDutyCycle = new DutyCycleOut(0);
  private final DutyCycleOut spintakeInnerDutyCycle = new DutyCycleOut(0);
  private final DutyCycleOut spintakeOuterDutyCycle = new DutyCycleOut(0);

  private Angle hopperAngleSetpoint = Degrees.of(0.0);

  protected RebuiltDrivetrain drivetrain;

  @SuppressWarnings("unused")
  private IntakeCommands.IntakeState lastState = IntakeCommands.IntakeState.RETRACTED;

  public IntakeIOReal(SubsystemBase parent) {
    spintakeOuter =
        buildSpintake(
            Constants.Intake.SpintakeOuter.CAN_ID, Constants.Intake.SpintakeOuter.INVERTED);
    spintakeInner =
        buildSpintake(
            Constants.Intake.SpintakeInner.CAN_ID, Constants.Intake.SpintakeInner.INVERTED);
    hopperLeader = buildHopperLeader();
    hopperFollower = buildHopperFollower(hopperLeader);
    drivetrain = Rebuilt.drivetrain;

    hopperAngleSetpoint = getHopperAngle();
  }

  private static TalonFX buildHopperLeader() {
    TalonFX motor = new TalonFX(Constants.Intake.Hopper.CAN_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        Constants.Intake.Hopper.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = HOPPER_GEAR_RATIO;

    Slot0Configs slot0 = config.Slot0;
    slot0.kP = Constants.Intake.Hopper.KP;
    slot0.kI = Constants.Intake.Hopper.KI;
    slot0.kD = Constants.Intake.Hopper.KD;
    slot0.kS = Constants.Intake.Hopper.KS;
    slot0.kV = Constants.Intake.Hopper.KV;
    slot0.kA = Constants.Intake.Hopper.KA;
    slot0.kG = Constants.Intake.Hopper.KG;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.MotionMagic.MotionMagicCruiseVelocity =
        Constants.Intake.Hopper.MAX_VEL_DEG_PER_SEC / 360.0;
    config.MotionMagic.MotionMagicAcceleration =
        Constants.Intake.Hopper.MAX_ACCEL_DEG_PER_SEC_SQ / 360.0;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Degrees.of(Constants.Intake.Hopper.UPPER_SOFT_LIMIT_DEG).in(Rotations);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Degrees.of(Constants.Intake.Hopper.LOWER_SOFT_LIMIT_DEG).in(Rotations);

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 100.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30.0;

    motor.getConfigurator().apply(config);
    motor.setPosition(Degrees.of(Constants.Intake.Hopper.STARTING_ANGLE_DEG).in(Rotations));
    return motor;
  }

  private static TalonFX buildHopperFollower(TalonFX leader) {
    TalonFX motor = new TalonFX(Constants.Intake.Hopper.FOLLOWER_CAN_ID);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Intake.Hopper.CURRENT_LIMIT_AMPS;
    motor.getConfigurator().apply(config);
    motor.setControl(
        new Follower(
            leader.getDeviceID(),
            Constants.Intake.Hopper.FOLLOWER_INVERTED
                ? MotorAlignmentValue.Opposed
                : MotorAlignmentValue.Aligned));
    return motor;
  }

  private static TalonFX buildSpintake(int canId, boolean inverted) {
    TalonFX motor = new TalonFX(canId);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = SPINTAKE_GEAR_RATIO;
    motor.getConfigurator().apply(config);
    return motor;
  }

  private Angle getHopperAngle() {
    return Rotations.of(hopperLeader.getPosition().getValueAsDouble());
  }

  private double getHopperVelocityDegPerSec() {
    return hopperLeader.getVelocity().getValueAsDouble() * 360.0;
  }

  @Override
  public void runSpintake(double speed) {
    spintakeOuter.setControl(spintakeOuterDutyCycle.withOutput(speed));
    spintakeInner.setControl(spintakeInnerDutyCycle.withOutput(Constants.Intake.INTAKE_INNER_IN));
  }

  public void runSpintakes(double outerSpeed, double innerSpeed) {
    spintakeOuter.setControl(spintakeOuterDutyCycle.withOutput(outerSpeed));
    spintakeInner.setControl(spintakeInnerDutyCycle.withOutput(innerSpeed));
  }

  public Command setHopperAngle(Angle angle) {
    return Commands.runOnce(() -> requestHopperAngle(angle));
  }

  public void setHopperPosition(Angle angle) {
    hopperLeader.setPosition(angle.in(Rotations));
  }

  public boolean isHopperMoving() {
    return Math.abs(getHopperVelocityDegPerSec())
        > Constants.Intake.HOPPER_MOVING_VELOCITY_THRESHOLD;
  }

  public boolean isHopperStalling() {
    return Math.abs(hopperLeader.getStatorCurrent().getValueAsDouble())
        > Constants.Intake.HOPPER_STALL_CURRENT_THRESHOLD;
  }

  public boolean isRetracted() {
    return getHopperAngle().gte(Constants.Intake.HOPPER_RETRACTED_ANGLE);
  }

  public boolean isDeployed() {
    return getHopperAngle().lte(Degrees.of(2.0));
  }

  public Command getHopperSysIdCommand() {
    return Commands.none();
  }

  public Command getHopperCharacterizationCommand(SubsystemBase intake) {
    return Commands.none();
  }

  public void runHopper(double speed) {
    hopperLeader.setControl(hopperDutyCycle.withOutput(speed));
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
    return getDegreesDifference(getHopperAngle(), location) < Constants.Intake.HOPPER_ANGLE_TOLERANCE;
  }

  private void requestHopperAngle(Angle angle) {
    hopperAngleSetpoint = angle;
    hopperLeader.setControl(hopperMotionMagicRequest.withPosition(angle.in(Rotations)));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    double hopperVelocityDegreesPerSecond = getHopperVelocityDegPerSec();
    double hopperAmps = hopperLeader.getStatorCurrent().getValueAsDouble();

    inputs.hopperAngleActual = getHopperAngle();
    inputs.hopperAngleDegrees = inputs.hopperAngleActual.in(Degrees);
    inputs.hopperVelocityDegreesPerSecond = hopperVelocityDegreesPerSecond;
    inputs.hopperAngleDesired = hopperAngleSetpoint;
    inputs.hopperAngleError = inputs.hopperAngleDesired.minus(inputs.hopperAngleActual).in(Degrees);
    inputs.hopperAtGoal =
        MathUtil.inputModulus(inputs.hopperAngleError, -180, 180)
            < Constants.Intake.HOPPER_ANGLE_TOLERANCE;
    inputs.speed = spintakeOuter.getDutyCycle().getValueAsDouble();
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
