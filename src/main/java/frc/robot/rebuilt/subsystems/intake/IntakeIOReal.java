package frc.robot.rebuilt.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.FieldConstants;
import frc.robot.rebuilt.commands.IntakeCommands;
import frc.robot.rebuilt.util.TorqueCurrentArmSupport;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.motors.SystemIdentification;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;

public class IntakeIOReal implements IntakeIO {
  protected Map<String, Object> devices;
  private FlyWheel spintakeInner;
  private FlyWheel spintakeOuter;
  private Arm intakeHopper;
  private TalonFX hopperTalonFX;
  private final MotionMagicTorqueCurrentFOC hopperMotionMagicRequest =
      new MotionMagicTorqueCurrentFOC(0).withSlot(0);
  private Angle hopperAngleSetpoint = Degrees.of(0.0);
  private TorqueCurrentArmSupport.Config hopperTorqueCurrentConfig =
      TorqueCurrentArmSupport.Config.defaults(false);
  protected GenericDrivetrain drivetrain;
  private boolean isNearTrench = false;
  private IntakeCommands.IntakeState lastState = IntakeCommands.IntakeState.RETRACTED;

  /** initializes the spintake and hopper */
  public IntakeIOReal(Map<String, Object> devices) {
    this.devices = devices;
    // spintakeLead = (FlyWheel) devices.get("spintake");
    spintakeOuter = (FlyWheel) devices.get("spintake_outer");
    spintakeInner = (FlyWheel) devices.get("spintake_inner");
    intakeHopper = (Arm) devices.get("hopper");
    intakeHopper.getMotorController().setStatorCurrentLimit(Amps.of(100));
    intakeHopper.getMotorController().setSupplyCurrentLimit(Amps.of(30));
    hopperTorqueCurrentConfig =
        TorqueCurrentArmSupport.loadConfig("intake/hopper.json", false, "hopper");
    hopperAngleSetpoint = intakeHopper.getAngle();

    Object rawController = intakeHopper.getMotorController().getMotorController();
    if (!RobotBase.isSimulation()
        && hopperTorqueCurrentConfig.useTorqueCurrentFOC()
        && rawController instanceof TalonFX talonFX) {
      hopperTalonFX = talonFX;
      TorqueCurrentArmSupport.syncSlot0Feedforward(intakeHopper, hopperTalonFX);
    }
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
  /** Returns a sysid command for the hopper */
  public Command getHopperSysIdCommand(GenericSubsystem intake) {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.angleSysIdRoutine(
            intakeHopper.getMotorController(), intakeHopper.getName(), intake),
        5,
        5,
        3,
        () ->
            intakeHopper
                .isNear(
                    intakeHopper.getMotorController().getConfig().getMechanismUpperLimit().get(),
                    Degrees.of(10))
                .getAsBoolean(),
        () ->
            intakeHopper
                .isNear(
                    intakeHopper.getMotorController().getConfig().getMechanismLowerLimit().get(),
                    Degrees.of(10))
                .getAsBoolean(),
        () -> intakeHopper.getMotor().setDutyCycle(0));
  }

  public Command getHopperCharacterizationCommand(GenericSubsystem intake) {
    return SystemIdentification.feedforwardCharacterization(
        intake,
        (Voltage voltage) -> intakeHopper.getMotor().setVoltage(voltage),
        () -> intakeHopper.getMotorController().getMechanismVelocity().in(Degrees.per(Second)));
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

    if (nearAllianceTop || nearOppAllianceTop || nearAllianceBottom || nearOppAllianceBottom)
      return true;
    else {
      return false;
    }
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

  /** updates the input structure with the current hopper and intake speed */
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
    // inputs.speed = spintakeLead.getMotor().getDutyCycle();
  }
}
