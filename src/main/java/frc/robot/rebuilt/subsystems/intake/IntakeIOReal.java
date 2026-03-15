package frc.robot.rebuilt.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.FieldConstants;
import frc.robot.rebuilt.commands.IntakeCommands;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.motors.SystemIdentification;
import org.frc5010.common.motors.function.PercentControlMotor;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.positional.Arm;

public class IntakeIOReal implements IntakeIO {
  protected Map<String, Object> devices;
  // private FlyWheel spintakeLead;
  private PercentControlMotor spintakeLead;
  private PercentControlMotor spinTakeFollow;
  private Arm intakeHopper;
  protected GenericDrivetrain drivetrain;
  private boolean isNearTrench = false;
  private IntakeCommands.IntakeState lastState = IntakeCommands.IntakeState.RETRACTED;

  /** initializes the spintake and hopper */
  public IntakeIOReal(Map<String, Object> devices) {
    this.devices = devices;
    // spintakeLead = (FlyWheel) devices.get("spintake");
    spintakeLead = (PercentControlMotor) devices.get("spintakeLead");
    spinTakeFollow = (PercentControlMotor) devices.get("spintakeFollow");
    spintakeLead.invert(true);
    spinTakeFollow.setFollow(spintakeLead, false);
    intakeHopper = (Arm) devices.get("hopper");
  }

  @Override
  public void runSpintake(double speed) {
    // spintakeLead.getMotor().setDutyCycle(speed);
    spintakeLead.set(speed);
  }

  public void setHopperAngle(Angle angle) {
    intakeHopper.getMotorController().setPosition(angle);
  }

  public void setHopperPosition(Angle angle) {
    intakeHopper.getMotor().setEncoderPosition(angle);
  }

  public boolean isHopperMoving() {
    return Math.abs(
            intakeHopper.getMotorController().getMechanismVelocity().in(Degrees.per(Second)))
        > 10.0;
  }

  public boolean isHopperStalling() {
    return Math.abs(intakeHopper.getMotor().getStatorCurrent().in(Amps))
        > Constants.Intake.HOPPER_STALL_CURRENT_THRESHOLD;
  }

  public boolean isRetracted() {
    return false;
    // return (intakeHopper.getAngle().isNear(Degrees.of(120), Degrees.of(10)));
  }

  public boolean isDeployed() {
    return (intakeHopper.getAngle().isNear(Degrees.of(0.0), Degrees.of(10)));
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

  /** updates the input structure with the current hopper and intake speed */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    Logger.recordOutput(
        "Hopper Velocity",
        intakeHopper.getMotorController().getMechanismVelocity().in(Degrees.per(Second)));
    Logger.recordOutput("Hopper MOving", isHopperMoving());
    inputs.hopperAngle = intakeHopper.getMotorController().getMechanismPosition();
    inputs.hopperAngleDouble = inputs.hopperAngle.in(Degrees);
    inputs.speed = spintakeLead.get();
    inputs.hopperAmps = intakeHopper.getMotor().getStatorCurrent().in(Amps);
    // inputs.speed = spintakeLead.getMotor().getDutyCycle();
  }
}
