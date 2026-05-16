package frc.robot.rebuilt.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.frc5010.common.sensors.Controller;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import swervelib.simulation.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

// TODO real swerve — placeholder so the rest of the code compiles and runs without drive motion.
public class StubDrivetrain extends SubsystemBase {

  private final Field2d field2d = new Field2d();
  private final PoseEstimator poseEstimator = new PoseEstimator();

  public PoseEstimator getPoseEstimator() {
    return poseEstimator;
  }

  public void drive(ChassisSpeeds speeds) {}

  public void toggleFieldOrientedDrive() {}

  public void configureButtonBindings(Controller driver, Controller operator) {}

  public Command createDefaultCommand(Controller driver) {
    return Commands.none();
  }

  public void setAutoBuilder() {}

  public Command generateAutoCommand(Command autoCommand) {
    return autoCommand;
  }

  public void addAutoCommands(LoggedDashboardChooser<Command> selectableCommand) {}

  public ChassisSpeeds getFieldVelocity() {
    return new ChassisSpeeds();
  }

  public ChassisSpeeds getFieldAcceleration() {
    return new ChassisSpeeds();
  }

  public Field2d getField2d() {
    return field2d;
  }

  public static Optional<AbstractDriveTrainSimulation> getMapleSimDrive() {
    return Optional.empty();
  }

  public static class PoseEstimator {
    public Pose2d getCurrentPose() {
      return Pose2d.kZero;
    }

    public Pose3d getCurrentPose3d() {
      return Pose3d.kZero;
    }
  }
}
