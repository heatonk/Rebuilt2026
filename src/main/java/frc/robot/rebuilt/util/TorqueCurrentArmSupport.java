package frc.robot.rebuilt.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.rebuilt.Rebuilt;
import java.io.File;
import java.io.IOException;
import org.frc5010.common.config.UnitsParser;
import org.frc5010.common.config.json.devices.YamsArmConfigurationJson;
import yams.mechanisms.positional.Arm;

public final class TorqueCurrentArmSupport {
  private static final ObjectMapper OBJECT_MAPPER = new ObjectMapper();
  private static final double RADIANS_PER_ROTATION = 2.0 * Math.PI;

  private TorqueCurrentArmSupport() {}

  public record Config(
      boolean useTorqueCurrentFOC, double gravityFeedforwardAmps, Angle horizontalZero) {
    public static Config defaults(boolean useTorqueCurrentFOC) {
      return new Config(useTorqueCurrentFOC, 0.0, Degrees.of(0.0));
    }
  }

  public static Config loadConfig(
      String relativeDevicePath, boolean defaultUseTorqueCurrentFOC, String mechanismName) {
    File configFile =
        new File(
            Filesystem.getDeployDirectory(),
            Rebuilt.configDirectory + "/subsystems/" + relativeDevicePath);
    if (!configFile.isFile()) {
      return Config.defaults(defaultUseTorqueCurrentFOC);
    }

    try {
      YamsArmConfigurationJson armConfig =
          OBJECT_MAPPER.readValue(configFile, YamsArmConfigurationJson.class);
      return new Config(
          armConfig.useTorqueCurrentFOC,
          armConfig.motorSystemId.feedForward.g,
          UnitsParser.parseAngle(armConfig.horizontalZero));
    } catch (IOException exception) {
      DriverStation.reportWarning(
          "Failed to read "
              + mechanismName
              + " TorqueCurrentFOC config, using safe defaults: "
              + exception.getMessage(),
          false);
      return Config.defaults(defaultUseTorqueCurrentFOC);
    }
  }

  public static void syncSlot0Feedforward(Arm arm, TalonFX talonFX) {
    ArmFeedforward feedforward =
        arm.getMotorController().getConfig().getArmFeedforward().orElse(null);
    if (feedforward == null) {
      return;
    }

    Slot0Configs slot0 = new Slot0Configs();
    talonFX.getConfigurator().refresh(slot0);
    talonFX
        .getConfigurator()
        .apply(
            slot0
                .withKS(feedforward.getKs())
                .withKV(feedforward.getKv() * RADIANS_PER_ROTATION)
                .withKA(feedforward.getKa() * RADIANS_PER_ROTATION));
  }

  public static double calculateGravityFeedforward(Angle targetAngle, Config config) {
    if (config.gravityFeedforwardAmps() == 0.0) {
      return 0.0;
    }

    return config.gravityFeedforwardAmps()
        * Math.cos(targetAngle.minus(config.horizontalZero()).in(Radians));
  }
}
