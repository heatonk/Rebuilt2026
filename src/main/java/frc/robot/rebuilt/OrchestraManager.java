package frc.robot.rebuilt;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Filesystem;

/** Manages CTRE Orchestra playback through all robot TalonFX motors during disabled mode. */
public class OrchestraManager {

  private final Orchestra orchestra = new Orchestra();

  // TalonFX CAN IDs on the standard rio bus
  private static final int[] RIO_IDS = {9, 10, 11, 12, 13, 14, 15, 16, 17, 19};

  // TalonFX CAN IDs on the CANivore bus
  private static final int[] CANIVORE_IDS = {1, 2, 3, 4, 5, 6, 7, 8, 18};

  private static final String MUSIC_FILE = "sea2.chrp";

  public OrchestraManager() {
    for (int id : RIO_IDS) {
      TalonFX motor = new TalonFX(id);
      AudioConfigs config = new AudioConfigs().withAllowMusicDurDisable(true);
      motor.getConfigurator().apply(config, 0);
      orchestra.addInstrument(motor);
    }
    for (int id : CANIVORE_IDS) {
      TalonFX motor = new TalonFX(id, "canivore");
      AudioConfigs config = new AudioConfigs().withAllowMusicDurDisable(true);
      motor.getConfigurator().apply(config);
      orchestra.addInstrument(motor, 0);
    }
    orchestra.loadMusic(Filesystem.getDeployDirectory().toPath().resolve(MUSIC_FILE).toString());
  }

  /** Start playing the mariachi song. Safe to call repeatedly. */
  public void play() {
    orchestra.play();
  }

  /** Stop playback and release motors back to normal control. */
  public void stop() {
    orchestra.stop();
  }
}
